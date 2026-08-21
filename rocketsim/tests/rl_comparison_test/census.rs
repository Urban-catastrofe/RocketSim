//! Error-mass census (`RLCENSUS=1`).
//!
//! The per-segment breakdown (`RLSEG=1`) shows the car velocity error is split
//! almost evenly between grounded and airborne ticks with near-identical means
//! (4.90 vs 4.58 uu/s) — but with an rms of 28-33, so a few percent of steps
//! carry nearly all of the mass. A mean per regime cannot say *which* steps
//! those are, and "airborne" covers both free flight (where the physics is
//! gravity plus a constant) and the tick a car clips a wall.
//!
//! This pass runs the same per-tick restore as the gate, then files every step
//! into exactly one bucket named after the *cause* available in the recording
//! (ball contact, another car nearby, wall/ceiling, a wheel-contact transition,
//! free flight, steady driving), and reports the error mass per bucket. The
//! bucket holding the mass is the subsystem worth fixing next.
//!
//! Buckets are assigned in priority order because causes overlap: a car
//! wall-riding into the ball is filed under the ball, since that is the larger
//! effect. Steps the gate already excludes as unmeasurable (jump/flip presses
//! and sub-frame bumps) are filed under `impulse` so their mass stays visible
//! instead of silently vanishing from the totals.
//!
//! `RLCENSUS=2` additionally lists each bucket's worst steps, for feeding into
//! `RLDEEP`.

use glam::{Mat3A, Vec3A};
use rocketsim::CarControls;

use super::recording::Recording;
use super::recording::cpp_records::{CarRecord, PhysRecord};
use super::runner::{has_discontinuity, is_car_sentinel, make_arena, set_state_to_record_tick};

/// Octane hitbox, inflated by the ball radius, as in `residual.rs`.
const HITBOX_HALF: Vec3A = Vec3A::new(120.507 / 2.0, 86.6994 / 2.0, 38.6591 / 2.0);
const HITBOX_OFFSET: Vec3A = Vec3A::new(13.8757, 0.0, 20.755);
const BALL_RADIUS: f32 = 91.25;
/// Slack (UU) on the ball contact test. Generous on purpose: a step one tick
/// before contact is still a step whose error the ball caused.
const BALL_SLACK: f32 = 30.0;
/// Centre separation (UU) within which two Octanes can touch, plus a margin —
/// the same figure `detect_bump_onsets` uses.
const CAR_PROXIMITY: f32 = 240.0;
/// A wheel contact normal this far off vertical is a wall or the ceiling.
const FLOOR_NORMAL_Z: f32 = 0.9;

fn rot_of(phys: &PhysRecord) -> Mat3A {
    Mat3A::from_cols(
        phys.rot.rows[0].into(),
        phys.rot.rows[1].into(),
        phys.rot.rows[2].into(),
    )
}

fn ball_touches_car(ball_pos: Vec3A, car: &PhysRecord) -> bool {
    let local = rot_of(car).transpose() * (ball_pos - Vec3A::from(car.pos)) - HITBOX_OFFSET;
    let d = Vec3A::new(
        (local.x.abs() - HITBOX_HALF.x).max(0.0),
        (local.y.abs() - HITBOX_HALF.y).max(0.0),
        (local.z.abs() - HITBOX_HALF.z).max(0.0),
    );
    d.length() <= BALL_RADIUS + BALL_SLACK
}

fn is_sentinel(phys: &PhysRecord) -> bool {
    phys.pos.z < -1000.0
}

fn wheels_in_contact(car: &CarRecord) -> usize {
    car.wheels.iter().filter(|w| w.has_contact).count()
}

/// Lowest wheel contact normal z among contacting wheels (1.0 if none).
fn min_contact_z(car: &CarRecord) -> f32 {
    car.wheels
        .iter()
        .filter(|w| w.has_contact)
        .map(|w| w.contact_normal.z.abs())
        .fold(1.0f32, f32::min)
}

#[derive(Default, Clone)]
struct Bucket {
    n: u64,
    sum: f64,
    sum_sq: f64,
    max: f32,
    max_tick: usize,
    /// Every magnitude in the bucket, for percentiles. A bucket's *shape*
    /// decides what kind of defect it is: a broad population with p50 near the
    /// mean is a systematic force error worth re-deriving a coefficient for,
    /// while p50 near zero and a long tail is a rare event being mishandled,
    /// where a coefficient sweep would just fit noise.
    mags: Vec<f32>,
    /// Worst few steps, for follow-up (`(mag, tick, car)`), largest first.
    worst: Vec<(f32, usize, usize)>,
    /// Signed residual (`sim - game`) summed in the car's own frame:
    /// (forward, right, up). A bucket whose error is a consistent *bias* along
    /// one body axis is a mis-calibrated force along that axis; one whose
    /// signed sum cancels to near zero is timing or noise, and recalibrating a
    /// coefficient against it would just fit noise.
    local_bias: glam::DVec3,
    /// Per body axis, `|sim . axis| - |game . axis|` summed: does the sim retain
    /// too much or too little speed along that axis, regardless of sign? A
    /// powerslide is symmetric — players slide left as often as right — so
    /// `local_bias` cancels there and only this statistic can see a lateral
    /// friction that is uniformly too weak or too strong.
    local_mag_bias: glam::DVec3,
}

/// How many worst-offender steps to remember per bucket.
const WORST_KEPT: usize = 6;

impl Bucket {
    fn add(&mut self, mag: f32, tick: usize, car: usize, local: Vec3A, mag_local: Vec3A) {
        self.local_bias += glam::DVec3::new(local.x as f64, local.y as f64, local.z as f64);
        self.local_mag_bias +=
            glam::DVec3::new(mag_local.x as f64, mag_local.y as f64, mag_local.z as f64);
        self.n += 1;
        self.sum += mag as f64;
        self.sum_sq += (mag as f64) * (mag as f64);
        self.mags.push(mag);
        if mag > self.max {
            self.max = mag;
            self.max_tick = tick;
        }
        self.worst.push((mag, tick, car));
        if self.worst.len() > WORST_KEPT * 4 {
            self.trim();
        }
    }

    fn trim(&mut self) {
        self.worst
            .sort_unstable_by(|a, b| b.0.partial_cmp(&a.0).unwrap());
        self.worst.truncate(WORST_KEPT);
    }

    /// Magnitude at `q` (0..1). Sorts `mags` in place.
    fn pct(&mut self, q: f64) -> f32 {
        if self.mags.is_empty() {
            return 0.0;
        }
        self.mags.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
        let idx = ((self.mags.len() - 1) as f64 * q).round() as usize;
        self.mags[idx]
    }
}

thread_local! {
    /// Set from `RLCENSUS=4`: band by `friction_curve_input` instead of speed.
    static BAND_BY_FCI: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Set from `RLCENSUS=5`: band by recorded suspension compression.
    static BAND_BY_SUSP: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Set from `RLCENSUS=6`: band by compression crossed with compression rate.
    static BAND_BY_SUSP_RATE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Set from `RLCENSUS=7`: 1-UU compression bands, quiet suspension only.
    static BAND_BY_SUSP_FINE: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Secondary split of a bucket: forward speed band, plus whether the handbrake
/// is mid-ramp. Real play taps the handbrake, so many of its ticks sit between
/// 0 and 1 where `POWERSLIDE_RISE_RATE`/`FALL_RATE` govern the value; scripted
/// recordings hold it down and sit at 1. If the ramped ticks carry the error,
/// the defect is the ramp rate, not the friction curve.
fn band_of(from: &CarRecord, to: &CarRecord) -> String {
    if BAND_BY_SUSP_FINE.with(std::cell::Cell::get) {
        // 1-UU compression bands, restricted to steps where the suspension is
        // not moving much, so the damping term contributes nothing and the
        // residual isolates the spring force-vs-compression curve.
        let mean = |c: &CarRecord| {
            let mut sum = 0.0;
            let mut n = 0;
            for w in &c.wheels {
                if w.has_contact {
                    sum += w.susp_length;
                    n += 1;
                }
            }
            if n == 4 { Some(sum / 4.0) } else { None }
        };
        let (Some(a), Some(b)) = (mean(from), mean(to)) else {
            return "/x".to_string();
        };
        if (b - a).abs() > 0.5 {
            return "/x".to_string();
        }
        // Compression in UU below the ray's zero, i.e. -susp_length.
        let c = -a;
        if !(0.0..12.0).contains(&c) {
            return "/x".to_string();
        }
        return format!("/comp{:02}", c as u32);
    }
    if BAND_BY_SUSP_RATE.with(std::cell::Cell::get) {
        // 2D: how compressed the suspension is, crossed with how fast it is
        // compressing. `susp_rel_vel` is a dead field in every recording, so the
        // rate is derived from the `susp_length` delta across the step (UU per
        // tick; negative = compressing). The spring term depends on compression
        // alone and the damping term on the rate, so a residual that tracks one
        // axis and not the other names which of the two is wrong.
        let mean = |c: &CarRecord| {
            let mut sum = 0.0;
            let mut n = 0;
            for w in &c.wheels {
                if w.has_contact {
                    sum += w.susp_length;
                    n += 1;
                }
            }
            if n == 0 { None } else { Some(sum / n as f32) }
        };
        let (Some(a), Some(b)) = (mean(from), mean(to)) else {
            return "/rate-nocontact".to_string();
        };
        let comp = match a {
            v if v < -4.0 => "/c:heavy",
            v if v < -2.5 => "/c:mod",
            _ => "/c:rest",
        };
        let rate = match b - a {
            r if r < -0.5 => "/r:compressing",
            r if r > 0.5 => "/r:extending",
            _ => "/r:steady",
        };
        return format!("{comp}{rate}");
    }
    if BAND_BY_SUSP.with(std::cell::Cell::get) {
        // Mean recorded `susp_length` over contacting wheels: the sim's
        // `suspension_length - suspension_rest_length_1` in UU. Equilibrium ride
        // height is about -1.98; more negative is more compressed. This
        // separates the suspension spring (acts at every compression) from
        // `extra_pushback`, which only fires once a wheel is compressed past
        // `ray_pushback_thresh`.
        let mut sum = 0.0;
        let mut n = 0;
        for w in &from.wheels {
            if w.has_contact {
                sum += w.susp_length;
                n += 1;
            }
        }
        if n == 0 {
            return "/susp-none".to_string();
        }
        let susp = sum / n as f32;
        let band = match susp {
            v if v < -8.0 => "/susp<-8",
            v if v < -4.0 => "/susp-8..-4",
            v if v < -2.5 => "/susp-4..-2.5",
            v if v < -1.5 => "/susp-2.5..-1.5",
            v if v < 0.0 => "/susp-1.5..0",
            _ => "/susp0+",
        };
        return band.to_string();
    }
    if BAND_BY_FCI.with(std::cell::Cell::get) {
        // Mean recorded `friction_curve_input` over contacting wheels: how
        // sideways the slide is. This is the input axis of `LAT_FRICTION` and
        // `HANDBRAKE_LAT_FRICTION_FACTOR`, so banding by it says directly
        // whether those curves are wrong and where.
        let mut sum = 0.0;
        let mut n = 0;
        for w in &from.wheels {
            if w.has_contact {
                sum += w.friction_curve_input;
                n += 1;
            }
        }
        let fci = if n > 0 { sum / n as f32 } else { 0.0 };
        let band = match fci {
            f if f <= 0.0 => "/fci0",
            f if f < 0.2 => "/fci0-.2",
            f if f < 0.4 => "/fci.2-.4",
            f if f < 0.6 => "/fci.4-.6",
            f if f < 0.8 => "/fci.6-.8",
            _ => "/fci.8-1",
        };
        return band.to_string();
    }
    let speed = Vec3A::from(from.phys.lin_vel).length();
    let sb = match speed {
        s if s < 500.0 => "/v0-500",
        s if s < 1000.0 => "/v500-1k",
        s if s < 1400.0 => "/v1k-1.4k",
        s if s < 1800.0 => "/v1.4k-1.8k",
        _ => "/v1.8k+",
    };
    let hb = match from.handbrake_val {
        0.0 => "",
        v if v >= 0.999 => "/hb-full",
        _ => "/hb-ramp",
    };
    format!("{sb}{hb}")
}

/// Name the cause of this step, most significant cause first.
fn bucket_of(
    from: &CarRecord,
    to: &CarRecord,
    ball_from: &PhysRecord,
    ball_to: &PhysRecord,
    others: &[(Vec3A, bool)],
    straddles_impulse: bool,
) -> &'static str {
    if straddles_impulse {
        return "impulse";
    }

    let ball_near = [(ball_from, from), (ball_to, to)]
        .iter()
        .any(|(ball, car)| {
            !is_sentinel(ball) && ball_touches_car(Vec3A::from(ball.pos), &car.phys)
        });
    if ball_near {
        return "ball_contact";
    }

    let here = Vec3A::from(to.phys.pos);
    if others
        .iter()
        .any(|&(pos, alive)| alive && (pos - here).length() < CAR_PROXIMITY)
    {
        return "car_proximity";
    }

    let (nw_from, nw_to) = (wheels_in_contact(from), wheels_in_contact(to));

    // There is no `body_scrape` bucket. There used to be, defined as
    // `nw_from == 0 && (from.has_world_contact || to.has_world_contact)` and
    // meant to catch a chassis dragging with no wheel down. `RLTOUCH=2` shows
    // the field cannot express that: `has_world_contact` is true on *zero* of
    // 183,257 car-ticks with no wheel in contact, so the condition could only
    // ever fire through its `to` term, and the flag itself lags contact by a
    // tick - true on 93.7% of steady wheel-contact ticks but only 2.9% of
    // touchdown ticks. The bucket was therefore a 3% sample of landings,
    // selected by the lag, wearing the name of a mechanism this dataset never
    // records. `world_contact_point` and `world_contact_normal` are dead too:
    // across 238,076 logged contacts they are the origin and exactly `+z` every
    // single time. See `touchdown.rs`.
    if nw_from > 0 && min_contact_z(from) < FLOOR_NORMAL_Z {
        return "wall_ceiling";
    }
    // Gaining and losing contact are different mechanisms and want separate
    // rows. On a landing the sim's wheel ray, cast from the start-of-tick pose,
    // finds nothing on 90% of the steps where RL has already produced an
    // impulse, so `touchdown` is where that phase error lands.
    if nw_from != nw_to {
        return match (nw_from, nw_to) {
            (0, _) => "touchdown",
            (_, 0) => "liftoff",
            _ => "wheel_transition",
        };
    }
    match nw_from {
        0 => {
            if from.is_boosting {
                "air_boost"
            } else {
                "air_free"
            }
        }
        4 => {
            if from.handbrake_val > 0.0 {
                "drive_handbrake"
            } else if from.is_boosting {
                "drive_boost"
            } else if from.prev_controls.throttle.abs() > 0.01 {
                "drive_throttle"
            } else {
                "drive_coast"
            }
        }
        _ => "drive_partial",
    }
}

pub fn analyze(recording: &Recording) {
    let num_cars = recording.info.num_cars as usize;
    if num_cars == 0 {
        return;
    }
    let stride = recording.stride;
    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];
    let mut buckets: std::collections::BTreeMap<String, Bucket> = Default::default();
    let mode = std::env::var("RLCENSUS").unwrap_or_default();
    let detail = mode == "2";
    // `RLCENSUS=3` splits each bucket by forward speed and handbrake ramp
    // state, to separate "the friction model is wrong" from "the friction model
    // is wrong in one corner of its input range".
    let sub_band = matches!(mode.as_str(), "3" | "4" | "5" | "6" | "7");
    BAND_BY_FCI.with(|b| b.set(mode == "4"));
    BAND_BY_SUSP.with(|b| b.set(mode == "5"));
    BAND_BY_SUSP_RATE.with(|b| b.set(mode == "6"));
    BAND_BY_SUSP_FINE.with(|b| b.set(mode == "7"));
    // `RLCTRLOFF=n` shifts which tick the step's controls are read from, to test
    // whether the harness has the recording's control alignment right. The
    // harness assumes tick T's `prev_controls` are the controls that drove the
    // step T-1 -> T, i.e. offset 0 (read from `to`). Offset -1 reads from
    // `from` instead; +1 reads from the tick after `to`.
    let ctrl_off: i64 = std::env::var("RLCTRLOFF")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(0);

    let last = recording.ticks.len().saturating_sub(stride + 1);
    for i in (0..=last).step_by(stride) {
        // Void the same non-physical discontinuities the gate voids, tick-wide
        // rather than per car: on a goal reset every car is teleported, and the
        // ones that happen to move less than the threshold are still mid-reset.
        if has_discontinuity(recording, i, stride) {
            continue;
        }

        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];

        let ctrl_idx = (i as i64 + stride as i64 + ctrl_off * stride as i64)
            .clamp(0, recording.ticks.len() as i64 - 1) as usize;
        for (j, car_record) in recording.ticks[ctrl_idx].car_records.iter().enumerate() {
            controls_buf[j] = car_record.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        // Other cars' positions at `to`, for the proximity test.
        let others: Vec<(Vec3A, bool)> = to_tick
            .car_records
            .iter()
            .map(|c| (Vec3A::from(c.phys.pos), !c.is_demoed))
            .collect();

        for (j, &car_idx) in car_idcs.iter().enumerate() {
            let from = &from_tick.car_records[j];
            let to = &to_tick.car_records[j];
            // Same exclusions as the gate: a demoed or absent car is parked by
            // the logger, so measuring it fabricates divergence.
            if to.is_demoed || is_car_sentinel(&to.phys) {
                continue;
            }

            let sim_vel = arena.get_car_state(car_idx).phys.vel;
            let err = (sim_vel - Vec3A::from(to.phys.lin_vel)).length();

            let mut rest = others.clone();
            rest[j] = (Vec3A::ZERO, false);
            let key = bucket_of(
                from,
                to,
                &from_tick.ball_record,
                &to_tick.ball_record,
                &rest,
                recording.step_straddles_impulse(i, stride, j),
            );
            let key = if sub_band {
                format!("{key}{}", band_of(from, to))
            } else {
                key.to_string()
            };
            // Residual in the car's own frame: forward / right / up.
            let rot = rot_of(&to.phys);
            let resid = sim_vel - Vec3A::from(to.phys.lin_vel);
            let local = Vec3A::new(
                resid.dot(rot.x_axis),
                resid.dot(rot.y_axis),
                resid.dot(rot.z_axis),
            );
            let game_vel = Vec3A::from(to.phys.lin_vel);
            let mag_local = Vec3A::new(
                sim_vel.dot(rot.x_axis).abs() - game_vel.dot(rot.x_axis).abs(),
                sim_vel.dot(rot.y_axis).abs() - game_vel.dot(rot.y_axis).abs(),
                sim_vel.dot(rot.z_axis).abs() - game_vel.dot(rot.z_axis).abs(),
            );
            buckets
                .entry(key)
                .or_default()
                .add(err, i, j, local, mag_local);
        }
    }

    for (key, b) in &mut buckets {
        let (p50, p90, p99) = (b.pct(0.50), b.pct(0.90), b.pct(0.99));
        println!(
            "[{}] CENSUS {key} n={} sum={:.3} sumsq={:.3} max={:.3}@t{} p50={p50:.3} p90={p90:.3} p99={p99:.3} bias=({:.4},{:.4},{:.4}) magbias=({:.4},{:.4},{:.4})",
            recording.name,
            b.n,
            b.sum,
            b.sum_sq,
            b.max,
            b.max_tick,
            b.local_bias.x / b.n as f64,
            b.local_bias.y / b.n as f64,
            b.local_bias.z / b.n as f64,
            b.local_mag_bias.x / b.n as f64,
            b.local_mag_bias.y / b.n as f64,
            b.local_mag_bias.z / b.n as f64,
        );
        if !detail {
            continue;
        }
        b.trim();
        for &(mag, tick, car) in &b.worst {
            println!(
                "[{}] CENSUS-WORST {key} {mag:.1} t{tick} car{car}",
                recording.name
            );
        }
    }
}
