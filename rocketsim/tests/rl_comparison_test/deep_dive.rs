//! Failure-mode deep dive.
//!
//! When the gate fails (or `RLDEEP=always` / `RLTICK` is set), re-drive a
//! fresh arena over a window of ticks around the worst divergence and print
//! everything needed to reason about *why*: per-tick physics deltas, the
//! situation flags on both sides, the controls applied, and a rich field-by-
//! field pred-vs-real snapshot at the center tick.
//!
//! This is the "hand the LLM a UUID and get the context" entry point: run
//! `cargo test case_<name>` with `RLDEEP=always` (optionally `RLCAR`,
//! `RLTICK`) and read the `[<name>] DIVE` lines.

use rocketsim::consts::TICK_TIME;
use rocketsim::{CarControls, CarState};

use super::compare::{self, Comparison};
use super::config::HarnessConfig;
use super::measure::compute_delta;
use super::recording::Recording;
use super::runner::{make_arena, set_state_to_record_tick};
use super::stats::Field;

fn fmt_controls(c: &CarControls) -> String {
    // CarControls is repr(packed); copy fields to locals before referencing.
    let throttle = c.throttle;
    let steer = c.steer;
    let pitch = c.pitch;
    let yaw = c.yaw;
    let roll = c.roll;
    let jump = c.jump;
    let boost = c.boost;
    let handbrake = c.handbrake;
    format!(
        "th={:+.2} st={:+.2} pitch={:+.2} yaw={:+.2} roll={:+.2} jump={} boost={} hb={}",
        throttle, steer, pitch, yaw, roll, jump as u8, boost as u8, handbrake as u8,
    )
}

fn bool_pair(p: bool, r: bool) -> String {
    if p == r {
        format!("{}", p as u8)
    } else {
        format!("{}!{}", p as u8, r as u8) // pred ! real, mismatched
    }
}

/// Print the rich field-by-field snapshot at the center tick.
fn print_center_snapshot(
    name: &str,
    car_states: &[CarState],
    ball_state: &rocketsim::BallState,
    to_tick: &super::recording::tick_record::TickRecord,
    focus_entity: usize,
    num_cars: usize,
) {
    let comparison = compare::compare_states_to_tick(car_states, ball_state, to_tick);
    let mut fields: Vec<(String, f32, String)> = comparison
        .map()
        .iter()
        .map(|(k, v)| {
            let detail = match v {
                Comparison::Float(d) => format!("{} vs {}", d.pred, d.real),
                Comparison::Bool(d) => format!("{} vs {}", d.pred, d.real),
                Comparison::Vec(d) => format!("{:.3} vs {:.3}", d.pred, d.real),
            };
            (k.clone(), v.rel_error(), detail)
        })
        .collect();
    fields.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    let focus_label = if focus_entity < num_cars {
        format!("car_{focus_entity}")
    } else {
        "ball".to_string()
    };
    println!("[{name}] DIVE center snapshot (focus={focus_label}), worst-first:");
    for (k, rel, detail) in fields {
        println!("[{name}] DIVE   {k:<32} rel={rel:8.3}  pred vs real: {detail}");
    }
}

/// Re-drive `recording` over a window around `center` and print context.
pub fn dump_window(recording: &Recording, cfg: &HarnessConfig, center: usize, entity: usize) {
    let stride = recording.stride;
    let num_cars = recording.info.num_cars as usize;
    let raw_max = recording.ticks.len().saturating_sub(stride + 1);
    let n = (raw_max / stride) * stride;

    let radius = cfg.deep_dive_radius * stride;
    let start = (center.saturating_sub(radius) / stride) * stride;
    let end = ((center + radius) / stride).min(n) * stride;

    println!(
        "[{}] DIVE window ticks {}..={} (center={}, stride={}, radius={} ticks/side)",
        recording.name,
        start,
        end,
        center,
        stride,
        cfg.deep_dive_radius
    );

    let (mut arena, car_idcs) = make_arena(num_cars);
    let mut controls_buf: Vec<CarControls> = vec![CarControls::DEFAULT; num_cars];

    for i in (start..=end).step_by(stride) {
        let from_tick = &recording.ticks[i];
        let to_tick = &recording.ticks[i + stride];
        for (j, cr) in to_tick.car_records.iter().enumerate() {
            controls_buf[j] = cr.prev_controls.into();
        }
        set_state_to_record_tick(&mut arena, &car_idcs, from_tick, &controls_buf);
        arena.step_tick();

        let time_s = i as f32 * TICK_TIME / stride as f32;
        let marker = if i == center { ">>" } else { "  " };

        // Ball-only recordings (no cars): focus the ball.
        if num_cars == 0 {
            let ball_state = arena.get_ball_state();
            let delta = compute_delta(&ball_state.phys, &to_tick.ball_record);
            let pos_e = delta.get(Field::Pos).mag;
            let vel_e = delta.get(Field::Vel).mag;
            let ang_e = delta.get(Field::AngVel).mag;
            println!(
                "[{}] DIVE {} t={:>6} ({:7.3}s) ball pos_e={:9.4} vel_e={:9.4} ang_e={:8.4}",
                recording.name,
                marker,
                i,
                time_s,
                pos_e,
                vel_e,
                ang_e,
            );
            println!(
                "[{}] DIVE        SIM  pos=({:.3},{:.3},{:.3}) vel=({:.3},{:.3},{:.3}) | REAL pos=({:.3},{:.3},{:.3}) vel=({:.3},{:.3},{:.3})",
                recording.name,
                ball_state.phys.pos.x,
                ball_state.phys.pos.y,
                ball_state.phys.pos.z,
                ball_state.phys.vel.x,
                ball_state.phys.vel.y,
                ball_state.phys.vel.z,
                to_tick.ball_record.pos.x,
                to_tick.ball_record.pos.y,
                to_tick.ball_record.pos.z,
                to_tick.ball_record.lin_vel.x,
                to_tick.ball_record.lin_vel.y,
                to_tick.ball_record.lin_vel.z,
            );
            if i == center {
                print_center_snapshot(
                    &recording.name,
                    &[],
                    ball_state,
                    to_tick,
                    entity,
                    num_cars,
                );
            }
            continue;
        }

        // Focused entity's physics deltas.
        let focus_car = entity.min(num_cars.saturating_sub(1));
        let cs: CarState = *arena.get_car_state(car_idcs[focus_car]);
        let real = &to_tick.car_records[focus_car];
        let delta = compute_delta(&cs.phys, &real.phys);
        let pos_e = delta.get(Field::Pos).mag;
        let vel_e = delta.get(Field::Vel).mag;
        let ang_e = delta.get(Field::AngVel).mag;

        let from_car = &from_tick.car_records[focus_car];
        let pred_pitch = cs.phys.rot_mat.x_axis.z.asin().to_degrees();
        let real_rot = glam::Mat3A::from_cols(
            real.phys.rot.rows[0].into(),
            real.phys.rot.rows[1].into(),
            real.phys.rot.rows[2].into(),
        );
        let real_pitch = real_rot.x_axis.z.asin().to_degrees();
        println!(
            "[{}] DIVE {} t={:>6} ({:7.3}s) car{} pos_e={:8.3} vel_e={:8.3} ang_e={:7.3} | pitch {:+5.1}deg/{:+5.1}deg vz {:+6.1}/{:+6.1} | ground={} jump={} flip={} boost={} | {}",
            recording.name,
            marker,
            i,
            time_s,
            focus_car,
            pos_e,
            vel_e,
            ang_e,
            pred_pitch,
            real_pitch,
            cs.phys.vel.z,
            real.phys.lin_vel.z,
            bool_pair(cs.is_on_ground, real.is_on_ground),
            bool_pair(cs.is_jumping, real.is_jumping),
            bool_pair(cs.is_flipping, real.is_flipping),
            bool_pair(cs.is_boosting, real.is_boosting),
            fmt_controls(&controls_buf[focus_car]),
        );
        let wcn = cs
            .world_contact_normal
            .map(|n| format!("wcn=({:.2},{:.2},{:.2})", n.x, n.y, n.z))
            .unwrap_or_else(|| "wcn=none".into());
        let nw = arena
            .get_car_state(car_idcs[focus_car])
            .num_wheels_in_contact();
        println!(
            "[{}] DIVE        {} nwheels={} from_pos=({:.1},{:.1},{:.1})",
            recording.name,
            wcn,
            nw,
            from_car.phys.pos.x,
            from_car.phys.pos.y,
            from_car.phys.pos.z
        );
        println!(
            "[{}] DIVE        SIM  pos=({:.2},{:.2},{:.2}) vel=({:.1},{:.1},{:.1}) | REAL pos=({:.2},{:.2},{:.2}) vel=({:.1},{:.1},{:.1})",
            recording.name,
            cs.phys.pos.x,
            cs.phys.pos.y,
            cs.phys.pos.z,
            cs.phys.vel.x,
            cs.phys.vel.y,
            cs.phys.vel.z,
            real.phys.pos.x,
            real.phys.pos.y,
            real.phys.pos.z,
            real.phys.lin_vel.x,
            real.phys.lin_vel.y,
            real.phys.lin_vel.z,
        );

        // Situation context line (from-state regime) for the focused car.
        println!(
            "[{}] DIVE        from-state: ground={} jump={} flip={} boost={} super={} air_time={:.3}",
            recording.name,
            from_car.is_on_ground as u8,
            from_car.is_jumping as u8,
            from_car.is_flipping as u8,
            from_car.is_boosting as u8,
            from_car.is_supersonic as u8,
            from_car.air_time,
        );

        if i == center {
            let car_states: Vec<CarState> = car_idcs
                .iter()
                .map(|&ci| *arena.get_car_state(ci))
                .collect();
            let ball_state = arena.get_ball_state();
            print_center_snapshot(
                &recording.name,
                &car_states,
                ball_state,
                to_tick,
                entity,
                num_cars,
            );
        }
    }
}
