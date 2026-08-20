pub mod cpp_records;
mod data_reader;
mod normalize;
pub mod tick_record;

use std::io::ErrorKind;

use glam::Vec3A;
use rocketsim::consts::TICK_TIME;

use crate::rl_comparison_test::recording::tick_record::TickRecord;
use cpp_records::*;
use data_reader::DataReader;

const RLPR_MAGIC_BYTES: [u8; 4] = [82, 76, 80, 82];
const RLPR_VERSION: u32 = 3;
const RLPR_MIN_COMPAT_VERSION: u32 = 2;
const RLPR_MAX_CARS: usize = 8;

#[allow(dead_code)]
pub struct Recording {
    pub name: String,
    pub info: RecordingInfo,
    pub ticks: Vec<TickRecord>,
    pub stride: usize,
    /// Per `[tick][car]`: this tick begins a button-triggered impulse (jump,
    /// double jump or flip). Such a tick and the one before it straddle an
    /// unrecorded sub-frame press phase, so neither step is reproducible — see
    /// [`normalize::detect_impulse_onsets`]. Kept in step with `ticks` by
    /// `car_order::reorder_cars`, which permutes both.
    pub impulse_onsets: Vec<Vec<bool>>,
    /// Per `[tick][car]`: a car-car bump landed inside this tick at a sub-frame
    /// time. Recovered from Rocket League's own position/velocity consistency
    /// rather than from a flag — see [`normalize::detect_bump_onsets`]. Permuted
    /// alongside `impulse_onsets` by `car_order::reorder_cars`.
    pub bump_onsets: Vec<Vec<bool>>,
}

impl Recording {
    /// True if car `car` starts a button-triggered impulse on `tick`.
    pub fn is_impulse_onset(&self, tick: usize, car: usize) -> bool {
        Self::marked(&self.impulse_onsets, tick, car)
    }

    /// True if car `car` took or dealt a sub-frame bump on `tick`.
    pub fn is_bump_onset(&self, tick: usize, car: usize) -> bool {
        Self::marked(&self.bump_onsets, tick, car)
    }

    fn marked(marks: &[Vec<bool>], tick: usize, car: usize) -> bool {
        marks
            .get(tick)
            .and_then(|row| row.get(car))
            .copied()
            .unwrap_or(false)
    }

    /// True if car `car` begins any sub-frame impulse on `tick` — a jump, double
    /// jump or flip press, or a car-car bump.
    pub fn is_any_onset(&self, tick: usize, car: usize) -> bool {
        self.is_impulse_onset(tick, car) || self.is_bump_onset(tick, car)
    }

    /// True if the step `tick -> tick + stride` straddles a sub-frame impulse
    /// for car `car`. The impulse sits between the two recorded frames either
    /// side of an onset tick, so both the step into the onset and the step out
    /// of it carry an arbitrary share of it.
    pub fn step_straddles_impulse(&self, tick: usize, stride: usize, car: usize) -> bool {
        self.is_any_onset(tick, car) || self.is_any_onset(tick + stride, car)
    }
}

impl Recording {
    pub fn from_bytes(name: &str, bytes: &[u8]) -> Result<Recording, std::io::Error> {
        let mut reader = DataReader::new(bytes);

        for magic_byte in RLPR_MAGIC_BYTES {
            if reader.read_u8()? != magic_byte {
                return Err(std::io::Error::new(
                    ErrorKind::InvalidData,
                    "File is not a valid recording (wrong magic)",
                ));
            }
        }

        let are_we_big_endian = cfg!(target_endian = "big");
        let is_file_big_endian = reader.read_bool()?;
        if is_file_big_endian != are_we_big_endian {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                "File has wrong endianness",
            ));
        }

        let version = reader.read_u32()?;
        if !(RLPR_MIN_COMPAT_VERSION..=RLPR_VERSION).contains(&version) {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR version mismatch (expected: {RLPR_VERSION}, got: {version})"),
            ));
        }

        let info = unsafe { reader.read_struct_unsafe::<RecordingInfo>() }?;
        let num_cars = info.num_cars as usize;
        if num_cars > RLPR_MAX_CARS {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!("RLPR recording has too many cars (max: {RLPR_MAX_CARS}, got: {num_cars})"),
            ));
        }

        let num_ticks = reader.read_u32()?;
        let mut ticks = Vec::with_capacity(num_ticks as usize);

        for _ in 0..num_ticks {
            let mut car_records = Vec::with_capacity(num_cars);
            for _ in 0..num_cars {
                let car_record = if version >= 3 {
                    unsafe { reader.read_struct_unsafe::<CarRecord>()? }
                } else {
                    let v2: CarRecordV2 = unsafe { reader.read_struct_unsafe()? };
                    v2.into()
                };
                car_records.push(car_record);
            }
            let ball_record = unsafe { reader.read_struct_unsafe::<PhysRecord>() }?;
            ticks.push(TickRecord {
                car_records,
                ball_record,
            });
        }

        if reader.num_bytes_left() > 0 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "RLPR recording still has {} bytes left after reading all ticks",
                    reader.num_bytes_left()
                ),
            ));
        }

        // Mark impulse onsets from the *raw* pulses first: normalising clears
        // the airborne double-jump pulses, which are onsets all the same.
        let impulse_onsets = normalize::detect_impulse_onsets(&ticks, num_cars);

        // Bring the observer's state-machine flags onto the sim's semantics
        // before anything measures against them (see `normalize`).
        normalize::normalize_jump_active(&mut ticks, num_cars);

        let stride = Self::detect_stride(&ticks);
        if stride != 1 {
            return Err(std::io::Error::new(
                ErrorKind::InvalidData,
                format!(
                    "recording '{name}' detected at ~{}Hz (stride={stride}); \
                     only 120Hz recordings are reliable enough to sample for \
                     rocketsim tests. Re-record at 120Hz.",
                    120 * stride
                ),
            ));
        }
        // Both of these need the 120 Hz guarantee above: one array entry has to
        // be one physics tick for the ramp integration and for the position
        // identity to mean anything. They read `prev_controls`, positions and
        // velocities, none of which `normalize_jump_active` touches.
        normalize::normalize_handbrake_val(&mut ticks, num_cars);
        let bump_onsets = normalize::detect_bump_onsets(&ticks, num_cars, stride);

        Ok(Self {
            name: name.to_string(),
            info,
            ticks,
            stride,
            impulse_onsets,
            bump_onsets,
        })
    }

    /// Detect the recording tick rate relative to the sim's 120 Hz.
    ///
    /// Compares actual position deltas between consecutive ticks against
    /// the expected delta (velocity × TICK_TIME).  Returns the stride
    /// needed to down-sample to 120 Hz equivalent.
    ///
    /// - stride=1 → recording is at 120 Hz (normal)
    /// - stride=2 → recording is at 240 Hz (skip every other tick)
    fn detect_stride(ticks: &[TickRecord]) -> usize {
        let mut ratios: Vec<f32> = Vec::new();
        for i in 0..ticks.len().saturating_sub(1) {
            if ticks[i].car_records.is_empty() {
                continue;
            }
            let vel: Vec3A = ticks[i].car_records[0].phys.lin_vel.into();
            let speed = vel.length();
            if speed < 100.0 {
                continue; // skip stationary / very slow ticks
            }
            let pos_curr: Vec3A = ticks[i].car_records[0].phys.pos.into();
            let pos_next: Vec3A = ticks[i + 1].car_records[0].phys.pos.into();
            let actual_delta = (pos_next - pos_curr).length();
            if actual_delta < 0.01 {
                continue; // skip duplicate frames
            }
            let expected_delta = speed * TICK_TIME;
            ratios.push(expected_delta / actual_delta);
            if ratios.len() >= 60 {
                break;
            }
        }

        if ratios.len() < 5 {
            return 1; // not enough data, assume 120 Hz
        }

        // Use median for robustness against outliers
        ratios.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = ratios[ratios.len() / 2];
        let stride = median.round() as usize;

        stride.clamp(1, 8)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The C++ writer (RlprWriter.h) asserts these exact sizes; a mismatch here
    /// means the binary layout drifted between the two ends and every recording
    /// would parse garbage. Verified by `static_assert` in RlprWriter.h.
    #[test]
    fn layout_matches_cpp_writer() {
        assert_eq!(size_of::<VecRecord>(), 12);
        assert_eq!(size_of::<Mat3Record>(), 36);
        assert_eq!(size_of::<PhysRecord>(), 104);
        assert_eq!(size_of::<ControlsRecord>(), 24);
        assert_eq!(size_of::<WheelRecord>(), 56);
        assert_eq!(size_of::<HitRecord>(), 60);
        assert_eq!(size_of::<CarRecord>(), 468);
        assert_eq!(size_of::<CarRecordV2>(), 408);
        assert_eq!(size_of::<RecordingInfo>(), 28);
    }

    /// Round-trip: hand-build a v3 RLPR (one tick, one car with a hit record)
    /// and confirm the parser recovers the hit fields.
    #[test]
    fn parses_v3_hit_record() {
        let mut bytes: Vec<u8> = Vec::new();
        let mut push = |b: &[u8]| bytes.extend_from_slice(b);

        push(&RLPR_MAGIC_BYTES);
        push(&[0]); // little-endian
        push(&RLPR_VERSION.to_le_bytes());
        push(&(size_of::<RecordingInfo>() as u32).to_le_bytes());
        let info = RecordingInfo {
            num_cars: 1,
            hitbox_rel_min_bt: VecRecord::new(0., 0., 0.),
            hitbox_rel_max_bt: VecRecord::new(0., 0., 0.),
        };
        let info_bytes: [u8; 28] = unsafe { std::mem::transmute(info) };
        push(&info_bytes);
        push(&1u32.to_le_bytes()); // num_ticks

        // Car record (v3, with hit)
        let mut car = CarRecord {
            phys: PhysRecord {
                physics_frame: 1,
                pos: VecRecord::new(100., 0., 20.),
                rot: Mat3Record {
                    rows: [
                        VecRecord::new(1., 0., 0.),
                        VecRecord::new(0., 1., 0.),
                        VecRecord::new(0., 0., 1.),
                    ],
                },
                lin_vel: VecRecord::new(500., 0., 0.),
                ang_vel: VecRecord::new(0., 0., 0.),
                has_world_contact: true,
                world_contact_point: VecRecord::new(0., 0., 0.),
                world_contact_normal: VecRecord::new(0., 0., 1.),
            },
            is_on_ground: true,
            is_jumping: false,
            is_flipping: false,
            jump_time: 0.,
            flip_time: 0.,
            has_jumped: false,
            double_jumped_or_flipped: false,
            has_flip: false,
            flip_rel_torque: VecRecord::new(0., 0., 0.),
            boost_amount: 100.,
            is_touching_ball: true,
            prev_controls: ControlsRecord {
                throttle: 1.,
                steer: 0.,
                pitch: 0.,
                yaw: 0.,
                roll: 0.,
                jump: false,
                boost: false,
                handbrake: false,
            },
            wheels: [WheelRecord {
                susp_length: 0.,
                susp_rel_vel: 0.,
                has_contact: true,
                contact_normal: VecRecord::new(0., 0., 1.),
                steer_amount: 0.,
                engine_force: 0.,
                brake: 0.,
                lat_friction: 0.,
                long_friction: 0.,
                extra_pushback: 0.,
                spin_speed: 0.,
                friction_curve_input: 0.,
            }; 4],
            is_boosting: false,
            is_supersonic: false,
            is_demoed: false,
            handbrake_val: 0.,
            demo_respawn_timer: 0.,
            air_time: 0.,
            air_time_since_jump: 0.,
            hit: HitRecord {
                has_hit: true,
                _pad: [0; 3],
                ball_vel_before: VecRecord::new(0., 0., 0.),
                car_vel_before: VecRecord::new(500., 0., 0.),
                hit_normal: VecRecord::new(1., 0., 0.),
                hit_location: VecRecord::new(100., 0., 20.),
                rel_vel_mag: 500.,
                closing_speed: 500.,
            },
        };
        // Fix the hit's _pad to be zeroed (transmute of struct with bool).
        car.hit._pad = [0; 3];
        let car_bytes: [u8; 468] = {
            let mut b = [0u8; 468];
            let raw: [u8; 468] = unsafe { std::mem::transmute(car) };
            b.copy_from_slice(&raw);
            b
        };
        push(&(size_of::<CarRecord>() as u32).to_le_bytes());
        push(&car_bytes);

        // Ball record
        let ball = PhysRecord {
            physics_frame: 1,
            pos: VecRecord::new(100., 0., 20.),
            rot: Mat3Record {
                rows: [
                    VecRecord::new(1., 0., 0.),
                    VecRecord::new(0., 1., 0.),
                    VecRecord::new(0., 0., 1.),
                ],
            },
            lin_vel: VecRecord::new(300., 0., 0.),
            ang_vel: VecRecord::new(0., 0., 0.),
            has_world_contact: false,
            world_contact_point: VecRecord::new(0., 0., 0.),
            world_contact_normal: VecRecord::new(0., 0., 0.),
        };
        let ball_bytes: [u8; 104] = unsafe { std::mem::transmute(ball) };
        push(&(size_of::<PhysRecord>() as u32).to_le_bytes());
        push(&ball_bytes);

        let rec = Recording::from_bytes("v3_test", &bytes).expect("v3 parse failed");
        assert_eq!(rec.ticks.len(), 1);
        let hit = &rec.ticks[0].car_records[0].hit;
        assert!(hit.has_hit);
        assert_eq!(hit.rel_vel_mag, 500.);
        assert_eq!(hit.closing_speed, 500.);
        assert_eq!(hit.hit_normal.x, 1.);
    }
}
