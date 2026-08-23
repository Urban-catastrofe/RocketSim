//! Scan wall recordings for wall-dash sequences: car grounded on a wall
//! (is_on_ground + low contact-normal z) and jumping rapidly (two jump presses
//! within a short window). Prints the sequence window per recording so the
//! harness can be deep-dived on exactly those ticks.

use std::fs;
use std::io::{Cursor, Read};

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct VecRecord {
    x: f32,
    y: f32,
    z: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct Mat3Record {
    rows: [VecRecord; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct PhysRecord {
    physics_frame: u32,
    pos: VecRecord,
    rot: Mat3Record,
    lin_vel: VecRecord,
    ang_vel: VecRecord,
    has_world_contact: bool,
    world_contact_point: VecRecord,
    world_contact_normal: VecRecord,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct ControlsRecord {
    throttle: f32,
    steer: f32,
    pitch: f32,
    yaw: f32,
    roll: f32,
    jump: bool,
    boost: bool,
    handbrake: bool,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct WheelRecord {
    susp_length: f32,
    susp_rel_vel: f32,
    has_contact: bool,
    contact_normal: VecRecord,
    steer_amount: f32,
    engine_force: f32,
    brake: f32,
    lat_friction: f32,
    long_friction: f32,
    extra_pushback: f32,
    spin_speed: f32,
    friction_curve_input: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct CarRecord {
    phys: PhysRecord,
    is_on_ground: bool,
    is_jumping: bool,
    is_flipping: bool,
    jump_time: f32,
    flip_time: f32,
    has_jumped: bool,
    double_jumped_or_flipped: bool,
    has_flip: bool,
    flip_rel_torque: VecRecord,
    boost_amount: f32,
    is_touching_ball: bool,
    prev_controls: ControlsRecord,
    wheels: [WheelRecord; 4],
    is_boosting: bool,
    is_supersonic: bool,
    is_demoed: bool,
    handbrake_val: f32,
    demo_respawn_timer: f32,
    air_time: f32,
    air_time_since_jump: f32,
    hit: HitRecord,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct HitRecord {
    has_hit: bool,
    _pad: [u8; 3],
    ball_vel_before: VecRecord,
    car_vel_before: VecRecord,
    hit_normal: VecRecord,
    hit_location: VecRecord,
    rel_vel_mag: f32,
    closing_speed: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
struct RecordingInfo {
    num_cars: u32,
    hitbox_rel_min_bt: VecRecord,
    hitbox_rel_max_bt: VecRecord,
}

fn read_u32(c: &mut Cursor<&[u8]>) -> u32 {
    let mut b = [0u8; 4];
    c.read_exact(&mut b).unwrap();
    u32::from_le_bytes(b)
}

fn read_u8(c: &mut Cursor<&[u8]>) -> u8 {
    let mut b = [0u8; 1];
    c.read_exact(&mut b).unwrap();
    b[0]
}

unsafe fn read_struct<T: Copy>(c: &mut Cursor<&[u8]>) -> T {
    let size_prefix = read_u32(c) as usize;
    assert_eq!(
        size_prefix,
        std::mem::size_of::<T>(),
        "struct size mismatch"
    );
    let mut obj = std::mem::MaybeUninit::<T>::zeroed();
    let slice = std::slice::from_raw_parts_mut(obj.as_mut_ptr() as *mut u8, size_prefix);
    c.read_exact(slice).unwrap();
    unsafe { obj.assume_init() }
}

fn main() {
    let dir = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "rocketsim/tests/rl_comparison_test/test_recordings".into());
    let mut names: Vec<String> = fs::read_dir(&dir)
        .unwrap()
        .filter_map(|e| e.ok())
        .map(|e| e.path().to_string_lossy().into_owned())
        .filter(|p| p.ends_with(".rlpr"))
        .collect();
    names.sort();
    // Only wall-related recordings
    names.retain(|p| {
        let base = fs::canonicalize(p).unwrap();
        let b = base.file_stem().unwrap().to_string_lossy().into_owned();
        b.contains("wall") || b.contains("ground_to_wall") || b.contains("mech_wall")
    });
    // Only recordings with at least one car (ball-only recordings have cars=0).
    names.retain(|p| {
        let bytes = fs::read(p).unwrap();
        let mut c = Cursor::new(bytes.as_slice());
        let mut magic = [0u8; 4];
        c.read_exact(&mut magic).unwrap();
        if &magic != b"RLPR" {
            return false;
        }
        let _endian = read_u8(&mut c);
        let _version = read_u32(&mut c);
        let info: RecordingInfo = unsafe { read_struct(&mut c) };
        info.num_cars > 0
    });

    let mut found_any = false;
    for path in names {
        let bytes = fs::read(&path).unwrap();
        let mut c = Cursor::new(bytes.as_slice());
        let mut magic = [0u8; 4];
        c.read_exact(&mut magic).unwrap();
        if &magic != b"RLPR" {
            continue;
        }
        let _endian = read_u8(&mut c);
        let _version = read_u32(&mut c);
        let info: RecordingInfo = unsafe { read_struct(&mut c) };
        let num_cars = info.num_cars as usize;
        let num_ticks = read_u32(&mut c) as usize;

        let mut ticks: Vec<Vec<CarRecord>> = Vec::with_capacity(num_ticks);
        for _ in 0..num_ticks {
            let mut cars = Vec::with_capacity(num_cars);
            for _ in 0..num_cars {
                cars.push(unsafe { read_struct::<CarRecord>(&mut c) });
            }
            let _ball: PhysRecord = unsafe { read_struct(&mut c) };
            ticks.push(cars);
        }

        // Detect wall-dash sequences: the car was grounded on a wall
        // (has_world_contact with |contact normal z| < 0.5) shortly before a
        // jump press, and then jumps/flips again rapidly (within the
        // double-jump window, ~40 ticks at 120Hz).
        let name = std::path::Path::new(&path)
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .into_owned();
        let mut seqs: Vec<(usize, usize, f32)> = Vec::new();
        if ticks.is_empty() || ticks[0].is_empty() {
            eprintln!("[{name}] EMPTY (cars={num_cars} ticks={num_ticks})");
            continue;
        }
        for t in 0..ticks.len() {
            let car = ticks[t][0];
            // jump pressed this tick
            if !car.prev_controls.jump {
                continue;
            }
            // was the car wall-grounded within the last 20 ticks before this jump?
            let wall_before = (t.saturating_sub(20)..t).any(|k| {
                let c = ticks[k][0];
                c.is_on_ground
                    && c.phys.has_world_contact
                    && c.phys.world_contact_normal.z.abs() < 0.5
            });
            // find next jump press within the double-jump window
            let mut j2 = None;
            for k in (t + 1)..(t + 60).min(ticks.len()) {
                if ticks[k][0].prev_controls.jump {
                    j2 = Some(k);
                    break;
                }
            }
            if let Some(t2) = j2 {
                let gap = t2 - t;
                let upz = car.phys.rot.rows[2].z;
                let pos = car.phys.pos;
                let on_wall_now =
                    car.phys.has_world_contact && car.phys.world_contact_normal.z.abs() < 0.5;
                let tag = if wall_before { "WALL-DASH" } else { "jump" };
                if gap <= 40 {
                    found_any = true;
                    eprintln!(
                        "[{name}] {tag} t={t}->{t2} gap={gap}t pos=({:.0},{:.0},{:.0}) upz={upz:.2} on_wall_now={} av=({:.1},{:.1},{:.1})",
                        pos.x,
                        pos.y,
                        pos.z,
                        on_wall_now as u8,
                        car.phys.ang_vel.x,
                        car.phys.ang_vel.y,
                        car.phys.ang_vel.z,
                    );
                }
                seqs.push((t, t2, gap as f32));
            }
        }
        if !seqs.is_empty() {
            let min_t = seqs.iter().map(|s| s.0).min().unwrap();
            let max_t = seqs.iter().map(|s| s.1).max().unwrap();
            let _ = min_t;
            let _ = max_t;
        }
    }
    if !found_any {
        eprintln!("no tight wall-dash sequences found (gap<=40)");
    }
}
