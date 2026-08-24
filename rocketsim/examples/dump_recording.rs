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
    let path = std::env::args()
        .nth(1)
        .expect("usage: dump_recording <file.rlpr> [start_tick] [end_tick]");
    let start: usize = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let end: usize = std::env::args()
        .nth(3)
        .and_then(|s| s.parse().ok())
        .unwrap_or(usize::MAX);

    let bytes = fs::read(&path).expect("read file");
    let mut c = Cursor::new(bytes.as_slice());

    let mut magic = [0u8; 4];
    c.read_exact(&mut magic).unwrap();
    assert_eq!(&magic, b"RLPR", "bad magic");
    let _endian = read_u8(&mut c);
    let version = read_u32(&mut c);
    assert!(version == 2 || version == 3, "version");
    let info: RecordingInfo = unsafe { read_struct(&mut c) };
    let num_cars = info.num_cars as usize;
    let num_ticks = read_u32(&mut c) as usize;

    println!("cars={} ticks={}", num_cars, num_ticks);
    let end = end.min(num_ticks);
    for i in 0..num_ticks {
        let mut cars = Vec::with_capacity(num_cars);
        for _ in 0..num_cars {
            cars.push(unsafe { read_struct::<CarRecord>(&mut c) });
        }
        let _ball: PhysRecord = unsafe { read_struct(&mut c) };
        if i >= start && i < end {
            if i % 2 == 0 {
                let bp = _ball.pos;
                let bv = _ball.lin_vel;
                println!(
                    "t={i:5} BALL pos=({:7.1},{:7.1},{:5.1}) vel=({:7.1},{:7.1},{:5.1})",
                    bp.x, bp.y, bp.z, bv.x, bv.y, bv.z
                );
            }
            for (j, cr) in cars.iter().enumerate() {
                let p = cr.phys.pos;
                let v = cr.phys.lin_vel;
                println!(
                    "t={i:5} c{j}: pf={:6} pos=({:7.1},{:7.1},{:5.1}) vel=({:7.1},{:7.1},{:5.1}) av=({:5.2},{:5.2},{:5.2}) upz={:.3} g={} j={} jt={:.4} hj={} df={} flip={} ft={:.4} frt=({:.2},{:.2},{:.2}) atj={:.4} at={:.4} ctrl_jump={} boost={}",
                    cr.phys.physics_frame,
                    p.x,
                    p.y,
                    p.z,
                    v.x,
                    v.y,
                    v.z,
                    cr.phys.ang_vel.x,
                    cr.phys.ang_vel.y,
                    cr.phys.ang_vel.z,
                    cr.phys.rot.rows[2].z,
                    cr.is_on_ground as u8,
                    cr.is_jumping as u8,
                    cr.jump_time,
                    cr.has_jumped as u8,
                    cr.double_jumped_or_flipped as u8,
                    cr.is_flipping as u8,
                    cr.flip_time,
                    cr.flip_rel_torque.x,
                    cr.flip_rel_torque.y,
                    cr.flip_rel_torque.z,
                    cr.air_time_since_jump,
                    cr.air_time,
                    cr.prev_controls.jump as u8,
                    cr.boost_amount,
                );
            }
        }
    }
}
