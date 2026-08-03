//! Diagnostic: drive a single car on flat ground and log its vertical dynamics.
//!
//! The sim's grounded cars show a systematic +4..+7.5 uu/s per-tick z-velocity
//! bias vs the game (the "car bounces" problem). This example isolates the
//! car's vertical response: place an Octane on flat ground, hold throttle, and
//! print z-velocity, z-position, and per-wheel suspension each tick.
//!
//! cargo run -p rocketsim --release --example car_z_dynamics

use rocketsim::{Arena, CarBodyConfig, CarControls, GameMode, Team, init_from_default};

fn main() {
    init_from_default(true).unwrap();

    let mut arena = Arena::new(GameMode::Soccar);
    let idx = arena.add_car(Team::Blue, CarBodyConfig::OCTANE);

    let mut cs = *arena.get_car_state(idx);
    cs.phys.pos = glam::Vec3A::new(0.0, 0.0, 17.0);
    cs.phys.vel = glam::Vec3A::ZERO;
    cs.is_on_ground = true;
    cs.wheels_with_contact = [true; 4];
    arena.set_car_state(idx, cs);

    let mut controls = CarControls::default();
    controls.throttle = 1.0;
    arena.set_car_controls(idx, controls);

    // Drive for 5 seconds, report max speed + final speed.
    let mut max_speed = 0.0f32;
    for t in 0..600 {
        arena.step_tick();
        let s = *arena.get_car_state(idx);
        max_speed = max_speed.max(s.phys.vel.length());
    }
    let s = *arena.get_car_state(idx);
    println!("=== throttle 1.0, 5s ===");
    println!("max_speed={max_speed:.1}  final_speed={:.1}  final_pos_z={:.3}  boost={:.1}", s.phys.vel.length(), s.phys.pos.z, s.boost);

    println!("=== throttle 1.0 + boost, 5s ===");
    controls.boost = true;
    arena.set_car_controls(idx, controls);
    let mut max_speed = 0.0f32;
    for _ in 0..600 {
        arena.step_tick();
        let s = *arena.get_car_state(idx);
        max_speed = max_speed.max(s.phys.vel.length());
    }
    println!("max_speed={max_speed:.1}  boost={:.1}", arena.get_car_state(idx).boost);
}