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

    println!("=== throttle 1.0 ===");
    println!("tick | z_pos | z_vel | z_vel_delta | on_ground");
    let mut prev_z = 0.0f32;
    for t in 0..240 {
        arena.step_tick();
        let s = *arena.get_car_state(idx);
        let dz = s.phys.vel.z - prev_z;
        if t % 6 == 0 {
            println!(
                "{t:>4} | {:>7.3} | {:>7.3} | {:>7.3} | {}",
                s.phys.pos.z, s.phys.vel.z, dz, s.is_on_ground
            );
        }
        prev_z = s.phys.vel.z;
    }

    // Now coast (no throttle) from a rolling state.
    controls.throttle = 0.0;
    arena.set_car_controls(idx, controls);
    let mut cs = *arena.get_car_state(idx);
    cs.phys.vel = glam::Vec3A::new(1000.0, 0.0, 0.0);
    arena.set_car_state(idx, cs);
    println!("=== coast @1000 uu/s ===");
    let mut prev_z = 0.0f32;
    for t in 240..480 {
        arena.step_tick();
        let s = *arena.get_car_state(idx);
        let dz = s.phys.vel.z - prev_z;
        if t % 6 == 0 {
            println!(
                "{t:>4} | {:>7.3} | {:>7.3} | {:>7.3} | {}",
                s.phys.pos.z, s.phys.vel.z, dz, s.is_on_ground
            );
        }
        prev_z = s.phys.vel.z;
    }
}