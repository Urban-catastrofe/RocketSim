//! Benchmarks the per-tick-restore loop at long-replay scale.
//!
//! `run_per_tick` restores full car + ball state every tick, then steps. Ticks
//! are independent (state is restored from ground truth), so the pass shards
//! across threads, each owning its own arena. This reproduces that design to
//! confirm a 20-minute (~144k tick) replay stays usable, single- or
//! multi-threaded.
//!
//! cargo run -p rocketsim --release --example bench_long_replay -- --ticks 144000 --cars 6 --threads 16

use std::time::Instant;

use clap::Parser;
use rocketsim::{Arena, CarBodyConfig, GameMode, Team, init_from_default};

#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = 144_000)]
    ticks: usize,
    #[arg(long, default_value_t = 1)]
    cars: usize,
    #[arg(long, default_value_t = 1)]
    threads: usize,
}

/// One shard: own arena, restore a fresh driving state each tick, step.
fn run_shard(range: std::ops::Range<usize>, num_cars: usize) {
    let mut arena = Arena::new(GameMode::Soccar);
    let mut idcs = Vec::with_capacity(num_cars);
    for i in 0..num_cars {
        let team = if i % 2 == 0 { Team::Blue } else { Team::Orange };
        idcs.push(arena.add_car(team, CarBodyConfig::OCTANE));
    }
    let mut template = *arena.get_car_state(idcs[0]);
    template.phys.vel = glam::Vec3A::new(1500.0, 0.0, 0.0);
    template.controls.throttle = 1.0;
    template.controls.boost = true;
    template.is_on_ground = true;
    template.wheels_with_contact = [true; 4];
    let ball = *arena.get_ball_state();

    for t in range {
        for &idx in &idcs {
            let mut cs = template;
            cs.phys.pos.x = -2000.0 + (t % 400) as f32 * 10.0;
            arena.set_car_state(idx, cs);
        }
        arena.set_ball_state(ball);
        arena.step_tick();
    }
}

fn main() {
    let args = Args::parse();
    init_from_default(true).unwrap();

    let threads = args.threads.clamp(1, args.ticks.max(1));
    let start = Instant::now();
    if threads <= 1 {
        run_shard(0..args.ticks, args.cars);
    } else {
        let chunk = args.ticks.div_ceil(threads);
        std::thread::scope(|s| {
            let handles: Vec<_> = (0..threads)
                .map(|ti| {
                    let lo = ti * chunk;
                    let hi = (lo + chunk).min(args.ticks);
                    s.spawn(move || run_shard(lo..hi, args.cars))
                })
                .collect();
            for h in handles {
                h.join().unwrap();
            }
        });
    }
    let elapsed = start.elapsed().as_secs_f64();

    println!(
        "ticks={} cars={} threads={}: {:.2}s total, {:.0} ticks/s, {:.1}x realtime (120Hz)",
        args.ticks,
        args.cars,
        threads,
        elapsed,
        args.ticks as f64 / elapsed,
        (args.ticks as f64 / elapsed) / 120.0,
    );
}
