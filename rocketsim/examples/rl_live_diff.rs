//! Compare a scripted Rocket League TCP capture against this RocketSim build.

use std::{cmp::Ordering, collections::BTreeMap, error::Error, fs, path::PathBuf};

use clap::Parser;
use glam::EulerRot;
use rocketsim::{
    Arena, ArenaConfig, BallState, CarBodyConfig, CarControls, CarState, GameMode, Mat3A, Team,
    Vec3A,
};
use serde::{Deserialize, Serialize};

#[cfg(feature = "cpp-compare")]
use rocketsim_rs::{
    math::{RotMat as CppRotMat, Vec3 as CppVec3},
    sim::{
        Arena as CppArena, CarConfig as CppCarConfig, CarControls as CppCarControls,
        Team as CppTeam,
    },
};

#[derive(Parser)]
struct Args {
    /// JSON object containing `{ "request": ..., "response": ... }`.
    #[arg(long)]
    input: PathBuf,

    /// Number of individual worst samples retained in the JSON output.
    #[arg(long, default_value_t = 25)]
    worst: usize,

    /// Position error above which a trigger window is emitted.
    #[arg(long, default_value_t = 1.0)]
    position_threshold: f32,

    /// Number of ticks included before a position-error trigger.
    #[arg(long, default_value_t = 120)]
    pre_ticks: usize,

    /// Number of ticks included after a position-error trigger.
    #[arg(long, default_value_t = 120)]
    post_ticks: usize,

    /// Also replay through stock C++ RocketSim (requires `--features cpp-compare`).
    #[arg(long)]
    cpp: bool,
}

#[derive(Debug, Deserialize)]
struct Capture {
    request: Scenario,
    response: LiveResponse,
}

#[derive(Debug, Deserialize)]
struct Scenario {
    #[serde(default)]
    name: String,
    ticks: usize,
    #[serde(default)]
    ball: Option<InitialBody>,
    #[serde(default)]
    cars: Vec<InitialCar>,
}

#[derive(Debug, Deserialize)]
struct InitialBody {
    p: [f32; 3],
    v: [f32; 3],
    #[serde(default)]
    av: [f32; 3],
}

#[derive(Debug, Deserialize)]
struct InitialCar {
    p: [f32; 3],
    v: [f32; 3],
    #[serde(default)]
    av: [f32; 3],
    /// Yaw, pitch, roll in radians.
    #[serde(default)]
    rot: [f32; 3],
    #[serde(default)]
    og: bool,
    #[serde(default)]
    jumped: bool,
    #[serde(default)]
    double_jumped: bool,
    #[serde(default)]
    supersonic: bool,
    #[serde(default = "default_boost")]
    boost: f32,
    #[serde(default)]
    inp: Vec<ControlRun>,
}

const fn default_boost() -> f32 {
    100.0
}

#[derive(Debug, Clone, Copy, Default, Deserialize)]
struct JsonControls {
    #[serde(default)]
    th: f32,
    #[serde(default)]
    st: f32,
    #[serde(default)]
    p: f32,
    #[serde(default)]
    y: f32,
    #[serde(default)]
    r: f32,
    #[serde(default)]
    j: bool,
    #[serde(default)]
    bo: bool,
    #[serde(default)]
    hb: bool,
}

impl From<JsonControls> for CarControls {
    fn from(value: JsonControls) -> Self {
        Self {
            throttle: value.th,
            steer: value.st,
            pitch: value.p,
            yaw: value.y,
            roll: value.r,
            jump: value.j,
            boost: value.bo,
            handbrake: value.hb,
        }
        .clamp()
    }
}

#[derive(Debug, Deserialize)]
struct ControlRun {
    c: JsonControls,
    d: usize,
}

#[derive(Debug, Deserialize)]
struct LiveResponse {
    ok: bool,
    #[serde(default)]
    schema_version: Option<u32>,
    #[serde(default)]
    tick_rate: Option<u32>,
    #[serde(default)]
    state_phase: Option<String>,
    #[serde(default)]
    control_phase: Option<String>,
    #[serde(default)]
    name: String,
    num_cars: usize,
    ticks: Vec<LiveTick>,
}

#[derive(Debug, Deserialize)]
struct LiveTick {
    t: usize,
    bp: [f32; 3],
    bv: [f32; 3],
    ba: [f32; 3],
    #[serde(default)]
    c0p: [f32; 3],
    #[serde(default)]
    c0v: [f32; 3],
    #[serde(default)]
    c0a: [f32; 3],
    #[serde(default)]
    c0r: [f32; 3],
    #[serde(default)]
    c0og: bool,
    #[serde(default)]
    c0bo: f32,
    #[serde(default)]
    c0ctl: Option<JsonControls>,
    #[serde(default)]
    c0state: Option<LiveCarState>,
    #[serde(default)]
    c1p: [f32; 3],
    #[serde(default)]
    c1v: [f32; 3],
    #[serde(default)]
    c1a: [f32; 3],
    #[serde(default)]
    c1r: [f32; 3],
    #[serde(default)]
    c1og: bool,
    #[serde(default)]
    c1bo: f32,
    #[serde(default)]
    c1ctl: Option<JsonControls>,
    #[serde(default)]
    c1state: Option<LiveCarState>,
}

#[derive(Debug, Clone, Copy, Default, Deserialize)]
struct LiveCarState {
    #[serde(default)]
    jumped: bool,
    #[serde(default)]
    double_jumped: bool,
    #[serde(default)]
    is_jumping: bool,
    #[serde(default)]
    is_flipping: bool,
    #[serde(default)]
    supersonic: bool,
    #[serde(default)]
    jump_time: f32,
    #[serde(default)]
    flip_time: f32,
    #[serde(default)]
    air_time: f32,
    #[serde(default)]
    air_time_since_jump: f32,
    #[serde(default)]
    dodge_torque: [f32; 3],
}

impl LiveTick {
    fn car(&self, index: usize) -> LiveCar<'_> {
        match index {
            0 => LiveCar {
                pos: &self.c0p,
                vel: &self.c0v,
                ang_vel: &self.c0a,
                rot: &self.c0r,
                on_ground: self.c0og,
                boost: self.c0bo,
                controls: self.c0ctl,
                state: self.c0state,
            },
            1 => LiveCar {
                pos: &self.c1p,
                vel: &self.c1v,
                ang_vel: &self.c1a,
                rot: &self.c1r,
                on_ground: self.c1og,
                boost: self.c1bo,
                controls: self.c1ctl,
                state: self.c1state,
            },
            _ => unreachable!("the live logger currently supports at most two cars"),
        }
    }
}

struct LiveCar<'a> {
    pos: &'a [f32; 3],
    vel: &'a [f32; 3],
    ang_vel: &'a [f32; 3],
    rot: &'a [f32; 3],
    on_ground: bool,
    boost: f32,
    controls: Option<JsonControls>,
    state: Option<LiveCarState>,
}

#[derive(Debug, Clone, Serialize)]
struct Sample {
    tick: usize,
    entity: String,
    field: &'static str,
    error: f32,
    predicted: Vec<f32>,
    expected: Vec<f32>,
}

#[derive(Debug, Serialize)]
struct FieldSummary {
    entity: String,
    field: &'static str,
    count: usize,
    mean: f32,
    p95: f32,
    max: f32,
    max_tick: usize,
}

#[derive(Debug, Clone, Serialize)]
struct Trigger {
    tick: usize,
    entity: String,
    error: f32,
    window_start: usize,
    window_end: usize,
}

#[derive(Debug, Serialize)]
struct DiffOutput {
    name: String,
    requested_ticks: usize,
    ticks: usize,
    complete: bool,
    summary: Vec<FieldSummary>,
    worst: Vec<Sample>,
    triggers: Vec<Trigger>,
    #[serde(skip_serializing_if = "Option::is_none")]
    cpp_summary: Option<Vec<FieldSummary>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    cpp_worst: Option<Vec<Sample>>,
}

fn vec3(value: &[f32; 3]) -> Vec3A {
    Vec3A::from_array(*value)
}

fn controls_by_tick(car: &InitialCar, ticks: usize) -> Vec<JsonControls> {
    let mut result = Vec::with_capacity(ticks);
    for run in &car.inp {
        result.extend(std::iter::repeat_n(run.c, run.d.min(ticks - result.len())));
        if result.len() == ticks {
            break;
        }
    }
    let trailing = result.last().copied().unwrap_or_default();
    result.resize(ticks, trailing);
    result
}

fn initial_rotation(rot: &[f32; 3]) -> Mat3A {
    let [yaw, pitch, roll] = *rot;
    Mat3A::from_euler(EulerRot::ZYX, yaw, pitch, roll)
}

fn response_rotation(rot: &[f32; 3]) -> Mat3A {
    const UE_TO_RAD: f32 = std::f32::consts::PI / 32768.0;
    let [pitch, yaw, roll] = *rot;
    Mat3A::from_euler(
        EulerRot::ZYX,
        yaw * UE_TO_RAD,
        pitch * UE_TO_RAD,
        roll * UE_TO_RAD,
    )
}

fn validate_phase(response: &LiveResponse) -> Result<(), Box<dyn Error>> {
    if response.schema_version != Some(9) {
        return Err("capture must use schema_version 9".into());
    }
    if response.tick_rate != Some(120)
        || response.state_phase.as_deref() != Some("active_tick_post_hook")
        || response.control_phase.as_deref() != Some("set_vehicle_input_pre_hook")
    {
        return Err("capture tick rate or phase semantics are incompatible".into());
    }
    Ok(())
}

fn build_triggers(
    samples: &[Sample],
    threshold: f32,
    pre_ticks: usize,
    post_ticks: usize,
    final_tick: usize,
) -> Vec<Trigger> {
    let mut by_entity: BTreeMap<&str, Vec<&Sample>> = BTreeMap::new();
    for sample in samples
        .iter()
        .filter(|sample| sample.field == "pos" && sample.error > threshold)
    {
        by_entity.entry(&sample.entity).or_default().push(sample);
    }

    let mut triggers = Vec::new();
    for (entity, mut events) in by_entity {
        events.sort_by_key(|sample| sample.tick);
        let mut run_start = 0;
        while run_start < events.len() {
            let mut run_end = run_start + 1;
            let mut worst = events[run_start];
            while run_end < events.len()
                && events[run_end].tick <= events[run_end - 1].tick.saturating_add(1)
            {
                if events[run_end].error > worst.error {
                    worst = events[run_end];
                }
                run_end += 1;
            }
            triggers.push(Trigger {
                tick: worst.tick,
                entity: entity.to_owned(),
                error: worst.error,
                window_start: events[run_start].tick.saturating_sub(pre_ticks).max(1),
                window_end: events[run_end - 1]
                    .tick
                    .saturating_add(post_ticks)
                    .min(final_tick),
            });
            run_start = run_end;
        }
    }
    triggers.sort_by(|a, b| a.tick.cmp(&b.tick).then_with(|| a.entity.cmp(&b.entity)));
    triggers
}

fn restore_initial_car(state: &mut CarState, initial: &InitialCar) {
    *state = CarState::default();
    state.phys.pos = vec3(&initial.p);
    state.phys.vel = vec3(&initial.v);
    state.phys.ang_vel = vec3(&initial.av);
    state.phys.rot_mat = initial_rotation(&initial.rot);
    state.is_on_ground = initial.og;
    state.wheels_with_contact = [initial.og; 4];
    state.has_jumped = initial.jumped;
    state.has_double_jumped = initial.double_jumped;
    state.is_supersonic = initial.supersonic;
    state.boost = initial.boost;
}

fn restore_live_car(
    state: &mut CarState,
    live: LiveCar<'_>,
    previous_controls: JsonControls,
) -> Result<(), Box<dyn Error>> {
    let logged = live
        .state
        .ok_or("schema-v9 capture tick is missing a car state")?;
    *state = CarState::default();
    state.phys.pos = vec3(live.pos);
    state.phys.vel = vec3(live.vel);
    state.phys.ang_vel = vec3(live.ang_vel);
    state.phys.rot_mat = response_rotation(live.rot);
    state.is_on_ground = live.on_ground;
    state.wheels_with_contact = [live.on_ground; 4];
    state.has_jumped = logged.jumped;
    state.has_flipped = logged.double_jumped && logged.flip_time > 0.0;
    state.has_double_jumped = logged.double_jumped && !state.has_flipped;
    state.is_jumping = logged.is_jumping;
    state.is_flipping = logged.is_flipping;
    state.jump_ticks = (logged.jump_time * rocketsim::consts::TICK_RATE).round() as u32;
    state.flip_time = logged.flip_time;
    state.air_time = logged.air_time;
    state.air_time_since_jump = logged.air_time_since_jump;
    state.flip_rel_torque = vec3(&logged.dodge_torque);
    state.is_supersonic = logged.supersonic;
    state.boost = live.boost;
    state.controls = previous_controls.into();
    state.prev_controls = previous_controls.into();
    Ok(())
}

fn rotation_error(predicted: Mat3A, expected: Mat3A) -> f32 {
    let relative = expected.transpose() * predicted;
    let trace = relative.x_axis.x + relative.y_axis.y + relative.z_axis.z;
    ((trace - 1.0) * 0.5).clamp(-1.0, 1.0).acos()
}

fn add_vec_sample(
    samples: &mut Vec<Sample>,
    tick: usize,
    entity: &str,
    field: &'static str,
    predicted: Vec3A,
    expected: Vec3A,
) {
    samples.push(Sample {
        tick,
        entity: entity.to_owned(),
        field,
        error: predicted.distance(expected),
        predicted: predicted.to_array().to_vec(),
        expected: expected.to_array().to_vec(),
    });
}

fn add_scalar_sample(
    samples: &mut Vec<Sample>,
    tick: usize,
    entity: &str,
    field: &'static str,
    predicted: f32,
    expected: f32,
) {
    samples.push(Sample {
        tick,
        entity: entity.to_owned(),
        field,
        error: (predicted - expected).abs(),
        predicted: vec![predicted],
        expected: vec![expected],
    });
}

fn summarize(samples: &[Sample]) -> Vec<FieldSummary> {
    let mut keys: Vec<(String, &'static str)> = samples
        .iter()
        .map(|sample| (sample.entity.clone(), sample.field))
        .collect();
    keys.sort_unstable();
    keys.dedup();

    keys.into_iter()
        .map(|(entity, field)| {
            let mut matching: Vec<&Sample> = samples
                .iter()
                .filter(|sample| sample.entity == entity && sample.field == field)
                .collect();
            matching.sort_by(|a, b| a.error.total_cmp(&b.error));
            let count = matching.len();
            let p95_index = count.saturating_sub(1) * 95 / 100;
            let max = matching[count - 1];
            FieldSummary {
                entity,
                field,
                count,
                mean: matching.iter().map(|sample| sample.error).sum::<f32>() / count as f32,
                p95: matching[p95_index].error,
                max: max.error,
                max_tick: max.tick,
            }
        })
        .collect()
}

fn rank_worst(mut samples: Vec<Sample>, count: usize) -> Vec<Sample> {
    samples.sort_by(|a, b| {
        b.error
            .partial_cmp(&a.error)
            .unwrap_or(Ordering::Equal)
            .then_with(|| a.tick.cmp(&b.tick))
    });
    samples.truncate(count);
    samples
}

#[cfg(feature = "cpp-compare")]
fn cpp_vec(value: &[f32; 3]) -> CppVec3 {
    let mut result = CppVec3::default();
    result.x = value[0];
    result.y = value[1];
    result.z = value[2];
    result
}

#[cfg(feature = "cpp-compare")]
fn cpp_vec3a(value: CppVec3) -> Vec3A {
    Vec3A::new(value.x, value.y, value.z)
}

#[cfg(feature = "cpp-compare")]
fn cpp_rotation(rotation: Mat3A) -> CppRotMat {
    CppRotMat {
        forward: cpp_vec(&rotation.x_axis.to_array()),
        right: cpp_vec(&rotation.y_axis.to_array()),
        up: cpp_vec(&rotation.z_axis.to_array()),
    }
}

#[cfg(feature = "cpp-compare")]
fn cpp_controls(value: JsonControls) -> CppCarControls {
    CppCarControls {
        throttle: value.th.clamp(-1.0, 1.0),
        steer: value.st.clamp(-1.0, 1.0),
        pitch: value.p.clamp(-1.0, 1.0),
        yaw: value.y.clamp(-1.0, 1.0),
        roll: value.r.clamp(-1.0, 1.0),
        jump: value.j,
        boost: value.bo,
        handbrake: value.hb,
    }
}

#[cfg(feature = "cpp-compare")]
fn evaluate_cpp(
    capture: &Capture,
    ticks: usize,
    request_controls: &[Vec<JsonControls>],
) -> Result<Vec<Sample>, Box<dyn Error>> {
    rocketsim_rs::init(
        Some(concat!(env!("CARGO_MANIFEST_DIR"), "/collision_meshes")),
        true,
    );
    let mut arena = CppArena::default_standard();
    let car_ids: Vec<u32> = capture
        .request
        .cars
        .iter()
        .enumerate()
        .map(|(index, _)| {
            arena.pin_mut().add_car(
                if index == 0 {
                    CppTeam::Blue
                } else {
                    CppTeam::Orange
                },
                CppCarConfig::octane(),
            )
        })
        .collect();

    if let Some(initial) = &capture.request.ball {
        let mut state = arena.pin_mut().get_ball();
        state.pos = cpp_vec(&initial.p);
        state.vel = cpp_vec(&initial.v);
        state.ang_vel = cpp_vec(&initial.av);
        arena.pin_mut().set_ball(state);
    } else {
        let mut state = arena.pin_mut().get_ball();
        state.pos = cpp_vec(&[0.0, 0.0, -9999.0]);
        state.vel = CppVec3::default();
        state.ang_vel = CppVec3::default();
        arena.pin_mut().set_ball(state);
    }

    for (index, initial) in capture.request.cars.iter().enumerate() {
        let mut state = arena.pin_mut().get_car(car_ids[index]);
        state.pos = cpp_vec(&initial.p);
        state.vel = cpp_vec(&initial.v);
        state.ang_vel = cpp_vec(&initial.av);
        state.rot_mat = cpp_rotation(initial_rotation(&initial.rot));
        state.is_on_ground = initial.og;
        state.wheels_with_contact = [initial.og; 4];
        state.has_jumped = initial.jumped;
        state.has_double_jumped = initial.double_jumped;
        state.is_supersonic = initial.supersonic;
        state.boost = initial.boost;
        state.last_controls = CppCarControls::default();
        arena
            .pin_mut()
            .set_car(car_ids[index], state)
            .expect("C++ car id vanished");
    }

    let mut samples = Vec::new();
    for (offset, expected) in capture.response.ticks.iter().take(ticks).enumerate() {
        if capture.request.ball.is_some() && offset > 0 {
            let previous = &capture.response.ticks[offset - 1];
            let mut state = arena.pin_mut().get_ball();
            state.pos = cpp_vec(&previous.bp);
            state.vel = cpp_vec(&previous.bv);
            state.ang_vel = cpp_vec(&previous.ba);
            arena.pin_mut().set_ball(state);
        }

        for (index, &car_id) in car_ids.iter().enumerate() {
            if offset > 0 {
                let previous = capture.response.ticks[offset - 1].car(index);
                let logged = previous
                    .state
                    .ok_or("schema-v9 capture tick is missing a car state")?;
                let previous_controls = previous
                    .controls
                    .unwrap_or(request_controls[index][offset - 1]);
                let mut state = arena.pin_mut().get_car(car_id);
                state.pos = cpp_vec(previous.pos);
                state.vel = cpp_vec(previous.vel);
                state.ang_vel = cpp_vec(previous.ang_vel);
                state.rot_mat = cpp_rotation(response_rotation(previous.rot));
                state.is_on_ground = previous.on_ground;
                state.wheels_with_contact = [previous.on_ground; 4];
                state.has_jumped = logged.jumped;
                state.has_flipped = logged.double_jumped && logged.flip_time > 0.0;
                state.has_double_jumped = logged.double_jumped && !state.has_flipped;
                state.is_jumping = logged.is_jumping;
                state.is_flipping = logged.is_flipping;
                state.jump_time = logged.jump_time;
                state.flip_time = logged.flip_time;
                state.air_time = logged.air_time;
                state.air_time_since_jump = logged.air_time_since_jump;
                state.flip_rel_torque = cpp_vec(&logged.dodge_torque);
                state.is_supersonic = logged.supersonic;
                state.boost = previous.boost;
                state.last_controls = cpp_controls(previous_controls);
                arena
                    .pin_mut()
                    .set_car(car_id, state)
                    .expect("C++ car id vanished");
            }
            let live = expected.car(index);
            let controls = live.controls.unwrap_or(request_controls[index][offset]);
            arena
                .pin_mut()
                .set_car_controls(car_id, cpp_controls(controls))
                .expect("C++ car id vanished");
        }
        arena.pin_mut().step(1);

        if capture.request.ball.is_some() {
            let predicted = arena.pin_mut().get_ball();
            add_vec_sample(
                &mut samples,
                expected.t,
                "ball",
                "pos",
                cpp_vec3a(predicted.pos),
                vec3(&expected.bp),
            );
            add_vec_sample(
                &mut samples,
                expected.t,
                "ball",
                "vel",
                cpp_vec3a(predicted.vel),
                vec3(&expected.bv),
            );
            add_vec_sample(
                &mut samples,
                expected.t,
                "ball",
                "ang_vel",
                cpp_vec3a(predicted.ang_vel),
                vec3(&expected.ba),
            );
        }

        for (index, &car_id) in car_ids.iter().enumerate() {
            let entity = format!("car_{index}");
            let predicted = arena.pin_mut().get_car(car_id);
            let live = expected.car(index);
            add_vec_sample(
                &mut samples,
                expected.t,
                &entity,
                "pos",
                cpp_vec3a(predicted.pos),
                vec3(live.pos),
            );
            add_vec_sample(
                &mut samples,
                expected.t,
                &entity,
                "vel",
                cpp_vec3a(predicted.vel),
                vec3(live.vel),
            );
            add_vec_sample(
                &mut samples,
                expected.t,
                &entity,
                "ang_vel",
                cpp_vec3a(predicted.ang_vel),
                vec3(live.ang_vel),
            );
            let rotation = Mat3A::from_cols(
                cpp_vec3a(predicted.rot_mat.forward),
                cpp_vec3a(predicted.rot_mat.right),
                cpp_vec3a(predicted.rot_mat.up),
            );
            add_scalar_sample(
                &mut samples,
                expected.t,
                &entity,
                "rot",
                rotation_error(rotation, response_rotation(live.rot)),
                0.0,
            );
            add_scalar_sample(
                &mut samples,
                expected.t,
                &entity,
                "boost",
                predicted.boost,
                live.boost,
            );
            add_scalar_sample(
                &mut samples,
                expected.t,
                &entity,
                "on_ground",
                u8::from(predicted.is_on_ground).into(),
                u8::from(live.on_ground).into(),
            );
        }
    }
    Ok(samples)
}

fn evaluate(
    capture: Capture,
    worst_count: usize,
    run_cpp: bool,
    position_threshold: f32,
    pre_ticks: usize,
    post_ticks: usize,
) -> Result<DiffOutput, Box<dyn Error>> {
    if !capture.response.ok {
        return Err("Rocket League response was not successful".into());
    }
    if capture.request.cars.len() != capture.response.num_cars {
        return Err(format!(
            "request has {} cars but response reports {}",
            capture.request.cars.len(),
            capture.response.num_cars
        )
        .into());
    }
    if capture.request.cars.len() > 2 {
        return Err("the live logger currently supports at most two cars".into());
    }
    if capture.request.ball.is_none() && capture.request.cars.is_empty() {
        return Err("scenario contains neither a ball nor a car".into());
    }
    validate_phase(&capture.response)?;
    for (offset, tick) in capture.response.ticks.iter().enumerate() {
        if tick.t != offset + 1 {
            return Err(format!(
                "capture ticks must be contiguous and one-based; index {offset} contains t{}",
                tick.t
            )
            .into());
        }
        for index in 0..capture.response.num_cars {
            if tick.car(index).state.is_none() {
                return Err(
                    format!("schema-v9 capture tick {} is missing c{index}state", tick.t).into(),
                );
            }
        }
    }
    if capture.response.ticks.len() > capture.request.ticks {
        return Err("response contains more ticks than requested".into());
    }

    let requested_ticks = capture.request.ticks;
    let ticks = capture.response.ticks.len();
    if ticks == 0 {
        return Err("capture contains no ticks".into());
    }

    rocketsim::init_from_default(true)?;
    let mut arena = Arena::new_with_config(ArenaConfig::new(GameMode::Soccar).with_rng_seed(0));
    let car_indices: Vec<usize> = capture
        .request
        .cars
        .iter()
        .enumerate()
        .map(|(index, _)| {
            arena.add_car(
                if index == 0 { Team::Blue } else { Team::Orange },
                CarBodyConfig::OCTANE,
            )
        })
        .collect();

    if let Some(initial) = &capture.request.ball {
        let mut state = *arena.get_ball_state();
        state.phys.pos = vec3(&initial.p);
        state.phys.vel = vec3(&initial.v);
        state.phys.ang_vel = vec3(&initial.av);
        arena.set_ball_state(state);
    } else {
        let mut state = *arena.get_ball_state();
        state.phys.pos = Vec3A::new(0.0, 0.0, -9999.0);
        state.phys.vel = Vec3A::ZERO;
        state.phys.ang_vel = Vec3A::ZERO;
        arena.set_ball_state(state);
    }

    for (index, initial) in capture.request.cars.iter().enumerate() {
        let mut state = *arena.get_car_state(car_indices[index]);
        state.phys.pos = vec3(&initial.p);
        state.phys.vel = vec3(&initial.v);
        state.phys.ang_vel = vec3(&initial.av);
        state.phys.rot_mat = initial_rotation(&initial.rot);
        state.is_on_ground = initial.og;
        state.wheels_with_contact = [initial.og; 4];
        state.has_jumped = initial.jumped;
        state.has_double_jumped = initial.double_jumped;
        state.is_supersonic = initial.supersonic;
        state.boost = initial.boost;
        arena.set_car_state(car_indices[index], state);
    }

    let request_controls: Vec<Vec<JsonControls>> = capture
        .request
        .cars
        .iter()
        .map(|car| controls_by_tick(car, ticks))
        .collect();
    let mut samples = Vec::new();

    for (offset, expected) in capture.response.ticks.iter().take(ticks).enumerate() {
        if capture.request.ball.is_some() {
            let mut state = BallState::default();
            if offset == 0 {
                let initial = capture.request.ball.as_ref().unwrap();
                state.phys.pos = vec3(&initial.p);
                state.phys.vel = vec3(&initial.v);
                state.phys.ang_vel = vec3(&initial.av);
            } else {
                let previous = &capture.response.ticks[offset - 1];
                state.phys.pos = vec3(&previous.bp);
                state.phys.vel = vec3(&previous.bv);
                state.phys.ang_vel = vec3(&previous.ba);
            }
            arena.set_ball_state(state);
        }

        for (index, &car_index) in car_indices.iter().enumerate() {
            let mut state = *arena.get_car_state(car_index);
            if offset == 0 {
                restore_initial_car(&mut state, &capture.request.cars[index]);
            } else {
                let previous = capture.response.ticks[offset - 1].car(index);
                let previous_controls = previous
                    .controls
                    .unwrap_or(request_controls[index][offset - 1]);
                restore_live_car(&mut state, previous, previous_controls)?;
            }
            arena.set_car_state(car_index, state);

            let live = expected.car(index);
            let controls = live.controls.unwrap_or(request_controls[index][offset]);
            arena.set_car_controls(car_index, controls.into());
        }

        arena.step_tick();

        if capture.request.ball.is_some() {
            let predicted = arena.get_ball_state();
            add_vec_sample(
                &mut samples,
                expected.t,
                "ball",
                "pos",
                predicted.phys.pos,
                vec3(&expected.bp),
            );
            add_vec_sample(
                &mut samples,
                expected.t,
                "ball",
                "vel",
                predicted.phys.vel,
                vec3(&expected.bv),
            );
            add_vec_sample(
                &mut samples,
                expected.t,
                "ball",
                "ang_vel",
                predicted.phys.ang_vel,
                vec3(&expected.ba),
            );
        }

        for (index, &car_index) in car_indices.iter().enumerate() {
            let entity = format!("car_{index}");
            let predicted = arena.get_car_state(car_index);
            let expected_car = expected.car(index);
            add_vec_sample(
                &mut samples,
                expected.t,
                &entity,
                "pos",
                predicted.phys.pos,
                vec3(expected_car.pos),
            );
            add_vec_sample(
                &mut samples,
                expected.t,
                &entity,
                "vel",
                predicted.phys.vel,
                vec3(expected_car.vel),
            );
            add_vec_sample(
                &mut samples,
                expected.t,
                &entity,
                "ang_vel",
                predicted.phys.ang_vel,
                vec3(expected_car.ang_vel),
            );
            add_scalar_sample(
                &mut samples,
                expected.t,
                &entity,
                "rot",
                rotation_error(predicted.phys.rot_mat, response_rotation(expected_car.rot)),
                0.0,
            );
            add_scalar_sample(
                &mut samples,
                expected.t,
                &entity,
                "boost",
                predicted.boost,
                expected_car.boost,
            );
            add_scalar_sample(
                &mut samples,
                expected.t,
                &entity,
                "on_ground",
                u8::from(predicted.is_on_ground).into(),
                u8::from(expected_car.on_ground).into(),
            );
        }
    }

    let triggers = build_triggers(&samples, position_threshold, pre_ticks, post_ticks, ticks);
    let summary = summarize(&samples);
    let worst = rank_worst(samples, worst_count);

    #[cfg(feature = "cpp-compare")]
    let (cpp_summary, cpp_worst) = if run_cpp {
        let cpp_samples = evaluate_cpp(&capture, ticks, &request_controls)?;
        (
            Some(summarize(&cpp_samples)),
            Some(rank_worst(cpp_samples, worst_count)),
        )
    } else {
        (None, None)
    };
    #[cfg(not(feature = "cpp-compare"))]
    let (cpp_summary, cpp_worst) = if run_cpp {
        return Err("--cpp requires building rl_live_diff with --features cpp-compare".into());
    } else {
        (None, None)
    };

    let name = if capture.response.name.is_empty() {
        capture.request.name
    } else {
        capture.response.name
    };
    Ok(DiffOutput {
        name,
        requested_ticks,
        ticks,
        complete: ticks == requested_ticks,
        summary,
        worst,
        triggers,
        cpp_summary,
        cpp_worst,
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let input = fs::read_to_string(args.input)?;
    let capture: Capture = serde_json::from_str(&input)?;
    let output = evaluate(
        capture,
        args.worst,
        args.cpp,
        args.position_threshold,
        args.pre_ticks,
        args.post_ticks,
    )?;
    println!("{}", serde_json::to_string_pretty(&output)?);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn expands_and_holds_last_rle_controls() {
        let car = InitialCar {
            p: [0.0; 3],
            v: [0.0; 3],
            av: [0.0; 3],
            rot: [0.0; 3],
            og: true,
            jumped: false,
            double_jumped: false,
            supersonic: false,
            boost: 100.0,
            inp: vec![ControlRun {
                c: JsonControls {
                    th: 1.0,
                    ..JsonControls::default()
                },
                d: 2,
            }],
        };

        let controls = controls_by_tick(&car, 3);
        assert_eq!(controls.len(), 3);
        assert_eq!(controls[0].th, 1.0);
        assert_eq!(controls[1].th, 1.0);
        assert_eq!(controls[2].th, 1.0);
    }

    #[test]
    fn rotation_error_is_zero_for_equal_rotations() {
        let rotation = Mat3A::from_euler(EulerRot::ZYX, 0.4, -0.2, 0.7);
        assert!(rotation_error(rotation, rotation) < 1e-6);
    }

    #[test]
    fn coalesces_adjacent_position_triggers_and_expands_windows() {
        let samples = [
            Sample {
                tick: 10,
                entity: "car_0".to_owned(),
                field: "pos",
                error: 2.0,
                predicted: vec![],
                expected: vec![],
            },
            Sample {
                tick: 11,
                entity: "car_0".to_owned(),
                field: "pos",
                error: 3.0,
                predicted: vec![],
                expected: vec![],
            },
            Sample {
                tick: 13,
                entity: "car_0".to_owned(),
                field: "pos",
                error: 4.0,
                predicted: vec![],
                expected: vec![],
            },
            Sample {
                tick: 11,
                entity: "ball".to_owned(),
                field: "pos",
                error: 5.0,
                predicted: vec![],
                expected: vec![],
            },
        ];

        let triggers = build_triggers(&samples, 1.0, 3, 4, 15);
        assert_eq!(triggers.len(), 3);
        assert_eq!(triggers[0].tick, 11);
        assert_eq!(triggers[0].entity, "ball");
        assert_eq!(triggers[0].window_start, 8);
        assert_eq!(triggers[0].window_end, 15);
        assert_eq!(triggers[1].tick, 11);
        assert_eq!(triggers[1].entity, "car_0");
        assert_eq!(triggers[1].error, 3.0);
        assert_eq!(triggers[1].window_start, 7);
        assert_eq!(triggers[1].window_end, 15);
        assert_eq!(triggers[2].tick, 13);
        assert_eq!(triggers[2].window_start, 10);
        assert_eq!(triggers[2].window_end, 15);
    }

    #[test]
    fn accepts_only_current_logger_phase_metadata() {
        let mut response = LiveResponse {
            ok: true,
            schema_version: Some(9),
            tick_rate: Some(120),
            state_phase: Some("active_tick_post_hook".to_owned()),
            control_phase: Some("set_vehicle_input_pre_hook".to_owned()),
            name: String::new(),
            num_cars: 0,
            ticks: vec![],
        };
        assert!(validate_phase(&response).is_ok());

        response.state_phase = Some("post_physics".to_owned());
        assert!(validate_phase(&response).is_err());
        response.state_phase = Some("active_tick_post_hook".to_owned());
        response.schema_version = Some(8);
        assert!(validate_phase(&response).is_err());
    }

    #[test]
    fn evaluates_a_ball_only_capture() {
        let capture = Capture {
            request: Scenario {
                name: "unit".to_owned(),
                ticks: 1,
                ball: Some(InitialBody {
                    p: [0.0, 0.0, 200.0],
                    v: [0.0; 3],
                    av: [0.0; 3],
                }),
                cars: vec![],
            },
            response: LiveResponse {
                ok: true,
                schema_version: Some(9),
                tick_rate: Some(120),
                state_phase: Some("active_tick_post_hook".to_owned()),
                control_phase: Some("set_vehicle_input_pre_hook".to_owned()),
                name: "unit".to_owned(),
                num_cars: 0,
                ticks: vec![LiveTick {
                    t: 1,
                    bp: [0.0, 0.0, 200.0],
                    bv: [0.0; 3],
                    ba: [0.0; 3],
                    c0p: [0.0; 3],
                    c0v: [0.0; 3],
                    c0a: [0.0; 3],
                    c0r: [0.0; 3],
                    c0og: false,
                    c0bo: 0.0,
                    c0ctl: None,
                    c0state: None,
                    c1p: [0.0; 3],
                    c1v: [0.0; 3],
                    c1a: [0.0; 3],
                    c1r: [0.0; 3],
                    c1og: false,
                    c1bo: 0.0,
                    c1ctl: None,
                    c1state: None,
                }],
            },
        };

        let output = evaluate(capture, 5, cfg!(feature = "cpp-compare"), 1.0, 120, 120).unwrap();
        assert_eq!(output.ticks, 1);
        assert!(output.complete);
        assert_eq!(output.summary.len(), 3);
        assert_eq!(output.worst.len(), 3);
        #[cfg(feature = "cpp-compare")]
        {
            assert_eq!(output.cpp_summary.as_ref().unwrap().len(), 3);
            assert_eq!(output.cpp_worst.as_ref().unwrap().len(), 3);
        }
    }
}
