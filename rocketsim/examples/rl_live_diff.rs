//! Compare a scripted Rocket League TCP capture against this RocketSim build.

use std::{cmp::Ordering, error::Error, fs, path::PathBuf};

use clap::Parser;
use glam::EulerRot;
use rocketsim::{Arena, ArenaConfig, CarBodyConfig, CarControls, GameMode, Mat3A, Team, Vec3A};
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
            },
            1 => LiveCar {
                pos: &self.c1p,
                vel: &self.c1v,
                ang_vel: &self.c1a,
                rot: &self.c1r,
                on_ground: self.c1og,
                boost: self.c1bo,
                controls: self.c1ctl,
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

#[derive(Debug, Serialize)]
struct DiffOutput {
    name: String,
    requested_ticks: usize,
    ticks: usize,
    complete: bool,
    summary: Vec<FieldSummary>,
    worst: Vec<Sample>,
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

fn rl_rotation(rot: &[f32; 3]) -> Mat3A {
    const UE_TO_RAD: f32 = std::f32::consts::PI / 32768.0;
    let [pitch, yaw, roll] = *rot;
    Mat3A::from_euler(
        EulerRot::ZYX,
        yaw * UE_TO_RAD,
        pitch * UE_TO_RAD,
        roll * UE_TO_RAD,
    )
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
) -> Vec<Sample> {
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
        let [yaw, pitch, roll] = initial.rot;
        state.rot_mat = cpp_rotation(Mat3A::from_euler(EulerRot::ZYX, yaw, pitch, roll));
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
        for (index, &car_id) in car_ids.iter().enumerate() {
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
                rotation_error(rotation, rl_rotation(live.rot)),
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
    samples
}

fn evaluate(
    capture: Capture,
    worst_count: usize,
    run_cpp: bool,
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
    if capture.response.schema_version.is_none()
        && (capture.response.tick_rate.is_some()
            || capture.response.state_phase.is_some()
            || capture.response.control_phase.is_some())
    {
        return Err("capture phase metadata requires a schema_version".into());
    }
    if let Some(version) = capture.response.schema_version {
        if version < 8 {
            return Err("capture schema metadata is present but older than v8".into());
        }
        if capture.response.tick_rate != Some(120)
            || capture.response.state_phase.as_deref() != Some("post_physics")
            || capture.response.control_phase.as_deref() != Some("pre_physics")
        {
            return Err("capture tick rate or phase semantics are incompatible".into());
        }
    }
    for (offset, tick) in capture.response.ticks.iter().enumerate() {
        if tick.t != offset + 1 {
            return Err(format!(
                "capture ticks must be contiguous and one-based; index {offset} contains t{}",
                tick.t
            )
            .into());
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
        let [yaw, pitch, roll] = initial.rot;
        state.phys.rot_mat = Mat3A::from_euler(EulerRot::ZYX, yaw, pitch, roll);
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
        for (index, &car_index) in car_indices.iter().enumerate() {
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
                rotation_error(predicted.phys.rot_mat, rl_rotation(expected_car.rot)),
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

    let summary = summarize(&samples);
    let worst = rank_worst(samples, worst_count);

    #[cfg(feature = "cpp-compare")]
    let (cpp_summary, cpp_worst) = if run_cpp {
        let cpp_samples = evaluate_cpp(&capture, ticks, &request_controls);
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
        cpp_summary,
        cpp_worst,
    })
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let input = fs::read_to_string(args.input)?;
    let capture: Capture = serde_json::from_str(&input)?;
    let output = evaluate(capture, args.worst, args.cpp)?;
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
                schema_version: Some(8),
                tick_rate: Some(120),
                state_phase: Some("post_physics".to_owned()),
                control_phase: Some("pre_physics".to_owned()),
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
                    c1p: [0.0; 3],
                    c1v: [0.0; 3],
                    c1a: [0.0; 3],
                    c1r: [0.0; 3],
                    c1og: false,
                    c1bo: 0.0,
                    c1ctl: None,
                }],
            },
        };

        let output = evaluate(capture, 5, cfg!(feature = "cpp-compare")).unwrap();
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
