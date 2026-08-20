pub const NUM_ITERATIONS: usize = 10;
pub const SOR: f32 = 1.0;
/// Error-reduction parameter for the wheel suspension ray's penetration
/// pushback (`resolve_single_collision`, whose only call site is
/// `WheelInfo::apply_ray_cast`).
///
/// Bullet's `m_erp` is 0.2 and both RocketSim ports use it here, but ground
/// truth says half that. Measured 2026-08-20 over the whole recording suite:
/// the sim's velocity along the car's own up axis runs ahead of Rocket League's
/// only once the suspension compresses past `SUSPENSION_SUBTRACTION` (2.5 UU),
/// which is exactly where this pushback term switches on. The required
/// multiplier on the term is flat at ~0.72 across every compression depth from
/// 3 to 11 UU, whereas no single multiplier on the spring force fits (it slides
/// 0.91 -> 0.57), and a force clamp is ruled out because it would need the
/// multiplier to fall with depth. Scaling only this positional term — leaving
/// the velocity term, which is a real collision response, alone — minimises
/// total per-step car error at 0.10, cutting the suite mean 4.7793 -> 4.5892
/// uu/s (-3.98%) and the wall/ceiling bucket by 16%.
///
/// Rocket League's Bullet dates from 2013-2015, and a factor of exactly two in
/// an error-reduction parameter is a plausible version difference; 0.1 is also
/// already the value of `SPLIT_IMPULSE_TURN_ERP` below. See `how-to-test.md`.
pub const RAY_PUSHBACK_ERP: f32 = 0.1;
pub const ERP_2: f32 = 0.8;
pub const SPLIT_IMPULSE_PENETRATION_THRESHOLD: f32 = 1e30;
pub const SPLIT_IMPULSE_TURN_ERP: f32 = 0.1;
pub const WARMSTARTING_FACTOR: f32 = 0.85;
pub const RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.2;
