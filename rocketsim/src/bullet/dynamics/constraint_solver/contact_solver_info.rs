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
pub const WHEEL_PUSHBACK_ERP: f32 = 0.1;
pub const ERP_2: f32 = 0.8;
pub const SPLIT_IMPULSE_PENETRATION_THRESHOLD: f32 = 1e30;
pub const SPLIT_IMPULSE_TURN_ERP: f32 = 0.1;
pub const WARMSTARTING_FACTOR: f32 = 0.85;
/// Below this relative normal speed a contact gets zero restitution.
///
/// This is Bullet's `m_restitutionVelocityThreshold`, and 0.2 is the default in
/// the bullet3-3.24 that RocketSim vendors (`btContactSolverInfo.h`). RocketSim
/// overrides exactly two solver-info values -- `m_splitImpulsePenetrationThreshold`
/// and `m_erp2` -- and this is not one of them, so 0.2 is what the C++ engine runs.
///
/// It was briefly raised to 1.0 on the claim that 1.0 was Bullet's default; it is
/// not. The port is BT-native, so the constant is BT/s: 0.2 is 10 uu/s and 1.0 is
/// 50 uu/s, and at 1.0 every contact closing between those speeds lost its bounce
/// -- the settling/rolling regime. Restoring 0.2 measured monotonically better on
/// a 0.2/0.4/0.6/1.0 sweep (ball vel 4.1899 -> 4.1776, ball pos 0.0378 -> 0.0377,
/// car suite mean 2.4943 -> 2.4935) with the gate's pass/fail set unchanged.
pub const RESTITUTION_VELOCITY_THRESHOLD: f32 = 0.2;
