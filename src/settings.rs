pub const GRAVITY_X: f64 = 0.0;
pub const GRAVITY_Y: f64 = -9.8 * 10.0;
pub const FPS: u16 = 60;
pub const LINEAR_DRAG: f64 = 0.99;
pub const ANGULAR_DRAG: f64 = 0.99;
pub const EPS: f64 = 1e-6f64;

pub const DT: f64 = 1.0 / FPS as f64;
pub const DELTA_VEL_X: f64 = GRAVITY_X * DT;
pub const DELTA_VEL_Y: f64 = GRAVITY_Y * DT;
pub const ANG_VEL_MULT: f64 = ANGULAR_DRAG * DT;
