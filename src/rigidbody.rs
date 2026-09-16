use crate::math::{Complex, PI, TWO_PI, Vec2};

pub struct RigidBody {
    pub pos: Vec2,
    pub centroid: Vec2,
    pub loc_centroid: Vec2,
    pub angle: f64,
    pub heading: Complex,
    pub density: f64,
    pub vel: Vec2,
    pub ang_vel: f64,
    pub inv_m: f64,
    pub inv_i: f64,
    pub shape_ptr: usize,
    pub restitution: f64,
    pub friction: f64,
}

impl RigidBody {
    pub fn new(density: f64) -> Self {
        Self {
            pos: Vec2::zero(),
            centroid: Vec2::zero(),
            loc_centroid: Vec2::zero(),
            angle: 0f64,
            heading: Complex::new(1f64, 0f64),
            density,
            vel: Vec2::zero(),
            ang_vel: 0f64,
            inv_m: 0f64,
            inv_i: 0f64,
            shape_ptr: 0,
            restitution: 0.2f64,
            friction: 0.4f64,
        }
    }

    pub fn set_pos(&mut self, pos: Vec2) {
        self.pos = pos;
        self.centroid = self.pos;
    }

    pub fn set_angle(&mut self, angle: f64) {
        self.angle = angle % TWO_PI;
        self.heading = Complex::new(angle.cos(), angle.sin());
    }

    pub fn calc_circle_properties(&mut self, radius: f64) {
        if self.density > 0.0 {
            let r_sq = radius * radius;
            self.inv_m = 1.0 / (PI * r_sq * self.density);
            self.inv_i = 1.0 / (0.5 * r_sq);
        } else {
            self.inv_m = 0.0;
            self.inv_i = 0.0;
        }
        self.loc_centroid = Vec2::zero();
    }

    pub fn apply_impulse(&mut self, impulse: &Vec2, arm: &Vec2) {
        self.vel = self.vel.add(&impulse.scale(self.inv_m));
        self.ang_vel += arm.cross(impulse) * self.inv_i;
    }
}
