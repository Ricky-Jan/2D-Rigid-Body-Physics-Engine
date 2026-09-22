use crate::{
    bvh::NULL_PTR,
    math::{Complex, TWO_PI, Vec2},
    settings::{DEFAULT_FRICTION, DEFAULT_RESTITUTION},
};

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
    pub shape_head: usize,
    pub restitution: f64,
    pub friction: f64,
    pub is_regular: bool,
    pub is_awake: bool,
    pub sleep_timer: f64,
    pub island_id: usize,
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
            shape_head: NULL_PTR,
            restitution: DEFAULT_RESTITUTION,
            friction: DEFAULT_FRICTION,
            is_regular: true,
            is_awake: true,
            sleep_timer: 0.0,
            island_id: 0,
        }
    }

    pub fn set_pos(&mut self, pos: Vec2) {
        self.pos = pos;
        if self.is_regular {
            self.centroid = self.pos;
        } else {
            self.centroid = self.pos.add(&self.heading.rotate(&self.loc_centroid));
        }
    }

    pub fn set_angle(&mut self, angle: f64) {
        self.angle = angle % TWO_PI;
        self.heading = Complex::new(self.angle.cos(), self.angle.sin());
        if self.is_regular {
            self.centroid = self.pos;
        } else {
            self.centroid = self.pos.add(&self.heading.rotate(&self.loc_centroid));
        }
    }

    pub fn wake_up(&mut self) {
        self.is_awake = true;
        self.sleep_timer = 0.0;
    }

    pub fn apply_impulse(&mut self, impulse: &Vec2, arm: &Vec2) {
        self.vel = self.vel.add(&impulse.scale(self.inv_m));
        self.ang_vel += arm.cross(impulse) * self.inv_i;
    }
}
