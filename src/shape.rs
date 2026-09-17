use crate::{
    aabb::AABB,
    bvh::NULL_PTR,
    math::{Complex, Vec2},
    rigidbody::RigidBody,
};

#[derive(Clone, Copy)]
pub enum Shape {
    Circle(usize),
}

#[derive(Clone, Copy)]
pub struct Circle {
    pub pos: Vec2,
    pub angle: f64,
    pub heading: Complex,
    pub index: usize,
    pub radius: f64,
    pub body_ptr: usize,

    pub node_ptr: usize,
    pub aabb: AABB,
    pub fat_aabb: AABB,
}

impl Circle {
    pub fn new(radius: f64, body_ptr: usize) -> Self {
        Self {
            pos: Vec2::zero(),
            angle: 0f64,
            heading: Complex::new(1.0, 0.0),
            radius,
            index: 0,
            body_ptr,
            node_ptr: NULL_PTR,
            aabb: AABB::empty(),
            fat_aabb: AABB::empty(),
        }
    }

    pub fn refresh_transform(&mut self, body: &RigidBody) {
        self.pos = body.pos;
        self.angle = body.angle;
        self.heading = body.heading;

        self.aabb = AABB::new(
            Vec2::new(self.pos.x - self.radius, self.pos.y - self.radius),
            Vec2::new(self.pos.x + self.radius, self.pos.y + self.radius),
        );
    }

    pub fn get_aabb(&self) -> AABB {
        self.aabb
    }
}
