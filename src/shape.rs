use crate::{
    aabb::AABB,
    bvh::NULL_PTR,
    math::{Complex, Vec2},
    rigidbody::RigidBody,
};

#[derive(Clone, Copy)]
pub enum ShapeType {
    Circle(Circle),
    Rect(Rect),
}
impl ShapeType {
    #[inline(always)]
    pub fn get_id(&self) -> usize {
        match self {
            ShapeType::Circle(_) => 0,
            ShapeType::Rect(_) => 1,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Shape {
    pub shape_type: ShapeType,
    pub body_ptr: usize,
    pub node_ptr: usize,

    pub pos: Vec2,
    pub angle: f64,
    pub heading: Complex,
    pub aabb: AABB,
    pub fat_aabb: AABB,
}

impl Shape {
    pub fn new(shape_type: ShapeType, body_ptr: usize) -> Self {
        Self {
            shape_type,
            body_ptr,
            node_ptr: NULL_PTR,
            pos: Vec2::zero(),
            angle: 0.0,
            heading: Complex::new(1.0, 0.0),
            aabb: AABB::empty(),
            fat_aabb: AABB::empty(),
        }
    }

    pub fn refresh_transform(&mut self, body: &RigidBody) {
        self.pos = body.pos;
        self.angle = body.angle;
        self.heading = body.heading;

        match &self.shape_type {
            ShapeType::Circle(circle) => {
                self.aabb = AABB::new(
                    Vec2::new(self.pos.x - circle.radius, self.pos.y - circle.radius),
                    Vec2::new(self.pos.x + circle.radius, self.pos.y + circle.radius),
                );
            }

            ShapeType::Rect(rect) => {
                let abs_hx = self.heading.re.abs();
                let abs_hy = self.heading.im.abs();
                let dx = rect.hw * abs_hx + rect.hh * abs_hy;
                let dy = rect.hw * abs_hy + rect.hh * abs_hx;

                self.aabb = AABB::new(
                    Vec2::new(self.pos.x - dx, self.pos.y - dy),
                    Vec2::new(self.pos.x + dx, self.pos.y + dy),
                );
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct Circle {
    pub radius: f64,
}

impl Circle {
    pub fn new(radius: f64) -> Self {
        Self { radius }
    }
}

#[derive(Clone, Copy)]
pub struct Rect {
    pub hw: f64,
    pub hh: f64,
}

impl Rect {
    pub fn new(width: f64, height: f64) -> Self {
        Self {
            hw: width * 0.5,
            hh: height * 0.5,
        }
    }
}
