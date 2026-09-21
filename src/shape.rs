use crate::{
    aabb::AABB,
    bvh::NULL_PTR,
    math::{Complex, NEG_INF, POS_INF, TWO_PI, Vec2},
    rigidbody::RigidBody,
};

pub const MAX_POLY_VERTICES: usize = 8;

#[derive(Clone, Copy)]
pub enum ShapeType {
    Circle(Circle),
    Rect(Rect),
    Polygon(Polygon),
}

impl ShapeType {
    #[inline(always)]
    pub fn get_id(&self) -> usize {
        match self {
            ShapeType::Circle(_) => 0,
            ShapeType::Rect(_) => 1,
            ShapeType::Polygon(_) => 2,
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

        match &mut self.shape_type {
            ShapeType::Circle(circle) => {
                self.aabb = AABB::new(
                    Vec2::new(self.pos.x - circle.radius, self.pos.y - circle.radius),
                    Vec2::new(self.pos.x + circle.radius, self.pos.y + circle.radius),
                );
            }

            ShapeType::Rect(rect) => {
                for i in 0..4 {
                    rect.world_vertices[i] =
                        self.pos.add(&self.heading.rotate(&rect.local_vertices[i]));
                    rect.world_normals[i] = self.heading.rotate(&rect.local_normals[i]);
                }

                let abs_hx = self.heading.re.abs();
                let abs_hy = self.heading.im.abs();
                let dx = rect.hw * abs_hx + rect.hh * abs_hy;
                let dy = rect.hw * abs_hy + rect.hh * abs_hx;

                self.aabb = AABB::new(
                    Vec2::new(self.pos.x - dx, self.pos.y - dy),
                    Vec2::new(self.pos.x + dx, self.pos.y + dy),
                );
            }

            ShapeType::Polygon(poly) => {
                let mut min_x = POS_INF;
                let mut min_y = POS_INF;
                let mut max_x = NEG_INF;
                let mut max_y = NEG_INF;

                for i in 0..poly.count {
                    let world_pos = self.pos.add(&self.heading.rotate(&poly.local_vertices[i]));
                    poly.world_vertices[i] = world_pos;
                    poly.world_normals[i] = self.heading.rotate(&poly.local_normals[i]);

                    if world_pos.x < min_x {
                        min_x = world_pos.x;
                    }
                    if world_pos.x > max_x {
                        max_x = world_pos.x;
                    }
                    if world_pos.y < min_y {
                        min_y = world_pos.y;
                    }
                    if world_pos.y > max_y {
                        max_y = world_pos.y;
                    }
                }

                self.aabb = AABB::new(Vec2::new(min_x, min_y), Vec2::new(max_x, max_y));
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
    pub local_vertices: [Vec2; 4],
    pub local_normals: [Vec2; 4],
    pub world_vertices: [Vec2; 4],
    pub world_normals: [Vec2; 4],
}

impl Rect {
    pub fn new(width: f64, height: f64) -> Self {
        let hw = width * 0.5;
        let hh = height * 0.5;

        Self {
            hw,
            hh,
            local_vertices: [
                Vec2::new(hw, -hh),
                Vec2::new(hw, hh),
                Vec2::new(-hw, hh),
                Vec2::new(-hw, -hh),
            ],
            local_normals: [
                Vec2::new(1.0, 0.0),
                Vec2::new(0.0, 1.0),
                Vec2::new(-1.0, 0.0),
                Vec2::new(0.0, -1.0),
            ],
            world_vertices: [Vec2::zero(); 4],
            world_normals: [Vec2::zero(); 4],
        }
    }
}

#[derive(Clone, Copy)]
pub struct Polygon {
    pub count: usize,
    pub local_vertices: [Vec2; MAX_POLY_VERTICES],
    pub local_normals: [Vec2; MAX_POLY_VERTICES],
    pub world_vertices: [Vec2; MAX_POLY_VERTICES],
    pub world_normals: [Vec2; MAX_POLY_VERTICES],
}

impl Polygon {
    pub fn new_regular(sides: usize, radius: f64) -> Self {
        let count = sides.clamp(3, MAX_POLY_VERTICES);
        let mut poly = Self {
            count,
            local_vertices: [Vec2::zero(); MAX_POLY_VERTICES],
            local_normals: [Vec2::zero(); MAX_POLY_VERTICES],
            world_vertices: [Vec2::zero(); MAX_POLY_VERTICES],
            world_normals: [Vec2::zero(); MAX_POLY_VERTICES],
        };

        let angle_step = TWO_PI / (count as f64);
        for i in 0..count {
            let theta = (i as f64) * angle_step;
            poly.local_vertices[i] = Vec2::new(radius * theta.cos(), radius * theta.sin());
        }

        for i in 0..count {
            let v1 = poly.local_vertices[i];
            let v2 = poly.local_vertices[(i + 1) % count];
            let edge = v2.sub(&v1);

            let normal = edge.normal();
            let len = normal.dist_sq().sqrt();
            poly.local_normals[i] = normal.scale(1.0 / len);
        }

        poly
    }

    pub fn new_custom(vertices: &[Vec2]) -> Self {
        let count = vertices.len().clamp(3, MAX_POLY_VERTICES);
        let mut poly = Self {
            count,
            local_vertices: [Vec2::zero(); MAX_POLY_VERTICES],
            local_normals: [Vec2::zero(); MAX_POLY_VERTICES],
            world_vertices: [Vec2::zero(); MAX_POLY_VERTICES],
            world_normals: [Vec2::zero(); MAX_POLY_VERTICES],
        };

        for i in 0..count {
            poly.local_vertices[i] = vertices[i];
        }

        for i in 0..count {
            let v1 = poly.local_vertices[i];
            let v2 = poly.local_vertices[(i + 1) % count];
            let normal = v2.sub(&v1).normal();
            let len = normal.dist_sq().sqrt();
            poly.local_normals[i] = normal.scale(1.0 / len);
        }

        poly
    }
}
