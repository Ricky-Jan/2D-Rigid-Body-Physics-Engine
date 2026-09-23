use crate::{
    aabb::AABB,
    bvh::NULL_PTR,
    math::{Complex, EPS, NEG_INF, PI, POS_INF, TWO_PI, Vec2},
    rigidbody::RigidBody,
};

pub const MAX_POLY_VERTICES: usize = 8;

#[derive(Clone, Copy)]
pub struct Filter {
    pub category_bits: u16,
    pub mask_bits: u16,
    pub group_index: i16,
}

impl Filter {
    pub fn new() -> Self {
        Self {
            category_bits: 0x0001,
            mask_bits: 0xFFFF,
            group_index: 0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct BodyProperties {
    pub mass: f64,
    pub local_centroid: Vec2,
    pub inertia: f64,
}

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
    pub next: usize,

    pub local_pos: Vec2,
    pub local_angle: f64,

    pub pos: Vec2,
    pub angle: f64,
    pub heading: Complex,
    pub aabb: AABB,
    pub fat_aabb: AABB,

    pub filter: Filter,
}

impl Shape {
    pub fn new(shape_type: ShapeType, body_ptr: usize, local_pos: Vec2, local_angle: f64) -> Self {
        Self {
            shape_type,
            body_ptr,
            node_ptr: NULL_PTR,
            next: NULL_PTR,
            local_pos,
            local_angle,
            pos: Vec2::zero(),
            angle: 0.0,
            heading: Complex::new(1.0, 0.0),
            aabb: AABB::empty(),
            fat_aabb: AABB::empty(),
            filter: Filter::new(),
        }
    }

    pub fn get_properties(&self, density: f64) -> BodyProperties {
        if density <= EPS {
            return BodyProperties {
                mass: 0.0,
                local_centroid: self.local_pos,
                inertia: 0.0,
            };
        }

        let (mass, shape_local_centroid, inertia) = match &self.shape_type {
            ShapeType::Circle(c) => {
                let m = PI * c.radius * c.radius * density;
                let i = 0.5 * m * c.radius * c.radius;
                (m, Vec2::zero(), i)
            }
            ShapeType::Rect(r) => {
                let width = r.hw * 2.0;
                let height = r.hh * 2.0;
                let m = width * height * density;
                let i = m * (width * width + height * height) / 12.0;
                (m, Vec2::zero(), i)
            }
            ShapeType::Polygon(p) => {
                let mut area = 0.0;
                let mut cx = 0.0;
                let mut cy = 0.0;
                for i in 0..p.count {
                    let v1 = p.local_vertices[i];
                    let v2 = p.local_vertices[(i + 1) % p.count];
                    let cross = v1.x * v2.y - v2.x * v1.y;
                    area += cross;
                    cx += (v1.x + v2.x) * cross;
                    cy += (v1.y + v2.y) * cross;
                }
                area = area.abs() * 0.5;
                let m = area * density;

                let c_local = if area > EPS {
                    Vec2::new(cx / (6.0 * area), cy / (6.0 * area))
                } else {
                    Vec2::zero()
                };

                let mut i_val = 0.0;
                for i in 0..p.count {
                    let v1 = p.local_vertices[i].sub(&c_local);
                    let v2 = p.local_vertices[(i + 1) % p.count].sub(&c_local);
                    let term1 = v1.x * v1.x
                        + v1.x * v2.x
                        + v2.x * v2.x
                        + v1.y * v1.y
                        + v1.y * v2.y
                        + v2.y * v2.y;
                    let cross = (v1.x * v2.y - v2.x * v1.y).abs();
                    i_val += term1 * cross;
                }
                i_val = (i_val * density) / 12.0;

                (m, c_local, i_val)
            }
        };

        let shape_heading = Complex::new(self.local_angle.cos(), self.local_angle.sin());
        let rotated_centroid = shape_heading.rotate(&shape_local_centroid);
        let final_centroid = self.local_pos.add(&rotated_centroid);

        BodyProperties {
            mass,
            local_centroid: final_centroid,
            inertia,
        }
    }

    pub fn refresh_transform(&mut self, body: &RigidBody) {
        self.angle = body.angle + self.local_angle;
        self.heading = Complex::new(self.angle.cos(), self.angle.sin());

        if self.local_pos.x == 0.0 && self.local_pos.y == 0.0 {
            self.pos = body.pos;
        } else {
            self.pos = body.pos.add(&body.heading.rotate(&self.local_pos));
        }

        match &mut self.shape_type {
            ShapeType::Circle(circle) => {
                self.aabb = AABB::new(
                    Vec2::new(self.pos.x - circle.radius, self.pos.y - circle.radius),
                    Vec2::new(self.pos.x + circle.radius, self.pos.y + circle.radius),
                );
            }
            ShapeType::Rect(rect) => {
                let mut min_x = POS_INF;
                let mut min_y = POS_INF;
                let mut max_x = NEG_INF;
                let mut max_y = NEG_INF;

                for i in 0..4 {
                    let world_pos = self.pos.add(&self.heading.rotate(&rect.local_vertices[i]));
                    rect.world_vertices[i] = world_pos;
                    rect.world_normals[i] = self.heading.rotate(&rect.local_normals[i]);

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

    pub fn contains_point(&self, point: &Vec2) -> bool {
        match &self.shape_type {
            ShapeType::Circle(circle) => {
                let delta = point.sub(&self.pos);
                delta.dist_sq() <= circle.radius * circle.radius
            }
            ShapeType::Rect(rect) => {
                let delta = point.sub(&self.pos);
                let local_p = self.heading.inv_rotate(&delta);
                local_p.x.abs() <= rect.hw && local_p.y.abs() <= rect.hh
            }
            ShapeType::Polygon(poly) => {
                for i in 0..poly.count {
                    let v = poly.world_vertices[i];
                    let n = poly.world_normals[i];
                    if point.sub(&v).dot(&n) > 0.0 {
                        return false;
                    }
                }
                true
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
            let normal = v2.sub(&v1).normal();
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

        let mut area = 0.0;
        for i in 0..count {
            let v1 = poly.local_vertices[i];
            let v2 = poly.local_vertices[(i + 1) % count];
            area += v1.x * v2.y - v2.x * v1.y;
        }
        if area < 0.0 {
            poly.local_vertices[0..count].reverse();
        }

        for i in 0..count {
            let v1 = poly.local_vertices[i];
            let v2 = poly.local_vertices[(i + 1) % count];
            let normal = v2.sub(&v1).normal();
            let len = normal.dist_sq().sqrt();
            if len > 1e-6 {
                poly.local_normals[i] = normal.scale(1.0 / len);
            }
        }

        poly
    }
}
