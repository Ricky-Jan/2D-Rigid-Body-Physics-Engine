use crate::{
    math::{Complex, EPS, PI, TWO_PI, Vec2},
    settings::{DEFAULT_FRICTION, DEFAULT_RESTITUTION},
    shape::Polygon,
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
    pub shape_ptr: usize,
    pub restitution: f64,
    pub friction: f64,
    pub is_regular: bool,
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
            restitution: DEFAULT_RESTITUTION,
            friction: DEFAULT_FRICTION,
            is_regular: true,
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

    pub fn calc_circle_properties(&mut self, radius: f64) {
        self.is_regular = true;
        if self.density > 0.0 {
            let r_sq = radius * radius;
            let mass = PI * r_sq * self.density;
            self.inv_m = 1.0 / mass;
            self.inv_i = 1.0 / (0.5 * mass * r_sq);
        } else {
            self.inv_m = 0.0;
            self.inv_i = 0.0;
        }
        self.loc_centroid = Vec2::zero();
        self.centroid = self.pos;
    }

    pub fn calc_rect_properties(&mut self, width: f64, height: f64) {
        self.is_regular = true;
        if self.density > 0.0 {
            let mass = width * height * self.density;
            let inertia = mass * (width * width + height * height) / 12.0;
            self.inv_m = 1.0 / mass;
            self.inv_i = 1.0 / inertia;
        } else {
            self.inv_m = 0.0;
            self.inv_i = 0.0;
        }
        self.loc_centroid = Vec2::zero();
        self.centroid = self.pos;
    }

    pub fn calc_polygon_properties(&mut self, poly: &mut Polygon, density: f64, is_regular: bool) {
        self.density = density;
        self.is_regular = is_regular;

        let count = poly.count;
        if count < 3 {
            return;
        }

        let mut area = 0.0;
        for i in 0..count {
            let v1 = poly.local_vertices[i];
            let v2 = poly.local_vertices[(i + 1) % count];
            area += v1.x * v2.y - v2.x * v1.y;
        }
        area *= 0.5;

        if area < 0.0 {
            area = area.abs();
            poly.local_vertices[0..count].reverse();
            for i in 0..count {
                let v1 = poly.local_vertices[i];
                let v2 = poly.local_vertices[(i + 1) % count];
                let normal = v2.sub(&v1).normal();
                let len = normal.dist_sq().sqrt();
                if len > 1e-6 {
                    poly.local_normals[i] = normal.scale(1.0 / len);
                }
            }
        }

        if self.is_regular {
            self.loc_centroid = Vec2::zero();
            self.centroid = self.pos;
        } else {
            let mut cx = 0.0;
            let mut cy = 0.0;
            for i in 0..count {
                let v1 = poly.local_vertices[i];
                let v2 = poly.local_vertices[(i + 1) % count];
                let cross = v1.x * v2.y - v2.x * v1.y;
                cx += (v1.x + v2.x) * cross;
                cy += (v1.y + v2.y) * cross;
            }
            if area > EPS {
                cx /= 6.0 * area;
                cy /= 6.0 * area;
            }
            self.loc_centroid = Vec2::new(cx, cy);
            self.centroid = self.pos.add(&self.heading.rotate(&self.loc_centroid));
        }

        if density <= EPS {
            self.inv_m = 0.0;
            self.inv_i = 0.0;
            return;
        }

        let mass = area * density;
        self.inv_m = 1.0 / mass;

        let mut inertia = 0.0;
        for i in 0..count {
            let v1 = poly.local_vertices[i].sub(&self.loc_centroid);
            let v2 = poly.local_vertices[(i + 1) % count].sub(&self.loc_centroid);

            let term1 =
                v1.x * v1.x + v1.x * v2.x + v2.x * v2.x + v1.y * v1.y + v1.y * v2.y + v2.y * v2.y;
            let cross = v1.x * v2.y - v2.x * v1.y;
            inertia += term1 * cross;
        }

        inertia = (inertia * density) / 12.0;
        self.inv_i = 1.0 / inertia.abs();
    }

    pub fn apply_impulse(&mut self, impulse: &Vec2, arm: &Vec2) {
        self.vel = self.vel.add(&impulse.scale(self.inv_m));
        self.ang_vel += arm.cross(impulse) * self.inv_i;
    }
}
