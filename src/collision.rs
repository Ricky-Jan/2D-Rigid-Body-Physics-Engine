use crate::{
    math::{Complex, Vec2},
    settings::EPS,
    shape::{Shape, ShapeType},
};

#[derive(Clone, Copy)]
pub struct ManifoldPoint {
    pub pos: Vec2,
    pub penetration: f64,
}

#[derive(Clone, Copy)]
pub struct Manifold {
    pub points: [ManifoldPoint; 2],
    pub point_count: usize,
    pub normal: Vec2,
}

impl Manifold {
    pub fn new() -> Self {
        Self {
            points: [ManifoldPoint {
                pos: Vec2::zero(),
                penetration: 0.0,
            }; 2],
            point_count: 0,
            normal: Vec2::zero(),
        }
    }

    pub fn add_contact(&mut self, pos: Vec2, normal: Vec2, penetration: f64) {
        self.normal = normal;
        self.points[self.point_count].pos = pos;
        self.points[self.point_count].penetration = penetration;
        self.point_count += 1;
    }
}

pub fn collide(manifold: &mut Manifold, shape_a: &Shape, shape_b: &Shape) -> bool {
    manifold.point_count = 0;
    let a_id = shape_a.shape_type.get_id();
    let b_id = shape_b.shape_type.get_id();

    let swap = a_id > b_id;
    let (s1, s2) = if swap {
        (shape_b, shape_a)
    } else {
        (shape_a, shape_b)
    };

    let is_hit = match (&s1.shape_type, &s2.shape_type) {
        (ShapeType::Circle(c1), ShapeType::Circle(c2)) => {
            circle_vs_circle(manifold, &s1.pos, c1.radius, &s2.pos, c2.radius)
        }
        (ShapeType::Circle(c), ShapeType::Rect(r)) => circle_vs_rect(
            manifold,
            &s1.pos,
            c.radius,
            &s2.pos,
            &s2.heading,
            r.hw,
            r.hh,
        ),
        (ShapeType::Rect(r1), ShapeType::Rect(r2)) => false,

        _ => unreachable!("Shape order enforcement failed!"),
    };

    if is_hit && swap {
        manifold.normal = manifold.normal.reverse();
    }
    is_hit
}

fn circle_vs_circle(
    manifold: &mut Manifold,
    pos_a: &Vec2,
    radius_a: f64,
    pos_b: &Vec2,
    radius_b: f64,
) -> bool {
    let delta = pos_b.sub(pos_a);
    let dist_sq = delta.dist_sq();
    let r_sum = radius_a + radius_b;

    if dist_sq < EPS {
        let normal = Vec2::new(1.0, 0.0);
        let pos = Vec2::new(pos_a.x + radius_a, pos_a.y);
        manifold.add_contact(pos, normal, r_sum);
        return true;
    }

    if dist_sq < r_sum * r_sum {
        let dist = dist_sq.sqrt();
        let normal = delta.scale(1.0 / dist);
        let penetration = r_sum - dist;
        let pos = pos_a.add(&normal.scale(radius_a));
        manifold.add_contact(pos, normal, penetration);
        return true;
    }
    false
}

fn circle_vs_rect(
    manifold: &mut Manifold,
    c_pos: &Vec2,
    c_radius: f64,
    r_pos: &Vec2,
    r_heading: &Complex,
    r_hw: f64,
    r_hh: f64,
) -> bool {
    let d = c_pos.sub(r_pos);
    let loc = r_heading.inv_rotate(&d);

    let cls_x = loc.x.clamp(-r_hw, r_hw);
    let cls_y = loc.y.clamp(-r_hh, r_hh);

    let clx = loc.x - cls_x;
    let cly = loc.y - cls_y;
    let len_sq = clx * clx + cly * cly;

    if len_sq > c_radius * c_radius {
        return false;
    }

    let len = len_sq.sqrt();

    if len < EPS {
        let dw = r_hw - loc.x.abs();
        let dh = r_hh - loc.y.abs();
        let normal: Vec2;
        let penetration: f64;

        if dw < dh {
            penetration = dw + c_radius;
            normal = if loc.x > 0.0 {
                r_heading.rotate(&Vec2::new(-1.0, 0.0))
            } else {
                r_heading.rotate(&Vec2::new(1.0, 0.0))
            };
        } else {
            penetration = dh + c_radius;
            normal = if loc.y > 0.0 {
                r_heading.rotate(&Vec2::new(0.0, -1.0))
            } else {
                r_heading.rotate(&Vec2::new(0.0, 1.0))
            };
        }
        let pos = c_pos.add(&normal.scale(c_radius));
        manifold.add_contact(pos, normal, penetration);
        return true;
    }

    let local_n = Vec2::new(-clx / len, -cly / len);
    let normal = r_heading.rotate(&local_n);
    let pos = c_pos.add(&normal.scale(c_radius));
    manifold.add_contact(pos, normal, c_radius - len);
    true
}
