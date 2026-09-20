use crate::{
    math::{Complex, Vec2},
    rigidbody::RigidBody,
    settings::EPS,
    shape::{Shape, ShapeType},
    world::PhysicsContext,
};

#[derive(Clone, Copy)]
pub struct Contact {
    pub pos: Vec2,
    pub normal: Vec2,
    pub penetration: f64,
    pub rA: Vec2,
    pub rB: Vec2,
    pub Knn: f64,
    pub Ktt: f64,
    pub Knt: f64,
    pub damped_Knn: f64,
    pub det: f64,
    pub target_vel: f64,
    pub prev_jn: f64,
    pub prev_jt: f64,
}
impl Contact {
    pub fn new(pos: Vec2, normal: Vec2, penetration: f64) -> Self {
        Self {
            pos,
            normal,
            penetration,
            rA: Vec2::zero(),
            rB: Vec2::zero(),
            Knn: 0.0,
            Ktt: 0.0,
            Knt: 0.0,
            damped_Knn: 0.0,
            det: 0.0,
            target_vel: 0.0,
            prev_jn: 0.0,
            prev_jt: 0.0,
        }
    }

    pub fn empty() -> Self {
        Self::new(Vec2::zero(), Vec2::zero(), 0.0)
    }
}

#[derive(Clone, Copy)]
pub struct Collision {
    pub contact: Contact,
    pub body_a_idx: usize,
    pub body_b_idx: usize,
    pub restitution: f64,
    pub friction: f64,
}
impl Collision {
    pub fn new_pair(body_a_idx: usize, body_b_idx: usize) -> Self {
        Self {
            body_a_idx: body_a_idx,
            body_b_idx: body_b_idx,
            contact: Contact::empty(),
            friction: 0.0,
            restitution: 0.0,
        }
    }

    pub fn add_contact(&mut self, pos: Vec2, normal: Vec2, penetration: f64) {
        self.contact.pos = pos;
        self.contact.normal = normal;
        self.contact.penetration = penetration;
    }

    pub fn init_collision(
        &mut self,
        context: &PhysicsContext,
        body_a: &RigidBody,
        body_b: &RigidBody,
    ) {
        self.restitution = body_a.restitution.max(body_b.restitution);
        self.friction = (body_a.friction * body_b.friction).sqrt();
        Self::presolve_contact(context, &mut self.contact, body_a, body_b, self.restitution);
    }

    fn presolve_contact(
        context: &PhysicsContext,
        contact: &mut Contact,
        body_a: &RigidBody,
        body_b: &RigidBody,
        restitution: f64,
    ) {
        contact.rA = contact.pos.sub(&body_a.centroid);
        contact.rB = contact.pos.sub(&body_b.centroid);
        let rnA = contact.rA.cross(&contact.normal);
        let rnB = contact.rB.cross(&contact.normal);
        let rtA = contact.rA.dot(&contact.normal);
        let rtB = contact.rB.dot(&contact.normal);

        let wSum = body_a.inv_m + body_b.inv_m;
        contact.Knn = wSum + rnA * rnA * body_a.inv_i + rnB * rnB * body_b.inv_i;
        contact.Ktt = wSum + rtA * rtA * body_a.inv_i + rtB * rtB * body_b.inv_i;
        contact.Knt = rnA * rtA * body_a.inv_i + rnB * rtB * body_b.inv_i;

        contact.damped_Knn = contact.Knn * context.impulse_damping;
        contact.det = contact.damped_Knn * contact.Ktt - contact.Knt * contact.Knt;

        let vA = body_a.vel.add(&Vec2::cross_sv(body_a.ang_vel, &contact.rA));
        let vB = body_b.vel.add(&Vec2::cross_sv(body_b.ang_vel, &contact.rB));
        let rv = vB.sub(&vA);
        let rvn = rv.dot(&contact.normal);

        let effective_restitution = (-1.0 * rvn * restitution - context.min_elastic).max(0.0);
        let effective_penetration = ((contact.penetration - context.slop) * context.bias).max(0.0);
        contact.target_vel = effective_restitution + effective_penetration;
    }

    pub fn resolve_collision(&mut self, body_a: &mut RigidBody, body_b: &mut RigidBody) {
        Self::resolve_impulse(&mut self.contact, body_a, body_b, self.friction);
    }

    fn resolve_impulse(
        contact: &mut Contact,
        body_a: &mut RigidBody,
        body_b: &mut RigidBody,
        friction: f64,
    ) {
        let vA = body_a.vel.add(&Vec2::cross_sv(body_a.ang_vel, &contact.rA));
        let vB = body_b.vel.add(&Vec2::cross_sv(body_b.ang_vel, &contact.rB));
        let rv = vB.sub(&vA);
        let rvn = rv.dot(&contact.normal);

        let vel_n = contact.target_vel - rvn
            + contact.prev_jn * contact.Knn
            + contact.prev_jt * contact.Knt;

        if vel_n <= 0.0 {
            let impulse = contact
                .normal
                .scale(-contact.prev_jn)
                .add(&Vec2::cross_sv(-contact.prev_jt, &contact.normal));

            contact.prev_jn = 0.0;
            contact.prev_jt = 0.0;

            body_a.apply_impulse(&impulse.reverse(), &contact.rA);
            body_b.apply_impulse(&impulse, &contact.rB);
            return;
        }

        let rvt = rv.cross(&contact.normal);
        let vel_t = rvt + contact.prev_jn * contact.Knt + contact.prev_jt * contact.Ktt;

        let mut jt = (vel_t * contact.damped_Knn - vel_n * contact.Knt) / contact.det;
        let mut jn = (vel_n - jt * contact.Knt) / contact.damped_Knn;

        if jt.abs() > jn * friction {
            let signed_friction = vel_t.signum() * friction;
            jn = vel_n / (contact.damped_Knn + contact.Knt * signed_friction);
            jt = jn * signed_friction;
        }

        let delta_jn = jn - contact.prev_jn;
        let delta_jt = jt - contact.prev_jt;

        contact.prev_jn = jn;
        contact.prev_jt = jt;

        let impulse = contact
            .normal
            .scale(delta_jn)
            .add(&Vec2::cross_sv(delta_jt, &contact.normal));

        body_a.apply_impulse(&impulse.reverse(), &contact.rA);
        body_b.apply_impulse(&impulse, &contact.rB);
    }

    pub fn is_colliding(&mut self, shape_a: &Shape, shape_b: &Shape) -> bool {
        let a_id = shape_a.shape_type.get_id();
        let b_id = shape_b.shape_type.get_id();

        let swap = a_id > b_id;
        let (s1, s2) = if swap {
            (shape_b, shape_a)
        } else {
            (shape_a, shape_b)
        };

        let col = match (&s1.shape_type, &s2.shape_type) {
            (ShapeType::Circle(c1), ShapeType::Circle(c2)) => {
                self.circle_vs_circle(&s1.pos, c1.radius, &s2.pos, c2.radius)
            }

            (ShapeType::Circle(c), ShapeType::Rect(r)) => {
                self.circle_vs_rect(&s1.pos, c.radius, &s2.pos, &s2.heading, r.hw, r.hh)
            }

            (ShapeType::Rect(r1), ShapeType::Rect(r2)) => false,

            _ => unreachable!("Shape order enforcement failed!"),
        };

        if col && swap {
            self.contact.normal = self.contact.normal.reverse();
        }

        col
    }

    fn circle_vs_circle(
        &mut self,
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
            self.add_contact(pos, normal, r_sum);
            return true;
        }

        if dist_sq < r_sum * r_sum {
            let dist = dist_sq.sqrt();
            let normal = delta.scale(1.0 / dist);
            let penetration = r_sum - dist;
            let pos = pos_a.add(&normal.scale(radius_a));
            self.add_contact(pos, normal, penetration);
            return true;
        }

        false
    }

    fn circle_vs_rect(
        &mut self,
        c_pos: &Vec2,
        c_radius: f64,
        r_pos: &Vec2,
        r_heading: &Complex,
        r_hw: f64,
        r_hh: f64,
    ) -> bool {
        let delta = r_pos.sub(c_pos);
        let loc = r_heading.inv_rotate(&delta);
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

            if dw < dh {
                if loc.x > 0.0 {
                    let normal = r_heading.rotate(&Vec2::new(1.0, 0.0));
                    let pos = c_pos.sub(&normal.scale(c_radius));
                    self.add_contact(pos, normal, dw + c_radius);
                } else {
                    let normal = r_heading.rotate(&Vec2::new(-1.0, 0.0));
                    let pos = c_pos.sub(&normal.scale(c_radius));
                    self.add_contact(pos, normal, dw + c_radius);
                }
            } else {
                if loc.y > 0.0 {
                    let normal = r_heading.rotate(&Vec2::new(0.0, 1.0));
                    let pos = c_pos.sub(&normal.scale(c_radius));
                    self.add_contact(pos, normal, dh + c_radius);
                } else {
                    let normal = r_heading.rotate(&Vec2::new(0.0, -1.0));
                    let pos = c_pos.sub(&normal.scale(c_radius));
                    self.add_contact(pos, normal, dh + c_radius);
                }
            }
            return true;
        }

        let local_n = Vec2::new(clx / len, cly / len);
        let normal = r_heading.rotate(&local_n);
        let pos = c_pos.add(&normal.scale(c_radius));

        self.add_contact(pos, normal, c_radius - len);

        true
    }
}
