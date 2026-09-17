use crate::{
    math::Vec2, rigidbody::RigidBody, settings::EPS, shape::Circle, world::PhysicsContext,
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

    pub fn circle_vs_circle(&mut self, circle_a: &Circle, circle_b: &Circle) -> bool {
        let delta = circle_b.pos.sub(&circle_a.pos);
        let dist_sq = delta.dist_sq();

        if dist_sq < EPS {
            return false;
        }

        let r_sum = circle_a.radius + circle_b.radius;
        if dist_sq < r_sum * r_sum {
            let dist = dist_sq.sqrt();
            let normal = delta.scale(1.0 / dist);
            let penetration = r_sum - dist;
            let pos = circle_a.pos.add(&normal.scale(circle_a.radius));

            self.add_contact(pos, normal, penetration);

            true
        } else {
            false
        }
    }
}
