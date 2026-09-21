use crate::{
    collision::{FeatureCache, Manifold},
    math::Vec2,
    rigidbody::RigidBody,
    world::PhysicsContext,
};

#[derive(Clone, Copy)]
pub struct SolverState {
    pub prev_jn: f64,
    pub prev_jt: f64,
    pub rA: Vec2,
    pub rB: Vec2,
    pub Knn: f64,
    pub Ktt: f64,
    pub Knt: f64,
    pub damped_Knn: f64,
    pub det: f64,
    pub target_vel: f64,
}

impl SolverState {
    pub fn empty() -> Self {
        Self {
            prev_jn: 0.0,
            prev_jt: 0.0,
            rA: Vec2::zero(),
            rB: Vec2::zero(),
            Knn: 0.0,
            Ktt: 0.0,
            Knt: 0.0,
            damped_Knn: 0.0,
            det: 0.0,
            target_vel: 0.0,
        }
    }
}

#[derive(Clone, Copy)]
pub struct Arbiter {
    pub body_a_idx: usize,
    pub body_b_idx: usize,
    pub manifold: Manifold,
    pub states: [SolverState; 2],
    pub friction: f64,
    pub restitution: f64,
    pub cache: FeatureCache,
}

impl Arbiter {
    pub fn new(body_a_idx: usize, body_b_idx: usize) -> Self {
        Self {
            body_a_idx,
            body_b_idx,
            manifold: Manifold::new(),
            states: [SolverState::empty(); 2],
            friction: 0.0,
            restitution: 0.0,
            cache: FeatureCache::new(),
        }
    }

    pub fn init(&mut self, context: &PhysicsContext, body_a: &RigidBody, body_b: &RigidBody) {
        self.restitution = body_a.restitution.max(body_b.restitution);
        self.friction = (body_a.friction * body_b.friction).sqrt();

        for i in 0..self.manifold.point_count {
            let point = &self.manifold.points[i];
            let state = &mut self.states[i];

            state.rA = point.pos.sub(&body_a.centroid);
            state.rB = point.pos.sub(&body_b.centroid);

            let normal = self.manifold.normal;
            let rnA = state.rA.cross(&normal);
            let rnB = state.rB.cross(&normal);
            let rtA = state.rA.dot(&normal);
            let rtB = state.rB.dot(&normal);

            let wSum = body_a.inv_m + body_b.inv_m;
            state.Knn = wSum + rnA * rnA * body_a.inv_i + rnB * rnB * body_b.inv_i;
            state.Ktt = wSum + rtA * rtA * body_a.inv_i + rtB * rtB * body_b.inv_i;
            state.Knt = rnA * rtA * body_a.inv_i + rnB * rtB * body_b.inv_i;

            state.damped_Knn = state.Knn * context.impulse_damping;
            state.det = state.damped_Knn * state.Ktt - state.Knt * state.Knt;

            let vA = body_a.vel.add(&Vec2::cross_sv(body_a.ang_vel, &state.rA));
            let vB = body_b.vel.add(&Vec2::cross_sv(body_b.ang_vel, &state.rB));
            let rv = vB.sub(&vA);
            let rvn = rv.dot(&normal);

            let effective_restitution =
                (-1.0 * rvn * self.restitution - context.min_elastic).max(0.0);
            let effective_penetration =
                ((point.penetration - context.slop) * context.bias).max(0.0);
            state.target_vel = effective_restitution + effective_penetration;
        }
    }

    pub fn warm_start(&self, body_a: &mut RigidBody, body_b: &mut RigidBody) {
        for i in 0..self.manifold.point_count {
            let state = &self.states[i];
            let impulse = self
                .manifold
                .normal
                .scale(state.prev_jn)
                .add(&Vec2::cross_sv(state.prev_jt, &self.manifold.normal));

            body_a.apply_impulse(&impulse.reverse(), &state.rA);
            body_b.apply_impulse(&impulse, &state.rB);
        }
    }

    pub fn resolve(&mut self, body_a: &mut RigidBody, body_b: &mut RigidBody) {
        for i in 0..self.manifold.point_count {
            let state = &mut self.states[i];
            let normal = self.manifold.normal;

            let vA = body_a.vel.add(&Vec2::cross_sv(body_a.ang_vel, &state.rA));
            let vB = body_b.vel.add(&Vec2::cross_sv(body_b.ang_vel, &state.rB));
            let rv = vB.sub(&vA);
            let rvn = rv.dot(&normal);

            let vel_n =
                state.target_vel - rvn + state.prev_jn * state.Knn + state.prev_jt * state.Knt;

            if vel_n <= 0.0 {
                let impulse = normal
                    .scale(-state.prev_jn)
                    .add(&Vec2::cross_sv(-state.prev_jt, &normal));
                state.prev_jn = 0.0;
                state.prev_jt = 0.0;
                body_a.apply_impulse(&impulse.reverse(), &state.rA);
                body_b.apply_impulse(&impulse, &state.rB);
                continue;
            }

            let rvt = rv.cross(&normal);
            let vel_t = rvt + state.prev_jn * state.Knt + state.prev_jt * state.Ktt;

            let mut jt = (vel_t * state.damped_Knn - vel_n * state.Knt) / state.det;
            let mut jn = (vel_n - jt * state.Knt) / state.damped_Knn;

            if jt.abs() > jn * self.friction {
                let signed_friction = vel_t.signum() * self.friction;
                jn = vel_n / (state.damped_Knn + state.Knt * signed_friction);
                jt = jn * signed_friction;
            }

            let delta_jn = jn - state.prev_jn;
            let delta_jt = jt - state.prev_jt;

            state.prev_jn = jn;
            state.prev_jt = jt;

            let impulse = normal
                .scale(delta_jn)
                .add(&Vec2::cross_sv(delta_jt, &normal));
            body_a.apply_impulse(&impulse.reverse(), &state.rA);
            body_b.apply_impulse(&impulse, &state.rB);
        }
    }
}
