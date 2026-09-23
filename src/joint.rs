use crate::{
    math::{EPS, Mat22, TWO_PI, Vec2},
    rigidbody::RigidBody,
};

#[derive(Clone, Copy)]
pub struct MouseJoint {
    pub body_idx: usize,
    pub target: Vec2,
    pub local_anchor: Vec2,
    pub max_impulse: f64,
    pub impulse: Vec2,
    r_b: Vec2,
    inv_k: Mat22,
    target_vel: Vec2,
    bias_coeff: f64,
    mass_coeff: f64,
    impulse_coeff: f64,
}

impl MouseJoint {
    pub fn new(
        body_idx: usize,
        target: Vec2,
        local_anchor: Vec2,
        max_force: f64,
        frequency: f64,
        damping: f64,
        dt: f64,
    ) -> Self {
        let mut bias_coeff = 0.0;
        let mut mass_coeff = 0.0;
        let mut impulse_coeff = 0.0;

        if frequency == 0.0 {
            bias_coeff = 0.2;
            mass_coeff = 0.0;
            impulse_coeff = 1.0;
        } else {
            let omega = TWO_PI * frequency;
            let a1 = 2.0 * damping + omega * dt;
            let a2 = omega * a1 * dt;
            let a3 = 1.0 / (1.0 + a2);
            bias_coeff = omega / a1;
            mass_coeff = a2 * a3;
            impulse_coeff = a3;
        }

        Self {
            body_idx,
            target,
            local_anchor,
            max_impulse: max_force * dt,
            impulse: Vec2::zero(),
            r_b: Vec2::zero(),
            inv_k: Mat22::zero(),
            target_vel: Vec2::zero(),
            bias_coeff,
            mass_coeff,
            impulse_coeff,
        }
    }

    pub fn init(&mut self, body: &RigidBody) {
        self.r_b = body.heading.rotate(&self.local_anchor);

        self.target_vel = body
            .centroid
            .add(&self.r_b)
            .sub(&self.target)
            .scale(self.bias_coeff);

        let k = Mat22::point_mass_matrix(body.inv_m, body.inv_i, &self.r_b);
        self.inv_k = k.invert();
    }

    pub fn warm_start(&self, body: &mut RigidBody) {
        body.apply_impulse(&self.impulse, &self.r_b);
    }

    pub fn solve(&mut self, body: &mut RigidBody) {
        let w_cross_r = Vec2::cross_sv(body.ang_vel, &self.r_b);

        let cdot = body.vel.add(&w_cross_r).add(&self.target_vel);

        let impulse_diff = self.inv_k.mul_v(&cdot).scale(self.impulse_coeff);
        let mut new_impulse = self.impulse.scale(self.mass_coeff).sub(&impulse_diff);
        new_impulse = new_impulse.clamp_mag(self.max_impulse);
        let delta_j = new_impulse.sub(&self.impulse);
        self.impulse = new_impulse;
        body.apply_impulse(&delta_j, &self.r_b);
    }
}

#[derive(Clone, Copy)]
pub struct DistanceJoint {
    pub body_a_idx: usize,
    pub body_b_idx: usize,
    pub local_anchor_a: Vec2,
    pub local_anchor_b: Vec2,
    pub rest_length: f64,

    // 解算器暫存變數
    pub impulse: f64, // 1D 約束，脈衝為純量
    r_a: Vec2,
    r_b: Vec2,
    normal: Vec2,
    inv_k: f64,
    target_vel: f64,
    bias_coeff: f64,
    mass_coeff: f64,
    impulse_coeff: f64,
}

impl DistanceJoint {
    pub fn new(
        body_a_idx: usize,
        body_b_idx: usize,
        local_anchor_a: Vec2,
        local_anchor_b: Vec2,
        rest_length: f64,
        frequency: f64,
        damping: f64,
        dt: f64,
    ) -> Self {
        let mut bias_coeff = 0.0;
        let mut mass_coeff = 0.0;
        let mut impulse_coeff = 0.0;

        if frequency == 0.0 {
            bias_coeff = 0.2;
            mass_coeff = 0.0;
            impulse_coeff = 1.0;
        } else {
            let omega = TWO_PI * frequency;
            let a1 = 2.0 * damping + omega * dt;
            let a2 = omega * a1 * dt;
            let a3 = 1.0 / (1.0 + a2);
            bias_coeff = omega / a1;
            mass_coeff = a2 * a3;
            impulse_coeff = a3;
        }

        Self {
            body_a_idx,
            body_b_idx,
            local_anchor_a,
            local_anchor_b,
            rest_length,
            impulse: 0.0,
            r_a: Vec2::zero(),
            r_b: Vec2::zero(),
            normal: Vec2::zero(),
            inv_k: 0.0,
            target_vel: 0.0,
            bias_coeff,
            mass_coeff,
            impulse_coeff,
        }
    }

    pub fn init(&mut self, body_a: &RigidBody, body_b: &RigidBody) {
        self.r_a = body_a.heading.rotate(&self.local_anchor_a);
        self.r_b = body_b.heading.rotate(&self.local_anchor_b);

        let anchor_a = body_a.centroid.add(&self.r_a);
        let anchor_b = body_b.centroid.add(&self.r_b);

        let delta = anchor_b.sub(&anchor_a);
        let len = delta.dist_sq().sqrt();

        if len < EPS {
            self.normal = Vec2::zero();
            self.inv_k = 0.0;
            self.target_vel = 0.0;
            return;
        }

        self.normal = delta.scale(1.0 / len);

        let rn_a = self.r_a.cross(&self.normal);
        let rn_b = self.r_b.cross(&self.normal);

        let k =
            body_a.inv_m + body_b.inv_m + rn_a * rn_a * body_a.inv_i + rn_b * rn_b * body_b.inv_i;

        self.inv_k = if k > 0.0 { 1.0 / k } else { 0.0 };
        self.target_vel = (len - self.rest_length) * self.bias_coeff;
    }

    pub fn warm_start(&mut self, body_a: &mut RigidBody, body_b: &mut RigidBody) {
        let p = self.normal.scale(self.impulse);
        body_a.apply_impulse(&p.reverse(), &self.r_a);
        body_b.apply_impulse(&p, &self.r_b);
    }

    pub fn solve(&mut self, body_a: &mut RigidBody, body_b: &mut RigidBody) {
        if self.inv_k == 0.0 {
            return;
        }

        let w_cross_r_a = Vec2::cross_sv(body_a.ang_vel, &self.r_a);
        let v_a = body_a.vel.add(&w_cross_r_a);

        let w_cross_r_b = Vec2::cross_sv(body_b.ang_vel, &self.r_b);
        let v_b = body_b.vel.add(&w_cross_r_b);

        let cdot = v_b.sub(&v_a).dot(&self.normal) + self.target_vel;

        let impulse_diff = cdot * self.inv_k * self.impulse_coeff;
        let new_impulse = self.impulse * self.mass_coeff - impulse_diff;
        let delta_impulse = new_impulse - self.impulse;

        self.impulse = new_impulse;

        let p = self.normal.scale(delta_impulse);
        body_a.apply_impulse(&p.reverse(), &self.r_a);
        body_b.apply_impulse(&p, &self.r_b);
    }
}
