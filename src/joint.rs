use crate::{
    math::{Mat22, TWO_PI, Vec2},
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
