use crate::{
    collision::Collision,
    math::{TWO_PI, Vec2},
    rigidbody::RigidBody,
    settings::*,
    shape::{Circle, Shape},
};
use std::collections::HashMap;

pub struct World {
    pub context: PhysicsContext,
    pub shapes: Vec<Shape>,
    pub bodies: Vec<RigidBody>,
    pub shapes_free_list: Vec<usize>,
    pub bodies_free_list: Vec<usize>,
    pub bodies_active_list: Vec<usize>,
    pub circle_all_list: Vec<Circle>,
    pub collisions: Vec<Collision>,
    pub impulse_cache: HashMap<(usize, usize), (f64, f64)>,
}

impl World {
    pub fn new() -> Self {
        let dt = 1.0 / FPS as f64;
        let omega = TWO_PI * SOFTNESS_FREQUENCY;
        let zeta = 2.0 * SOFTNESS_DAMPING + omega * dt;
        let impulse_damping = 1.0 + (1.0 / (omega * zeta) * dt);
        let bias = omega / zeta;
        Self {
            context: PhysicsContext {
                gravity: Vec2::new(0.0, -98.0),
                dt,
                linear_drag: 0.997,
                angular_drag: 0.997,
                min_elastic: 10f64,
                impulse_damping,
                bias,
                slop: 0.01,
            },
            shapes: Vec::new(),
            bodies: Vec::new(),
            shapes_free_list: Vec::new(),
            bodies_free_list: Vec::new(),
            bodies_active_list: Vec::new(),
            circle_all_list: Vec::new(),
            collisions: Vec::new(),
            impulse_cache: HashMap::new(),
        }
    }

    fn set_softness(&mut self, freq: f64, damping: f64) {
        let omega = TWO_PI * freq;
        let zeta = 2.0 * damping + omega * self.context.dt;
        self.context.impulse_damping = 1.0 + (1.0 / (omega * zeta) * self.context.dt);
        self.context.bias = omega / zeta;
    }

    pub fn add_body(&mut self, body: RigidBody) -> usize {
        let is_active = body.inv_m > 0.0;
        let index;
        if self.bodies_free_list.is_empty() {
            index = self.bodies.len();
            self.bodies.push(body);
        } else {
            index = self.bodies_free_list.pop().unwrap();
            self.bodies[index] = body;
        }

        if is_active {
            self.bodies_active_list.push(index);
        }
        index
    }

    pub fn add_circle(&mut self, mut circle: Circle) -> usize {
        let circle_idx = self.circle_all_list.len();
        circle.index = circle_idx;
        self.circle_all_list.push(circle);

        let shape_idx;
        if self.shapes_free_list.is_empty() {
            shape_idx = self.shapes.len();
            self.shapes.push(Shape::Circle(circle_idx));
        } else {
            shape_idx = self.shapes_free_list.pop().unwrap();
            self.shapes[shape_idx] = Shape::Circle(circle_idx);
        }

        let body_ptr = circle.body_ptr;
        let body = &self.bodies[body_ptr];
        self.circle_all_list[circle_idx].refresh_transform(body);
        self.bodies[body_ptr].shape_ptr = shape_idx;

        shape_idx
    }

    pub fn step(&mut self) {
        self.apply_forces();
        self.collisions.clear();
        let len = self.shapes.len();

        for i in 0..len {
            let shape_a = self.shapes[i];
            for j in (i + 1)..len {
                let shape_b = self.shapes[j];
                let mut col = match (shape_a, shape_b) {
                    (Shape::Circle(idx_a), Shape::Circle(idx_b)) => {
                        let circle_a = self.circle_all_list[idx_a];
                        let circle_b = self.circle_all_list[idx_b];
                        Collision::circle_vs_circle(
                            &circle_a,
                            &circle_b,
                            circle_a.body_ptr,
                            circle_b.body_ptr,
                        )
                    }

                    _ => Collision::new_pair(0, 0),
                };

                if col.contact1.is_some() {
                    let body_a = &self.bodies[col.body_a_idx];
                    let body_b = &self.bodies[col.body_b_idx];

                    if body_a.inv_m > 0.0 || body_b.inv_m > 0.0 {
                        col.init_collision(&self.context, body_a, body_b);
                        self.collisions.push(col);
                    }
                }
            }
        }
        self.warm_start();
        self.resolve_collisions();
        self.integrate_position();
    }

    fn integrate_position(&mut self) {
        for &body_idx in &self.bodies_active_list {
            let body = &mut self.bodies[body_idx];
            let shape = self.shapes[body.shape_ptr];

            body.set_pos(body.pos.add(&body.vel.scale(self.context.dt)));
            body.set_angle(body.angle + body.ang_vel * self.context.dt);

            match shape {
                Shape::Circle(circle_idx) => {
                    self.circle_all_list[circle_idx].refresh_transform(body)
                }
                Shape::Disabled => {}
            }
        }
    }

    fn apply_forces(&mut self) {
        for &body_idx in &self.bodies_active_list {
            let body = &mut self.bodies[body_idx];
            body.vel = body
                .vel
                .scale(self.context.linear_drag)
                .add(&self.context.gravity.scale(self.context.dt));
            body.ang_vel *= self.context.angular_drag;
        }
    }

    fn warm_start(&mut self) {
        for col in &mut self.collisions {
            if let Some(contact) = &mut col.contact1 {
                if let Some(&(old_jn, old_jt)) =
                    self.impulse_cache.get(&(col.body_a_idx, col.body_b_idx))
                {
                    contact.prev_jn = old_jn;
                    contact.prev_jt = old_jt;

                    let impulse = contact
                        .normal
                        .scale(old_jn)
                        .add(&Vec2::cross_sv(old_jt, &contact.normal));

                    let (left, right) = self.bodies.split_at_mut(col.body_b_idx);
                    let body_a = &mut left[col.body_a_idx];
                    let body_b = &mut right[0];

                    body_a.apply_impulse(&impulse.reverse(), &contact.rA);
                    body_b.apply_impulse(&impulse, &contact.rB);
                }
            }
        }
    }

    fn resolve_collisions(&mut self) {
        for _ in 0..SOLVER_ITER {
            for col in &mut self.collisions {
                let idx_a = col.body_a_idx;
                let idx_b = col.body_b_idx;
                let (left, right) = self.bodies.split_at_mut(idx_b);
                let body_a = &mut left[idx_a];
                let body_b = &mut right[0];

                col.resolve_collision(body_a, body_b);
            }
        }

        self.impulse_cache.clear();
        for col in &self.collisions {
            if let Some(contact) = &col.contact1 {
                self.impulse_cache.insert(
                    (col.body_a_idx, col.body_b_idx),
                    (contact.prev_jn, contact.prev_jt),
                );
            }
        }
    }
}

pub struct PhysicsContext {
    pub gravity: Vec2,
    pub dt: f64,
    pub linear_drag: f64,
    pub angular_drag: f64,
    pub min_elastic: f64,
    pub bias: f64,
    pub impulse_damping: f64,
    pub slop: f64,
}
