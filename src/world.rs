use crate::{
    aabb::AABB,
    bvh::{DynamicBVH, NULL_PTR, TreeNode},
    math::{TWO_PI, Vec2},
    pair::Pair,
    rigidbody::RigidBody,
    settings::*,
    shape::{Circle, Shape},
};

pub struct World {
    pub context: PhysicsContext,
    pub shapes: Vec<Shape>,
    pub bodies: Vec<RigidBody>,
    pub shapes_free_list: Vec<usize>,
    pub bodies_free_list: Vec<usize>,
    pub bodies_active_list: Vec<usize>,
    pub circle_all_list: Vec<Circle>,
    pub pair: Pair,
    pub pair_colliding_list: Vec<usize>,
    pub bvh: DynamicBVH,
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
                gravity: Vec2::new(0.0, -50.0),
                dt,
                linear_drag: 0.997,
                angular_drag: 0.997,
                min_elastic: 10f64,
                impulse_damping,
                bias,
                slop: 0.05,
                fatness_predict_rate: 5.0,
                aabb_fatness: 10.0,
            },
            shapes: Vec::new(),
            bodies: Vec::new(),
            shapes_free_list: Vec::new(),
            bodies_free_list: Vec::new(),
            bodies_active_list: Vec::new(),
            circle_all_list: Vec::new(),
            pair: Pair::new(),
            pair_colliding_list: Vec::new(),
            bvh: DynamicBVH::new(),
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
        self.sync_shapes();
        self.apply_forces();
        self.broad_phase();
        self.narrow_phase();
        self.warm_start();
        self.resolve_collisions();
        self.integrate_position();
    }

    fn sync_shapes(&mut self) {
        for circle in &mut self.circle_all_list {
            let body = &self.bodies[circle.body_ptr];
            circle.refresh_transform(body);
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

    fn broad_phase(&mut self) {
        let len = self.circle_all_list.len();
        for i in 0..len {
            let body_ptr = self.circle_all_list[i].body_ptr;
            let vel = self.bodies[body_ptr].vel;
            let inv_m = self.bodies[body_ptr].inv_m;
            let node_ptr = self.circle_all_list[i].node_ptr;
            let aabb = self.circle_all_list[i].get_aabb();

            if node_ptr == NULL_PTR {
                self.update_bvh(aabb, i, &vel, inv_m);
            } else {
                let old_fat_aabb = self.circle_all_list[i].fat_aabb;
                if !old_fat_aabb.contain(&aabb) {
                    self.bvh.remove_leaf(node_ptr);
                    self.update_bvh(aabb, i, &vel, inv_m);
                }
            }
        }
    }

    fn update_bvh(&mut self, aabb: AABB, shape_a_idx: usize, vel: &Vec2, inv_m: f64) {
        let new_fat_aabb = aabb.fat_aabb(
            vel,
            self.context.fatness_predict_rate * self.context.dt,
            self.context.aabb_fatness,
        );
        let new_node_ptr = self
            .bvh
            .alloc_node(TreeNode::new_leaf(shape_a_idx, new_fat_aabb));
        self.bvh.insert_leaf(new_node_ptr);
        self.circle_all_list[shape_a_idx].node_ptr = new_node_ptr;
        self.circle_all_list[shape_a_idx].fat_aabb = new_fat_aabb;

        let candidate = self.bvh.query(&new_fat_aabb);
        for &shape_b_idx in &candidate {
            if shape_a_idx != shape_b_idx {
                let body_b_ptr = self.circle_all_list[shape_b_idx].body_ptr;
                if inv_m > 0.0 || self.bodies[body_b_ptr].inv_m > 0.0 {
                    let body_a_ptr = self.circle_all_list[shape_a_idx].body_ptr;
                    self.pair.get(body_a_ptr, body_b_ptr);
                }
            }
        }
    }

    fn narrow_phase(&mut self) {
        self.pair_colliding_list.clear();

        let mut i = self.pair.active_pairs.len();
        while i > 0 {
            i -= 1;
            let pool_idx = self.pair.active_pairs[i];
            let col = &mut self.pair.pool[pool_idx].col;

            let shape_a_idx = match self.shapes[self.bodies[col.body_a_idx].shape_ptr] {
                Shape::Circle(idx) => idx,
            };
            let shape_b_idx = match self.shapes[self.bodies[col.body_b_idx].shape_ptr] {
                Shape::Circle(idx) => idx,
            };

            let circle_a = &self.circle_all_list[shape_a_idx];
            let circle_b = &self.circle_all_list[shape_b_idx];

            if circle_a.get_aabb().intersect(&circle_b.get_aabb()) {
                if col.circle_vs_circle(circle_a, circle_b) {
                    let body_a = &self.bodies[col.body_a_idx];
                    let body_b = &self.bodies[col.body_b_idx];
                    col.init_collision(&self.context, body_a, body_b);
                    self.pair_colliding_list.push(pool_idx);
                }
            } else {
                col.contact.prev_jn = 0.0;
                col.contact.prev_jt = 0.0;
                if !circle_a.fat_aabb.intersect(&circle_b.fat_aabb) {
                    self.pair.remove(pool_idx);
                }
            }
        }
    }

    fn warm_start(&mut self) {
        for &pool_idx in &self.pair_colliding_list {
            let col = &mut self.pair.pool[pool_idx].col;
            let contact = &col.contact;
            let impulse = contact
                .normal
                .scale(contact.prev_jn)
                .add(&Vec2::cross_sv(contact.prev_jt, &contact.normal));

            let (left, right) = self.bodies.split_at_mut(col.body_b_idx);
            let body_a = &mut left[col.body_a_idx];
            let body_b = &mut right[0];

            body_a.apply_impulse(&impulse.reverse(), &contact.rA);
            body_b.apply_impulse(&impulse, &contact.rB);
        }
    }

    fn resolve_collisions(&mut self) {
        for _ in 0..SOLVER_ITER {
            for &pool_idx in &self.pair_colliding_list {
                let col = &mut self.pair.pool[pool_idx].col;

                let idx_a = col.body_a_idx;
                let idx_b = col.body_b_idx;
                let (left, right) = self.bodies.split_at_mut(idx_b);
                let body_a = &mut left[idx_a];
                let body_b = &mut right[0];

                col.resolve_collision(body_a, body_b);
            }
        }
    }

    fn integrate_position(&mut self) {
        for &body_idx in &self.bodies_active_list {
            let body = &mut self.bodies[body_idx];
            body.set_pos(body.pos.add(&body.vel.scale(self.context.dt)));
            body.set_angle(body.angle + body.ang_vel * self.context.dt);
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
    pub fatness_predict_rate: f64,
    pub aabb_fatness: f64,
}
