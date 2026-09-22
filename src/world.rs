use crate::{
    aabb::AABB,
    bvh::{DynamicBVH, NULL_PTR, TreeNode},
    collision::collide,
    math::{Complex, EPS, TWO_PI, Vec2},
    pair::Pair,
    rigidbody::RigidBody,
    settings::*,
    shape::{Circle, Polygon, Rect, Shape, ShapeType},
};

pub struct World {
    pub context: PhysicsContext,
    pub shapes: Vec<Shape>,
    pub bodies: Vec<RigidBody>,
    pub shapes_free_list: Vec<usize>,
    pub shape_all_list: Vec<usize>,
    pub circle_all_list: Vec<usize>,
    pub rect_all_list: Vec<usize>,
    pub poly_all_list: Vec<usize>,
    pub bodies_free_list: Vec<usize>,
    pub bodies_active_list: Vec<usize>,
    pub bodies_awake_list: Vec<usize>,
    pub island_visited: Vec<bool>,
    pub island_stack: Vec<usize>,
    pub island_bodies_temp: Vec<usize>,
    pub island_adj: Vec<Vec<usize>>,
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
            shape_all_list: Vec::new(),
            circle_all_list: Vec::new(),
            rect_all_list: Vec::new(),
            poly_all_list: Vec::new(),
            bodies_free_list: Vec::new(),
            bodies_active_list: Vec::new(),
            bodies_awake_list: Vec::new(),
            island_visited: Vec::new(),
            island_stack: Vec::new(),
            island_bodies_temp: Vec::new(),
            island_adj: Vec::new(),
            pair: Pair::new(),
            pair_colliding_list: Vec::new(),
            bvh: DynamicBVH::new(),
        }
    }

    pub fn set_softness(&mut self, freq: f64, damping: f64) {
        let omega = TWO_PI * freq;
        let zeta = 2.0 * damping + omega * self.context.dt;
        self.context.impulse_damping = 1.0 + (1.0 / (omega * zeta) * self.context.dt);
        self.context.bias = omega / zeta;
    }

    pub fn create_body(&mut self, x: f64, y: f64, angle: f64, density: f64) -> usize {
        let mut body = RigidBody::new(density);
        body.set_pos(Vec2::new(x, y));
        body.set_angle(angle);

        let index;
        if let Some(free_idx) = self.bodies_free_list.pop() {
            index = free_idx;
            self.bodies[index] = body;
        } else {
            index = self.bodies.len();
            self.bodies.push(body);
        }
        index
    }

    pub fn add_shape(
        &mut self,
        body_ptr: usize,
        shape_type: ShapeType,
        local_pos: Vec2,
        local_angle: f64,
    ) -> usize {
        let shape = Shape::new(shape_type, body_ptr, local_pos, local_angle);

        let shape_idx;
        if let Some(free_idx) = self.shapes_free_list.pop() {
            shape_idx = free_idx;
            self.shapes[shape_idx] = shape;
        } else {
            shape_idx = self.shapes.len();
            self.shapes.push(shape);
        }

        let body = &mut self.bodies[body_ptr];
        self.shapes[shape_idx].next = body.shape_head;
        body.shape_head = shape_idx;

        self.shape_all_list.push(shape_idx);
        match shape_type {
            ShapeType::Circle(_) => self.circle_all_list.push(shape_idx),
            ShapeType::Rect(_) => self.rect_all_list.push(shape_idx),
            ShapeType::Polygon(_) => self.poly_all_list.push(shape_idx),
        }

        shape_idx
    }

    pub fn finalize_body(&mut self, body_ptr: usize) {
        let density = self.bodies[body_ptr].density;
        let mut total_mass = 0.0;
        let mut com_numerator = Vec2::zero();

        let mut curr = self.bodies[body_ptr].shape_head;
        while curr != NULL_PTR {
            let shape = &self.shapes[curr];
            let props = shape.get_properties(density);

            total_mass += props.mass;
            com_numerator = com_numerator.add(&props.local_centroid.scale(props.mass));

            curr = shape.next;
        }

        let loc_centroid = if total_mass > EPS {
            com_numerator.scale(1.0 / total_mass)
        } else {
            Vec2::zero()
        };

        let mut total_inertia = 0.0;
        let mut curr = self.bodies[body_ptr].shape_head;
        while curr != NULL_PTR {
            let shape = &self.shapes[curr];
            let props = shape.get_properties(density);

            let dist_sq = props.local_centroid.sub(&loc_centroid).dist_sq();
            total_inertia += props.inertia + props.mass * dist_sq;

            curr = shape.next;
        }

        let body = &mut self.bodies[body_ptr];
        body.loc_centroid = loc_centroid;
        body.is_regular = loc_centroid.dist_sq() < EPS;

        if total_mass > EPS {
            body.inv_m = 1.0 / total_mass;
            body.inv_i = 1.0 / total_inertia;
            if !self.bodies_active_list.contains(&body_ptr) {
                self.bodies_active_list.push(body_ptr);
                self.bodies_awake_list.push(body_ptr);
            }
        } else {
            body.inv_m = 0.0;
            body.inv_i = 0.0;
            body.is_awake = false;
        }

        body.centroid = body.pos.add(&body.heading.rotate(&body.loc_centroid));

        let mut curr = self.bodies[body_ptr].shape_head;
        while curr != NULL_PTR {
            let next = self.shapes[curr].next;
            let body_ref = &self.bodies[body_ptr];
            self.shapes[curr].refresh_transform(body_ref);
            curr = next;
        }

        let vel = self.bodies[body_ptr].vel;
        let inv_m = self.bodies[body_ptr].inv_m;
        let mut curr = self.bodies[body_ptr].shape_head;
        while curr != NULL_PTR {
            let next = self.shapes[curr].next;
            let aabb = self.shapes[curr].aabb;
            self.update_bvh(aabb, curr, &vel, inv_m);
            curr = next;
        }
    }

    pub fn create_circle(
        &mut self,
        x: f64,
        y: f64,
        radius: f64,
        angle: f64,
        density: f64,
    ) -> usize {
        let body_idx = self.create_body(x, y, angle, density);
        self.add_shape(
            body_idx,
            ShapeType::Circle(Circle::new(radius)),
            Vec2::zero(),
            0.0,
        );
        self.finalize_body(body_idx);
        body_idx
    }

    pub fn create_rect(
        &mut self,
        x: f64,
        y: f64,
        width: f64,
        height: f64,
        angle: f64,
        density: f64,
    ) -> usize {
        let body_idx = self.create_body(x, y, angle, density);
        self.add_shape(
            body_idx,
            ShapeType::Rect(Rect::new(width, height)),
            Vec2::zero(),
            0.0,
        );
        self.finalize_body(body_idx);
        body_idx
    }

    pub fn create_regular_polygon(
        &mut self,
        x: f64,
        y: f64,
        sides: usize,
        radius: f64,
        angle: f64,
        density: f64,
    ) -> usize {
        let body_idx = self.create_body(x, y, angle, density);
        self.add_shape(
            body_idx,
            ShapeType::Polygon(Polygon::new_regular(sides, radius)),
            Vec2::zero(),
            0.0,
        );
        self.finalize_body(body_idx);
        body_idx
    }

    pub fn create_custom_polygon(
        &mut self,
        x: f64,
        y: f64,
        vertices: &[Vec2],
        angle: f64,
        density: f64,
    ) -> usize {
        let body_idx = self.create_body(x, y, angle, density);
        self.add_shape(
            body_idx,
            ShapeType::Polygon(Polygon::new_custom(vertices)),
            Vec2::zero(),
            0.0,
        );
        self.finalize_body(body_idx);
        body_idx
    }

    pub fn step(&mut self) {
        self.sync_shapes();
        self.apply_forces();
        self.broad_phase();
        self.narrow_phase();
        self.resolve_collisions();
        self.integrate_position();
        self.update_islands();
    }

    fn sync_shapes(&mut self) {
        for &body_idx in &self.bodies_awake_list {
            let mut curr = self.bodies[body_idx].shape_head;
            while curr != NULL_PTR {
                let next = self.shapes[curr].next;
                let body = &self.bodies[body_idx];
                self.shapes[curr].refresh_transform(body);
                curr = next;
            }
        }
    }

    fn apply_forces(&mut self) {
        for &body_idx in &self.bodies_awake_list {
            let body = &mut self.bodies[body_idx];
            body.vel = body
                .vel
                .scale(self.context.linear_drag)
                .add(&self.context.gravity.scale(self.context.dt));
            body.ang_vel *= self.context.angular_drag;
        }
    }

    fn broad_phase(&mut self) {
        for i in 0..self.shape_all_list.len() {
            let shape_idx = self.shape_all_list[i];

            let body_ptr = self.shapes[shape_idx].body_ptr;
            let vel = self.bodies[body_ptr].vel;
            let inv_m = self.bodies[body_ptr].inv_m;

            let node_ptr = self.shapes[shape_idx].node_ptr;
            let aabb = self.shapes[shape_idx].aabb;
            let fat_aabb = self.shapes[shape_idx].fat_aabb;

            if !fat_aabb.contain(&aabb) {
                self.bvh.remove_leaf(node_ptr);
                self.update_bvh(aabb, shape_idx, &vel, inv_m);
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
        self.shapes[shape_a_idx].node_ptr = new_node_ptr;
        self.shapes[shape_a_idx].fat_aabb = new_fat_aabb;

        let candidate = self.bvh.query(&new_fat_aabb);
        for &shape_b_idx in &candidate {
            if shape_a_idx != shape_b_idx {
                let body_a_ptr = self.shapes[shape_a_idx].body_ptr;
                let body_b_ptr = self.shapes[shape_b_idx].body_ptr;

                if body_a_ptr != body_b_ptr {
                    if inv_m > 0.0 || self.bodies[body_b_ptr].inv_m > 0.0 {
                        self.pair
                            .get(shape_a_idx, shape_b_idx, body_a_ptr, body_b_ptr);
                    }
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
            let arbiter = &mut self.pair.pool[pool_idx].arbiter;

            let awake_a = self.bodies[arbiter.body_a_idx].is_awake;
            let awake_b = self.bodies[arbiter.body_b_idx].is_awake;

            if !awake_a && !awake_b {
                continue;
            }

            let shape_a_idx = arbiter.shape_a_idx;
            let shape_b_idx = arbiter.shape_b_idx;

            let shape_a = &self.shapes[shape_a_idx];
            let shape_b = &self.shapes[shape_b_idx];
            let mut is_colliding = false;

            if shape_a.aabb.intersect(&shape_b.aabb) {
                is_colliding = collide(&mut arbiter.manifold, shape_a, shape_b, &mut arbiter.cache);

                if is_colliding {
                    let body_a_idx = arbiter.body_a_idx;
                    let body_b_idx = arbiter.body_b_idx;

                    if awake_a && !awake_b && self.bodies[body_b_idx].inv_m > 0.0 {
                        self.bodies[body_b_idx].wake_up();
                    } else if awake_b && !awake_a && self.bodies[body_a_idx].inv_m > 0.0 {
                        self.bodies[body_a_idx].wake_up();
                    }

                    let body_a = &self.bodies[body_a_idx];
                    let body_b = &self.bodies[body_b_idx];
                    arbiter.init(&self.context, body_a, body_b);
                    self.pair_colliding_list.push(pool_idx);
                }
            }

            if !is_colliding {
                arbiter.manifold.point_count = 0;
                if !shape_a.fat_aabb.intersect(&shape_b.fat_aabb) {
                    self.pair.remove(pool_idx);
                }
            }
        }
    }

    fn update_islands(&mut self) {
        let len = self.bodies.len();

        if self.island_visited.len() < len {
            self.island_visited.resize(len, false);
            self.island_adj.resize(len, Vec::new());
        } else {
            self.island_visited.fill(false);
            for adj_list in &mut self.island_adj {
                adj_list.clear();
            }
        }
        self.island_stack.clear();
        self.island_bodies_temp.clear();

        for &pool_idx in &self.pair.active_pairs {
            let arbiter = &self.pair.pool[pool_idx].arbiter;
            if arbiter.manifold.point_count > 0 {
                let b_a = arbiter.body_a_idx;
                let b_b = arbiter.body_b_idx;
                if self.bodies[b_a].inv_m > 0.0 && self.bodies[b_b].inv_m > 0.0 {
                    self.island_adj[b_a].push(b_b);
                    self.island_adj[b_b].push(b_a);
                }
            }
        }

        self.bodies_awake_list.clear();

        let mut current_island_id = 1;

        for i in 0..self.bodies_active_list.len() {
            let start_idx = self.bodies_active_list[i];

            if self.island_visited[start_idx] || self.bodies[start_idx].inv_m == 0.0 {
                continue;
            }

            self.island_bodies_temp.clear();
            self.island_stack.push(start_idx);
            self.island_visited[start_idx] = true;

            let mut island_awake = false;

            while let Some(b) = self.island_stack.pop() {
                self.island_bodies_temp.push(b);
                let body = &self.bodies[b];

                if body.vel.dist_sq() > SLEEP_LINEAR_VEL_SQ
                    || body.ang_vel * body.ang_vel > SLEEP_ANGULAR_VEL_SQ
                {
                    island_awake = true;
                }

                let neighbors_count = self.island_adj[b].len();
                for j in 0..neighbors_count {
                    let neighbor = self.island_adj[b][j];
                    if !self.island_visited[neighbor] && self.bodies[neighbor].inv_m > 0.0 {
                        self.island_visited[neighbor] = true;
                        self.island_stack.push(neighbor);
                    }
                }
            }

            if island_awake {
                for &b in &self.island_bodies_temp {
                    self.bodies[b].wake_up();
                    self.bodies[b].island_id = 0;
                    self.bodies_awake_list.push(b);
                }
            } else {
                let mut all_can_sleep = true;
                for &b in &self.island_bodies_temp {
                    self.bodies[b].sleep_timer += self.context.dt;
                    if self.bodies[b].sleep_timer < SLEEP_TIME {
                        all_can_sleep = false;
                    }
                }

                if all_can_sleep {
                    for &b in &self.island_bodies_temp {
                        self.bodies[b].is_awake = false;
                        self.bodies[b].vel = Vec2::zero();
                        self.bodies[b].ang_vel = 0.0;
                        self.bodies[b].island_id = current_island_id;
                    }
                    current_island_id += 1;
                } else {
                    for &b in &self.island_bodies_temp {
                        self.bodies[b].island_id = 0;
                        self.bodies_awake_list.push(b);
                    }
                }
            }
        }
    }

    fn resolve_collisions(&mut self) {
        for &pool_idx in &self.pair_colliding_list {
            let arbiter = &self.pair.pool[pool_idx].arbiter;

            let idx_a = arbiter.body_a_idx;
            let idx_b = arbiter.body_b_idx;

            let (body_a, body_b) = if idx_a < idx_b {
                let (left, right) = self.bodies.split_at_mut(idx_b);
                (&mut left[idx_a], &mut right[0])
            } else {
                let (left, right) = self.bodies.split_at_mut(idx_a);
                (&mut right[0], &mut left[idx_b])
            };

            arbiter.warm_start(body_a, body_b);
        }

        for _ in 0..SOLVER_ITER {
            for &pool_idx in &self.pair_colliding_list {
                let arbiter = &mut self.pair.pool[pool_idx].arbiter;

                let idx_a = arbiter.body_a_idx;
                let idx_b = arbiter.body_b_idx;

                if !self.bodies[idx_a].is_awake && !self.bodies[idx_b].is_awake {
                    continue;
                }

                let (body_a, body_b) = if idx_a < idx_b {
                    let (left, right) = self.bodies.split_at_mut(idx_b);
                    (&mut left[idx_a], &mut right[0])
                } else {
                    let (left, right) = self.bodies.split_at_mut(idx_a);
                    (&mut right[0], &mut left[idx_b])
                };

                arbiter.resolve(body_a, body_b);
            }
        }
    }

    fn integrate_position(&mut self) {
        for &body_idx in &self.bodies_awake_list {
            let body = &mut self.bodies[body_idx];
            body.centroid = body.centroid.add(&body.vel.scale(self.context.dt));
            body.angle = (body.angle + body.ang_vel * self.context.dt) % TWO_PI;
            body.heading = Complex::new(body.angle.cos(), body.angle.sin());

            if body.is_regular {
                body.pos = body.centroid;
            } else {
                let rotated_loc = body.heading.rotate(&body.loc_centroid);
                body.pos = body.centroid.sub(&rotated_loc);
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
    pub fatness_predict_rate: f64,
    pub aabb_fatness: f64,
}
