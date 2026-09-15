use crate::{
    collision::Collision,
    math::Vec2,
    rigidbody::RigidBody,
    settings::*,
    shape::{Circle, Shape},
};

pub struct World {
    pub shapes: Vec<Shape>,
    pub bodies: Vec<RigidBody>,
    pub shapes_free_list: Vec<usize>,
    pub bodies_free_list: Vec<usize>,
    pub bodies_active_list: Vec<usize>,
    pub circle_all_list: Vec<Circle>,
    pub collisions: Vec<Collision>,
}

impl World {
    pub fn new() -> Self {
        Self {
            shapes: Vec::new(),
            bodies: Vec::new(),
            shapes_free_list: Vec::new(),
            bodies_free_list: Vec::new(),
            bodies_active_list: Vec::new(),
            circle_all_list: Vec::new(),
            collisions: Vec::new(),
        }
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
                let col = Collision::collide_pair(shape_a, shape_b, self);
                if col.contact1.is_some() {
                    self.collisions.push(col);
                }
            }
        }

        self.integrate_position();
    }

    fn integrate_position(&mut self) {
        for &body_idx in &self.bodies_active_list {
            let body = &mut self.bodies[body_idx];
            let shape = self.shapes[body.shape_ptr];

            body.pos = body.pos.add(&body.vel.scale(DT));
            body.set_angle(body.angle + body.ang_vel * ANG_VEL_MULT);

            match shape {
                Shape::Circle(circle_idx) => {
                    self.circle_all_list[circle_idx].refresh_transform(body)
                }
                Shape::Disabled => {}
            }
        }
    }

    fn apply_forces(&mut self) {
        let delta_vel = Vec2::new(DELTA_VEL_X, DELTA_VEL_Y);
        for &body_idx in &self.bodies_active_list {
            let body = &mut self.bodies[body_idx];
            body.vel = body.vel.scale(LINEAR_DRAG).add(&delta_vel);
        }
    }
}
