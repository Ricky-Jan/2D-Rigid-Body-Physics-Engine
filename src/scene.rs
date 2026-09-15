use crate::{
    math::{PI, Vec2},
    rigidbody::RigidBody,
    shape::Circle,
    world::World,
};

pub struct TestScene;
impl TestScene {
    pub fn create_world() -> World {
        let mut world = World::new();
        Self::add_circle(&mut world, 200.0, 300.0, 30.0, 0.0, 1.0);
        Self::add_circle(&mut world, 400.0, 200.0, 50.0, PI, 0.0);
        Self::add_circle(&mut world, 600.0, 400.0, 40.0, 1.5 * PI, 1.0);
        world
    }

    fn add_circle(world: &mut World, x: f64, y: f64, radius: f64, angle: f64, density: f64) {
        let mut body = RigidBody::new(density);
        body.set_pos(Vec2::new(x, y));
        body.set_angle(angle);
        body.calc_circle_properties(radius);
        let body_index = world.add_body(body);
        let circle = Circle::new(radius, body_index);
        world.add_circle(circle);
    }
}
