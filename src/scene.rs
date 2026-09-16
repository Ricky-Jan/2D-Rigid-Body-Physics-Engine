use crate::{
    math::{PI, Vec2},
    rigidbody::RigidBody,
    shape::Circle,
    world::World,
};
use macroquad::rand::gen_range;

pub struct TestScene;
impl TestScene {
    pub fn create_world() -> World {
        let mut world = World::new();

        Self::add_circle(&mut world, 400.0, -100.0, 300.0, 0.0, 0.0); // 大地板
        Self::add_circle(&mut world, 50.0, 100.0, 150.0, 0.0, 0.0); // 左牆
        Self::add_circle(&mut world, 750.0, 100.0, 150.0, 0.0, 0.0);

        for _ in 0..50 {
            let x = gen_range(100.0, 700.0);
            let y = gen_range(300.0, 800.0);
            let radius = gen_range(10.0, 25.0);
            let angle = gen_range(0.0, PI * 2.0);

            let density = if gen_range(0.0, 1.0) < 0.1 {
                0.0
            } else {
                gen_range(0.5, 2.0)
            };

            Self::add_circle(&mut world, x, y, radius, angle, density);
        }

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
