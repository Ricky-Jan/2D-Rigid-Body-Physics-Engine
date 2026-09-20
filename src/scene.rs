use crate::{
    math::{PI, Vec2},
    rigidbody::RigidBody,
    shape::{Circle, Rect, ShapeType},
    world::World,
};
use macroquad::rand::{gen_range, rand};

pub struct TestScene;
impl TestScene {
    pub fn create_world() -> World {
        let mut world = World::new();

        Self::add_rect(&mut world, 0.0, -100.0, 3000.0, 200.0, 0.0, 0.0);
        Self::add_rect(&mut world, -1400.0, 1500.0, 200.0, 3000.0, 0.0, 0.0);
        Self::add_rect(&mut world, 1400.0, 1500.0, 200.0, 3000.0, 0.0, 0.0);

        for _ in 0..1000 {
            let x = gen_range(-1000.0, 1000.0);
            let y = gen_range(300.0, 4000.0);
            let angle = gen_range(0.0, PI * 2.0);
            let density = gen_range(0.5, 2.0);

            if rand() % 2 == 0 {
                let radius = gen_range(10.0, 25.0);
                Self::add_circle(&mut world, x, y, radius, angle, density);
            } else {
                let width = gen_range(20.0, 60.0);
                let height = gen_range(20.0, 60.0);
                Self::add_rect(&mut world, x, y, width, height, angle, density);
            }
        }

        world
    }

    fn add_circle(world: &mut World, x: f64, y: f64, radius: f64, angle: f64, density: f64) {
        let mut body = RigidBody::new(density);
        body.set_pos(Vec2::new(x, y));
        body.set_angle(angle);
        body.calc_circle_properties(radius);
        let body_index = world.add_body(body);
        let circle = Circle::new(radius);
        world.add_shape(ShapeType::Circle(circle), body_index);
    }

    fn add_rect(
        world: &mut World,
        x: f64,
        y: f64,
        width: f64,
        height: f64,
        angle: f64,
        density: f64,
    ) {
        let mut body = RigidBody::new(density);
        body.set_pos(Vec2::new(x, y));
        body.set_angle(angle);
        body.calc_rect_properties(width, height);
        let body_index = world.add_body(body);
        let rect = Rect::new(width, height);
        world.add_shape(ShapeType::Rect(rect), body_index);
    }
}
