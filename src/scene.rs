use crate::{
    math::{PI, Vec2},
    shape::{Circle, Polygon, Rect, ShapeType},
    world::World,
};
use macroquad::rand::{gen_range, rand};

#[derive(Clone, Copy, PartialEq)]
pub enum SpawnType {
    Circle = 0,
    Rect = 1,
    RegularPoly = 2,
    CustomPoly = 3,
    Compound = 4,
    Random = 5,
}

pub struct TestScene;

impl TestScene {
    pub fn create_world() -> World {
        let mut world = World::new();

        world.create_rect(0.0, -100.0, 3000.0, 200.0, 0.0, 0.0);
        world.create_rect(-1400.0, 1500.0, 200.0, 3000.0, 0.0, 0.0);
        world.create_rect(1400.0, 1500.0, 200.0, 3000.0, 0.0, 0.0);

        let spawn_type = SpawnType::Random;
        let rows = 10;
        let cols = 15;
        let spacing_x = 100.0;
        let spacing_y = 100.0;

        let start_x = -((cols as f64 - 1.0) * spacing_x) / 2.0;
        let start_y = 100.0;

        for row in 0..rows {
            for col in 0..cols {
                let jitter_x = gen_range(-2.0, 2.0);
                let x = start_x + (col as f64) * spacing_x + jitter_x;
                let y = start_y + (row as f64) * spacing_y;

                let angle = gen_range(-0.05, 0.05);
                let density = gen_range(0.5, 2.0);

                let choice = if spawn_type == SpawnType::Random {
                    rand() % 5
                } else {
                    spawn_type as u32
                };

                match choice {
                    0 => {
                        let radius = gen_range(15.0, 25.0);
                        world.create_circle(x, y, radius, angle, density);
                    }
                    1 => {
                        let width = gen_range(30.0, 60.0);
                        let height = gen_range(30.0, 60.0);
                        world.create_rect(x, y, width, height, angle, density);
                    }
                    2 => {
                        let sides = gen_range(3, 8) as usize;
                        let radius = gen_range(20.0, 30.0);
                        world.create_regular_polygon(x, y, sides, radius, angle, density);
                    }
                    3 => {
                        let top_w = gen_range(15.0, 30.0);
                        let bot_w = gen_range(30.0, 50.0);
                        let h = gen_range(30.0, 50.0);

                        let vertices = [
                            Vec2::new(0.0, h),
                            Vec2::new(top_w, h),
                            Vec2::new(bot_w, 0.0),
                            Vec2::new(-bot_w * 0.5, 0.0),
                        ];
                        world.create_custom_polygon(x, y, &vertices, angle, density);
                    }
                    _ => {
                        let car_body = world.create_body(x, y, angle, density);

                        world.add_shape(
                            car_body,
                            ShapeType::Rect(Rect::new(80.0, 30.0)),
                            Vec2::zero(),
                            0.0,
                        );

                        world.add_shape(
                            car_body,
                            ShapeType::Circle(Circle::new(15.0)),
                            Vec2::new(-30.0, -15.0),
                            0.0,
                        );

                        world.add_shape(
                            car_body,
                            ShapeType::Circle(Circle::new(15.0)),
                            Vec2::new(30.0, -15.0),
                            0.0,
                        );

                        world.finalize_body(car_body);
                    }
                }
            }
        }

        world
    }
}
