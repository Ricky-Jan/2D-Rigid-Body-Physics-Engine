use crate::{
    math::{PI, Vec2},
    world::World,
};
use macroquad::rand::{gen_range, rand};

pub struct TestScene;

impl TestScene {
    pub fn create_world() -> World {
        let mut world = World::new();

        world.create_rect(0.0, -100.0, 3000.0, 200.0, 0.0, 0.0);
        world.create_rect(-1400.0, 1500.0, 200.0, 3000.0, 0.0, 0.0);
        world.create_rect(1400.0, 1500.0, 200.0, 3000.0, 0.0, 0.0);

        for _ in 0..1000 {
            let x = gen_range(-1000.0, 1000.0);
            let y = gen_range(300.0, 4000.0);
            let angle = gen_range(0.0, PI * 2.0);
            let density = gen_range(0.5, 2.0);

            let shape_choice = rand() % 4;

            if shape_choice == 0 {
                let radius = gen_range(10.0, 25.0);
                world.create_circle(x, y, radius, angle, density);
            } else if shape_choice == 1 {
                let width = gen_range(20.0, 60.0);
                let height = gen_range(20.0, 60.0);
                world.create_rect(x, y, width, height, angle, density);
            } else if shape_choice == 2 {
                let sides = gen_range(3, 8) as usize;
                let radius = gen_range(15.0, 30.0);
                world.create_regular_polygon(x, y, sides, radius, angle, density);
            } else {
                let top_w = gen_range(10.0, 40.0);
                let bot_w = gen_range(30.0, 60.0);
                let h = gen_range(20.0, 50.0);

                let vertices = [
                    Vec2::new(0.0, h),
                    Vec2::new(top_w, h),
                    Vec2::new(bot_w, 0.0),
                    Vec2::new(-bot_w * 0.5, 0.0),
                ];
                world.create_custom_polygon(x, y, &vertices, angle, density);
            }
        }

        world
    }
}
