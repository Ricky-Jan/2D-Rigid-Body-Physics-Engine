mod collision;
mod math;
mod renderer;
mod rigidbody;
mod scene;
mod settings;
mod shape;
mod world;

use crate::{math::Vec2, renderer::Camera, scene::TestScene, shape::Shape};
use macroquad::prelude::*;

#[macroquad::main("2D Rigid Body Physics Engine")]
async fn main() {
    let mut world = TestScene::create_world();
    let mut camera = Camera::new();
    let move_speed = 5.0;

    loop {
        if is_key_down(KeyCode::D) {
            camera.pos.x += move_speed;
        }

        if is_key_down(KeyCode::A) {
            camera.pos.x -= move_speed;
        }

        if is_key_down(KeyCode::W) {
            camera.pos.y += move_speed;
        }

        if is_key_down(KeyCode::S) {
            camera.pos.y -= move_speed;
        }

        if is_key_down(KeyCode::X) {
            camera.zoom *= 0.98;
        }

        if is_key_down(KeyCode::Z) {
            camera.zoom *= 1.02;
        }

        let (mouse_x, mouse_y) = mouse_position();
        let mouse_screen_pos = Vec2::new(mouse_x as f64, mouse_y as f64);
        let mouse_world_pos = camera.screen_to_world(&mouse_screen_pos);

        if let Some(Shape::Circle(circle_idx)) = world.shapes.first() {
            let body_idx = world.circle_all_list[*circle_idx].body_ptr;
            world.bodies[body_idx].set_pos(mouse_world_pos);
            world.bodies[body_idx].vel = Vec2::zero();
        }

        clear_background(LIGHTGRAY);
        world.step();
        renderer::render_world(&world, &camera);
        renderer::debug_renderer(&world, &camera);
        next_frame().await;
    }
}
