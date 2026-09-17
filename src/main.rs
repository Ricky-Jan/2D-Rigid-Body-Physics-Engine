mod aabb;
mod bvh;
mod collision;
mod math;
mod pair;
mod renderer;
mod rigidbody;
mod scene;
mod settings;
mod shape;
mod world;

use crate::{renderer::Camera, scene::TestScene};
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

        clear_background(LIGHTGRAY);
        world.step();
        renderer::render_world(&world, &camera);
        // renderer::debug_renderer(&world, &camera);
        next_frame().await;
    }
}
