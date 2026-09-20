mod aabb;
mod arbiter;
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

use crate::{
    renderer::{Camera, Renderer},
    scene::TestScene,
};
use macroquad::prelude::*;

#[macroquad::main("2D Rigid Body Physics Engine")]
async fn main() {
    let mut world = TestScene::create_world();
    let mut camera = Camera::new();
    let mut renderer = Renderer::new();

    let move_speed = 5.0;

    let mut show_debug = true;

    loop {
        camera.update_screen_size();

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

        if is_key_pressed(KeyCode::Space) {
            show_debug = !show_debug;
        }

        clear_background(Color::new(0.2118, 0.2706, 0.3098, 1.0));
        world.step();

        renderer.render(&world, &camera, show_debug);

        next_frame().await;
    }
}
