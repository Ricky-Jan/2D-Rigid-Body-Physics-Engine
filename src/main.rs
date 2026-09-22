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
    math::{PI, Vec2},
    renderer::{Camera, Renderer},
    scene::TestScene,
    shape::{Circle, Rect, ShapeType}, // 引入形狀庫以建立 Compound Body
};
use macroquad::prelude::*;
use macroquad::rand::{gen_range, rand};

#[macroquad::main("2D Rigid-Body Physics Engine")]
async fn main() {
    let mut world = TestScene::create_world();
    let mut camera = Camera::new();
    let mut renderer = Renderer::new();

    camera.pos = Vec2::new(-170.0, 100.0);

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

        let mut spawn_choice = None;

        if is_key_pressed(KeyCode::Key1) {
            spawn_choice = Some(0);
        }
        if is_key_pressed(KeyCode::Key2) {
            spawn_choice = Some(1);
        }
        if is_key_pressed(KeyCode::Key3) {
            spawn_choice = Some(2);
        }
        if is_key_pressed(KeyCode::Key4) {
            spawn_choice = Some(4);
        }
        if is_key_pressed(KeyCode::Key5) {
            spawn_choice = Some(rand() % 5);
        }

        if let Some(choice) = spawn_choice {
            let (mx, my) = mouse_position();
            let mouse_world = camera.screen_to_world(&Vec2::new(mx as f64, my as f64));

            let angle = gen_range(0.0, PI * 2.0);
            let density = gen_range(0.5, 2.0);

            match choice {
                0 => {
                    let radius = gen_range(10.0, 25.0);
                    world.create_circle(mouse_world.x, mouse_world.y, radius, angle, density);
                }
                1 => {
                    let width = gen_range(20.0, 60.0);
                    let height = gen_range(20.0, 60.0);
                    world.create_rect(mouse_world.x, mouse_world.y, width, height, angle, density);
                }
                2 => {
                    let sides = gen_range(3, 8) as usize;
                    let radius = gen_range(15.0, 30.0);
                    world.create_regular_polygon(
                        mouse_world.x,
                        mouse_world.y,
                        sides,
                        radius,
                        angle,
                        density,
                    );
                }
                3 => {
                    let top_w = gen_range(10.0, 40.0);
                    let bot_w = gen_range(30.0, 60.0);
                    let h = gen_range(20.0, 50.0);
                    let vertices = [
                        Vec2::new(0.0, h),
                        Vec2::new(top_w, h),
                        Vec2::new(bot_w, 0.0),
                        Vec2::new(-bot_w * 0.5, 0.0),
                    ];
                    world.create_custom_polygon(
                        mouse_world.x,
                        mouse_world.y,
                        &vertices,
                        angle,
                        density,
                    );
                }
                4 | _ => {
                    let car_body = world.create_body(mouse_world.x, mouse_world.y, angle, density);

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

        clear_background(Color::new(0.2118, 0.2706, 0.3098, 1.0));
        world.step();

        renderer.render(&world, &camera, show_debug);

        next_frame().await;
    }
}
