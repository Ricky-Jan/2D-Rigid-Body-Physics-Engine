mod aabb;
mod arbiter;
mod bvh;
mod collision;
mod joint;
mod math;
mod pair;
mod renderer;
mod rigidbody;
mod scene;
mod settings;
mod shape;
mod world;

use crate::{
    aabb::AABB,
    joint::MouseJoint,
    math::{PI, Vec2},
    renderer::{Camera, Renderer},
    scene::TestScene,
    shape::{Circle, Rect, ShapeType},
};
use macroquad::prelude::*;
use macroquad::rand::{gen_range, rand};

#[macroquad::main("2D Rigid-Body Physics Engine")]
async fn main() {
    let mut world = TestScene::create_world();
    let mut camera = Camera::new();
    let mut renderer = Renderer::new();

    camera.pos = Vec2::new(0.0, 1000.0);
    camera.zoom = 0.8;

    let move_speed = 5.0;
    let mut show_debug = false;

    loop {
        camera.update_screen_size();

        if is_key_down(KeyCode::D) {
            camera.pos.x += move_speed / camera.zoom;
        }
        if is_key_down(KeyCode::A) {
            camera.pos.x -= move_speed / camera.zoom;
        }
        if is_key_down(KeyCode::W) {
            camera.pos.y += move_speed / camera.zoom;
        }
        if is_key_down(KeyCode::S) {
            camera.pos.y -= move_speed / camera.zoom;
        }
        if is_key_down(KeyCode::X) {
            camera.zoom *= 0.99;
        }
        if is_key_down(KeyCode::Z) {
            camera.zoom *= 1.01;
        }

        if is_key_pressed(KeyCode::Space) {
            show_debug = !show_debug;
        }

        let (mx, my) = mouse_position();
        let mouse_world = camera.screen_to_world(&Vec2::new(mx as f64, my as f64));

        if is_key_pressed(KeyCode::Key0) {
            let target_aabb = AABB::new(
                Vec2::new(mouse_world.x - 0.1, mouse_world.y - 0.1),
                Vec2::new(mouse_world.x + 0.1, mouse_world.y + 0.1),
            );
            let candidates = world.bvh.query(&target_aabb);

            for shape_idx in candidates {
                let shape = &world.shapes[shape_idx];
                let body = &world.bodies[shape.body_ptr];

                if body.inv_m > 0.0 && shape.contains_point(&mouse_world) {
                    world.destroy_rigid(shape.body_ptr);
                    break;
                }
            }
        }

        if is_key_pressed(KeyCode::Key9) {
            let mut dynamic_bodies = Vec::new();
            for &b_idx in &world.bodies_active_list {
                if world.bodies[b_idx].inv_m > 0.0 {
                    dynamic_bodies.push(b_idx);
                }
            }

            let delete_count = 10.min(dynamic_bodies.len());
            for _ in 0..delete_count {
                let random_idx = (rand() as usize) % dynamic_bodies.len();
                let body_to_delete = dynamic_bodies.swap_remove(random_idx);
                world.destroy_rigid(body_to_delete);
            }
        }

        if is_mouse_button_pressed(MouseButton::Left) {
            let target_aabb = AABB::new(
                Vec2::new(mouse_world.x - 0.1, mouse_world.y - 0.1),
                Vec2::new(mouse_world.x + 0.1, mouse_world.y + 0.1),
            );
            let candidates = world.bvh.query(&target_aabb);

            for shape_idx in candidates {
                let shape = &world.shapes[shape_idx];
                let body = &world.bodies[shape.body_ptr];

                if body.inv_m == 0.0 {
                    continue;
                }

                if shape.contains_point(&mouse_world) {
                    let local_anchor = body.heading.inv_rotate(&mouse_world.sub(&body.centroid));
                    let mj = MouseJoint::new(
                        shape.body_ptr,
                        mouse_world,
                        local_anchor,
                        1000000.0,
                        1.0,
                        0.3,
                        world.context.dt,
                    );
                    world.mouse_joints.push(mj);
                    world.wake_up_body(shape.body_ptr);
                    break;
                }
            }
        } else if is_mouse_button_down(MouseButton::Left) {
            for i in 0..world.mouse_joints.len() {
                world.mouse_joints[i].target = mouse_world;
                let body_idx = world.mouse_joints[i].body_idx;
                world.wake_up_body(body_idx);
            }
        } else if is_mouse_button_released(MouseButton::Left) {
            if !world.mouse_joints.is_empty() {
                let last_idx = world.mouse_joints.len() - 1;
                world.destroy_mouse_joint(last_idx);
            }
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
            spawn_choice = Some(3);
        }
        if is_key_pressed(KeyCode::Key5) {
            spawn_choice = Some(4);
        }

        if let Some(choice) = spawn_choice {
            let angle = gen_range(0.0, PI * 2.0);
            let density = gen_range(0.5, 2.0);

            match choice {
                0 => {
                    let radius = gen_range(15.0, 30.0);
                    world.create_circle(mouse_world.x, mouse_world.y, radius, angle, density);
                }
                1 => {
                    let width = gen_range(30.0, 60.0);
                    let height = gen_range(30.0, 60.0);
                    world.create_rect(mouse_world.x, mouse_world.y, width, height, angle, density);
                }
                2 => {
                    let sides = gen_range(3, 8) as usize;
                    let radius = gen_range(20.0, 40.0);
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
                _ => {
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

        let hints = [
            "Controls:",
            "1,2,3,4,5 : Spawn Shapes",
            "W,A,S,D   : Move Camera",
            "Z,X       : Zoom In / Out",
            "0         : Delete Selected Body",
            "9         : Delete Random 10 Bodies",
            "Space     : Toggle Debug View",
        ];

        for (i, hint) in hints.iter().enumerate() {
            draw_text(hint, 20.0, 30.0 + (i as f32) * 25.0, 24.0, WHITE);
        }

        next_frame().await;
    }
}
