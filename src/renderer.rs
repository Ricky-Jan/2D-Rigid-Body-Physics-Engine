use crate::{math::Vec2, world::World};
use macroquad::prelude::*;

pub struct Camera {
    pub pos: Vec2,
    pub zoom: f64,
    pub width: f64,
    pub height: f64,
    pub half_size: Vec2,
}

impl Camera {
    pub fn new() -> Self {
        let width = screen_width() as f64;
        let height = screen_height() as f64;

        Self {
            pos: Vec2::new(0.0, 0.0),
            zoom: 1.0,
            width: width,
            height: height,
            half_size: Vec2::new(width * 0.5, height * 0.5),
        }
    }

    pub fn transform_pos(&self, pos: &Vec2) -> Vec2 {
        let mut local_pos = pos.sub(&self.pos).scale(self.zoom);
        local_pos.y = -local_pos.y;
        local_pos.add(&self.half_size)
    }

    pub fn transform_len(&self, len: f64) -> f64 {
        len * self.zoom
    }

    pub fn screen_to_world(&self, screen_pos: &Vec2) -> Vec2 {
        let mut local_pos = screen_pos.sub(&self.half_size);
        local_pos.y = -local_pos.y;
        local_pos.scale(1.0 / self.zoom).add(&self.pos)
    }
}

pub fn render_world(world: &World, camera: &Camera) {
    for circle in &world.circle_all_list {
        let scr_pos = camera.transform_pos(&circle.pos);
        let scaled_radius = camera.transform_len(circle.radius);

        let x = scr_pos.x as f32;
        let y = scr_pos.y as f32;
        let r = scaled_radius as f32;
        let end_x = x + r * (circle.heading.re as f32);
        let end_y = y + r * (circle.heading.im as f32);
        draw_circle_lines(x, y, r, 1.0, BLACK);
        draw_line(x, y, end_x, end_y, 1.0, BLACK);
    }
}

pub fn debug_renderer(world: &World, camera: &Camera) {
    for col in &world.collisions {
        if let Some(contact) = &col.contact1 {
            let scr_pos = camera.transform_pos(&contact.pos);
            draw_circle(scr_pos.x as f32, scr_pos.y as f32, 5.0, YELLOW);

            let len = contact.penetration.max(5.0);
            let line_end = contact.pos.add(&contact.normal.scale(len));
            let scr_end = camera.transform_pos(&line_end);

            draw_line(
                scr_pos.x as f32,
                scr_pos.y as f32,
                scr_end.x as f32,
                scr_end.y as f32,
                3.0,
                GREEN,
            )
        }
    }
}
