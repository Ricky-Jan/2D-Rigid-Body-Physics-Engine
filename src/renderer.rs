use crate::{aabb::AABB, bvh::NULL_PTR, math::Vec2, world::World};
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
        let end_y = y - r * (circle.heading.im as f32);
        draw_circle_lines(x, y, r, 1.0, BLACK);
        draw_line(x, y, end_x, end_y, 1.0, BLACK);
    }
}

pub fn debug_renderer(world: &World, camera: &Camera) {
    draw_bvh_node(world, world.bvh.root, camera, 0);

    for circle in &world.circle_all_list {
        draw_aabb(&circle.get_aabb(), camera, BLUE, 1.5);
        draw_aabb(&circle.fat_aabb, camera, RED, 1.0);
    }

    for &pool_idx in &world.pair_colliding_list {
        let col = world.pair.pool[pool_idx].col;
        let contact = &col.contact;

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

    fn draw_aabb(aabb: &AABB, camera: &Camera, color: Color, thickness: f32) {
        let min_scr = camera.transform_pos(&aabb.min);
        let max_scr = camera.transform_pos(&aabb.max);

        let x = min_scr.x as f32;
        let y = max_scr.y as f32;
        let w = (max_scr.x - min_scr.x) as f32;
        let h = (min_scr.y - max_scr.y) as f32;

        draw_rectangle_lines(x, y, w, h, thickness, color);
    }

    fn draw_bvh_node(world: &World, node_idx: usize, camera: &Camera, depth: u32) {
        if node_idx == NULL_PTR {
            return;
        }

        let node = &world.bvh.nodes[node_idx];

        let r = 0.5 + (depth as f32 * 0.1).min(0.5);
        let color = Color::new(r, 0.5, 0.5, 0.15);

        draw_aabb(&node.aabb, camera, color, 1.0);

        if !node.is_leaf {
            draw_bvh_node(world, node.left, camera, depth + 1);
            draw_bvh_node(world, node.right, camera, depth + 1);
        }
    }
}
