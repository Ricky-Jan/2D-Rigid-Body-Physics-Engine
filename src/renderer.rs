use crate::{
    aabb::AABB,
    bvh::NULL_PTR,
    math::Vec2,
    shape::{MAX_POLY_VERTICES, ShapeType},
    world::World,
};
use macroquad::color::hsl_to_rgb;
use macroquad::prelude::*;

pub struct Camera {
    pub pos: Vec2,
    pub zoom: f64,
    pub half_size: Vec2,
}

impl Camera {
    pub fn new() -> Self {
        let width = screen_width() as f64;
        let height = screen_height() as f64;

        Self {
            pos: Vec2::new(0.0, 0.0),
            zoom: 1.0,
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

    pub fn get_aabb(&self) -> AABB {
        let hw = self.half_size.x / self.zoom;
        let hh = self.half_size.y / self.zoom;
        AABB::new(
            Vec2::new(self.pos.x - hw, self.pos.y - hh),
            Vec2::new(self.pos.x + hw, self.pos.y + hh),
        )
    }

    pub fn update_screen_size(&mut self) {
        let width = screen_width() as f64;
        let height = screen_height() as f64;
        self.half_size = Vec2::new(width * 0.5, height * 0.5);
    }
}

pub struct Renderer {
    visible_mask: Vec<bool>,
}

impl Renderer {
    pub fn new() -> Self {
        Self {
            visible_mask: Vec::with_capacity(1024),
        }
    }

    pub fn render(&mut self, world: &World, camera: &Camera, show_debug: bool) {
        self.frustum_culling(world, camera);
        self.render_world(world, camera);

        if show_debug {
            self.debug_renderer(world, camera);
        }
    }

    fn frustum_culling(&mut self, world: &World, camera: &Camera) {
        let cam_aabb = camera.get_aabb();

        self.visible_mask.clear();
        self.visible_mask.resize(world.shapes.len(), false);

        for &shape_idx in &world.shape_all_list {
            if world.shapes[shape_idx].aabb.intersect(&cam_aabb) {
                self.visible_mask[shape_idx] = true;
            }
        }
    }

    fn render_world(&self, world: &World, camera: &Camera) {
        let thickness = 1.0;
        let outline_color = Color::new(1.0, 1.0, 1.0, 1.0);

        self.draw_circle_bodies(world, camera, thickness, outline_color);
        self.draw_rect_bodies(world, camera, thickness, outline_color);
        self.draw_poly_bodies(world, camera, thickness, outline_color);
    }

    fn get_body_color(&self, world: &World, body_ptr: usize) -> Color {
        let body = &world.bodies[body_ptr];
        if body.inv_m == 0.0 {
            return Color::new(0.3, 0.3, 0.3, 1.0);
        } else if body.is_awake || body.island_id == 0 {
            return Color::new(0.0, 0.0, 0.0, 0.0);
        } else {
            let hue = (body.island_id as f32 * 0.3819) % 1.0;
            return hsl_to_rgb(hue, 0.7, 0.6);
        }
    }

    fn draw_circle_bodies(
        &self,
        world: &World,
        camera: &Camera,
        thickness: f32,
        outline_color: Color,
    ) {
        for &shape_idx in &world.circle_all_list {
            if !self.visible_mask[shape_idx] {
                continue;
            }

            let shape = &world.shapes[shape_idx];
            let color = self.get_body_color(world, shape.body_ptr);

            if let ShapeType::Circle(circle) = shape.shape_type {
                let scr_pos = camera.transform_pos(&shape.pos);
                let scaled_radius = camera.transform_len(circle.radius);

                let x = scr_pos.x as f32;
                let y = scr_pos.y as f32;
                let r = scaled_radius as f32;
                let end_x = x + r * (shape.heading.re as f32);
                let end_y = y - r * (shape.heading.im as f32);

                draw_circle(x, y, r, color);
                draw_circle_lines(x, y, r, thickness, outline_color);
                draw_line(x, y, end_x, end_y, thickness, outline_color);
            }
        }
    }

    fn draw_rect_bodies(
        &self,
        world: &World,
        camera: &Camera,
        thickness: f32,
        outline_color: Color,
    ) {
        for &shape_idx in &world.rect_all_list {
            if !self.visible_mask[shape_idx] {
                continue;
            }

            let shape = &world.shapes[shape_idx];
            let color = self.get_body_color(world, shape.body_ptr);

            if let ShapeType::Rect(rect) = shape.shape_type {
                let mut scr_v = [Vec2::zero(); 4];
                let mut points = [Vec2::zero(); 4];

                for i in 0..4 {
                    scr_v[i] = camera.transform_pos(&rect.world_vertices[i]);
                    points[i] = scr_v[i];
                }

                draw_triangle(
                    vec2(scr_v[0].x as f32, scr_v[0].y as f32),
                    vec2(scr_v[1].x as f32, scr_v[1].y as f32),
                    vec2(scr_v[2].x as f32, scr_v[2].y as f32),
                    color,
                );
                draw_triangle(
                    vec2(scr_v[0].x as f32, scr_v[0].y as f32),
                    vec2(scr_v[2].x as f32, scr_v[2].y as f32),
                    vec2(scr_v[3].x as f32, scr_v[3].y as f32),
                    color,
                );

                for i in 0..4 {
                    let next_i = (i + 1) % 4;
                    draw_line(
                        points[i].x as f32,
                        points[i].y as f32,
                        points[next_i].x as f32,
                        points[next_i].y as f32,
                        thickness,
                        outline_color,
                    );
                }
            }
        }
    }

    fn draw_poly_bodies(
        &self,
        world: &World,
        camera: &Camera,
        thickness: f32,
        outline_color: Color,
    ) {
        for &shape_idx in &world.poly_all_list {
            if !self.visible_mask[shape_idx] {
                continue;
            }

            let shape = &world.shapes[shape_idx];
            let color = self.get_body_color(world, shape.body_ptr);

            if let ShapeType::Polygon(poly) = shape.shape_type {
                let mut scr_v = [Vec2::zero(); MAX_POLY_VERTICES];
                for i in 0..poly.count {
                    scr_v[i] = camera.transform_pos(&poly.world_vertices[i]);
                }

                let v0 = vec2(scr_v[0].x as f32, scr_v[0].y as f32);
                for i in 1..poly.count - 1 {
                    let v1 = vec2(scr_v[i].x as f32, scr_v[i].y as f32);
                    let v2 = vec2(scr_v[i + 1].x as f32, scr_v[i + 1].y as f32);
                    draw_triangle(v0, v1, v2, color);
                }

                for i in 0..poly.count {
                    let next_i = (i + 1) % poly.count;
                    draw_line(
                        scr_v[i].x as f32,
                        scr_v[i].y as f32,
                        scr_v[next_i].x as f32,
                        scr_v[next_i].y as f32,
                        thickness,
                        outline_color,
                    );
                }
            }
        }
    }

    fn debug_renderer(&self, world: &World, camera: &Camera) {
        // self.draw_islands(world, camera, Color::new(1.0, 1.0, 1.0, 1.0));
        // self.draw_centroids(world, camera, Color::new(1.0, 1.0, 1.0, 1.0), 2.0);
        // self.draw_shape_aabbs(world, camera, Color::new(0.0, 1.0, 0.0, 1.0), 1.0);
        // self.draw_bvh_aabbs(world, camera, Color::new(1.0, 1.0, 1.0, 0.2), 1.0);
        // self.draw_penetrations(world, camera, Color::new(1.0, 1.0, 0.0, 1.0), 1.0, 4.0);
        // self.draw_contact_points(world, camera, Color::new(0.0, 1.0, 1.0, 1.0), 2.0);

        // self.draw_bvh_branches(
        //     world,
        //     100.0,
        //     100.0,
        //     Color::new(1.0, 1.0, 1.0, 1.0),
        //     Color::new(1.0, 0.0, 0.0, 1.0),
        // );
    }

    fn draw_aabb(&self, aabb: &AABB, camera: &Camera, color: Color, thickness: f32) {
        let min_scr = camera.transform_pos(&aabb.min);
        let max_scr = camera.transform_pos(&aabb.max);

        let x = min_scr.x as f32;
        let y = max_scr.y as f32;
        let w = (max_scr.x - min_scr.x) as f32;
        let h = (min_scr.y - max_scr.y) as f32;

        draw_rectangle_lines(x, y, w, h, thickness, color);
    }

    fn draw_shape_aabbs(&self, world: &World, camera: &Camera, color: Color, thickness: f32) {
        for &shape_idx in &world.shape_all_list {
            if self.visible_mask[shape_idx] {
                self.draw_aabb(&world.shapes[shape_idx].aabb, camera, color, thickness);
            }
        }
    }

    fn draw_bvh_aabbs(&self, world: &World, camera: &Camera, color: Color, thickness: f32) {
        if world.bvh.root == NULL_PTR {
            return;
        }

        let cam_aabb = camera.get_aabb();
        let mut stack = Vec::with_capacity(64);
        stack.push(world.bvh.root);

        while let Some(node_idx) = stack.pop() {
            let node = &world.bvh.nodes[node_idx];

            if node.aabb.intersect(&cam_aabb) {
                self.draw_aabb(&node.aabb, camera, color, thickness);

                let left_idx = node.left;
                if left_idx != NULL_PTR {
                    stack.push(left_idx);
                }

                let right_idx = node.right;
                if right_idx != NULL_PTR {
                    stack.push(right_idx);
                }
            }
        }
    }

    fn draw_bvh_branches(
        &self,
        world: &World,
        x: f64,
        y: f64,
        branch_color: Color,
        leaf_color: Color,
    ) {
        if world.bvh.root == NULL_PTR {
            return;
        }

        let spacing_x = 40.0;
        let spacing_y = 30.0;
        let decrease_x = 0.5;
        let decrease_y = 1.2 as f32;

        let mut stack = Vec::with_capacity(128);
        stack.push((world.bvh.root, x, y, spacing_x, 0));

        while let Some((node_idx, x, y, spacing_x, layer)) = stack.pop() {
            let node = &world.bvh.nodes[node_idx];
            let offset_y = spacing_y / decrease_y.powi(layer) as f64;

            if node.is_leaf {
                draw_circle(x as f32, y as f32, 2.0, leaf_color);
            } else {
                let right_idx = node.right;
                if right_idx != NULL_PTR {
                    let next_x = x + spacing_x;
                    let next_y = y + offset_y;
                    draw_line(
                        x as f32,
                        y as f32,
                        next_x as f32,
                        next_y as f32,
                        1.0,
                        branch_color,
                    );
                    stack.push((right_idx, next_x, next_y, spacing_x * decrease_x, layer + 1));
                }

                let left_idx = node.left;
                if left_idx != NULL_PTR {
                    let next_x = x - spacing_x;
                    let next_y = y + offset_y;
                    draw_line(
                        x as f32,
                        y as f32,
                        next_x as f32,
                        next_y as f32,
                        1.0,
                        branch_color,
                    );
                    stack.push((left_idx, next_x, next_y, spacing_x * decrease_x, layer + 1));
                }
            }
        }
    }

    fn draw_contact_points(&self, world: &World, camera: &Camera, color: Color, size: f32) {
        for &pool_idx in &world.pair_colliding_list {
            let arbiter = &world.pair.pool[pool_idx].arbiter;
            let shape_a_idx = arbiter.shape_a_idx;
            let shape_b_idx = arbiter.shape_b_idx;

            if self.visible_mask[shape_a_idx] || self.visible_mask[shape_b_idx] {
                for i in 0..arbiter.manifold.point_count {
                    let contact_pos = arbiter.manifold.points[i].pos;
                    let scr_pos = camera.transform_pos(&contact_pos);
                    draw_circle(scr_pos.x as f32, scr_pos.y as f32, size, color);
                }
            }
        }
    }

    fn draw_penetrations(
        &self,
        world: &World,
        camera: &Camera,
        color: Color,
        thickness: f32,
        min_length: f64,
    ) {
        for &pool_idx in &world.pair_colliding_list {
            let arbiter = &world.pair.pool[pool_idx].arbiter;

            let shape_a_idx = arbiter.shape_a_idx;
            let shape_b_idx = arbiter.shape_b_idx;

            if self.visible_mask[shape_a_idx] || self.visible_mask[shape_b_idx] {
                for i in 0..arbiter.manifold.point_count {
                    let contact = arbiter.manifold.points[i];
                    let contact_pos = contact.pos;
                    let scr_start = camera.transform_pos(&contact_pos);
                    let len = contact.penetration.max(min_length);
                    let line_end = contact.pos.add(&arbiter.manifold.normal.scale(len));
                    let scr_end = camera.transform_pos(&line_end);
                    draw_line(
                        scr_start.x as f32,
                        scr_start.y as f32,
                        scr_end.x as f32,
                        scr_end.y as f32,
                        thickness,
                        color,
                    );
                }
            }
        }
    }

    fn draw_centroids(&self, world: &World, camera: &Camera, color: Color, size: f32) {
        for &body_idx in &world.bodies_active_list {
            let body = &world.bodies[body_idx];
            let scr_pos = camera.transform_pos(&body.centroid);
            draw_circle(scr_pos.x as f32, scr_pos.y as f32, size, color);
        }
    }

    fn draw_islands(&self, world: &World, camera: &Camera, color: Color) {
        for &pool_idx in &world.pair.active_pairs {
            let arbiter = &world.pair.pool[pool_idx].arbiter;
            if arbiter.manifold.point_count > 0 {
                let b_a = &world.bodies[arbiter.body_a_idx];
                let b_b = &world.bodies[arbiter.body_b_idx];

                if b_a.inv_m > 0.0 && b_b.inv_m > 0.0 {
                    let p1 = camera.transform_pos(&b_a.centroid);
                    let p2 = camera.transform_pos(&b_b.centroid);

                    let line_color =
                        if !b_a.is_awake && !b_b.is_awake && b_a.island_id == b_b.island_id {
                            let hue = (b_a.island_id as f32 * 0.3819) % 1.0;
                            hsl_to_rgb(hue, 1.0, 0.5)
                        } else {
                            color
                        };

                    draw_line(
                        p1.x as f32,
                        p1.y as f32,
                        p2.x as f32,
                        p2.y as f32,
                        1.0,
                        line_color,
                    );
                }
            }
        }
    }
}
