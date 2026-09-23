use crate::{
    bvh::NULL_PTR,
    math::{PI, Vec2},
    shape::{Circle, Polygon, Rect, ShapeType},
    world::World,
};
use macroquad::rand::{gen_range, rand};

pub struct TestScene;

impl TestScene {
    pub fn create_world() -> World {
        let mut world = World::new();

        let mut set_filter =
            |world: &mut World, body_idx: usize, cat: u16, mask: u16, group: i16| {
                let mut curr = world.bodies[body_idx].shape_head;
                while curr != NULL_PTR {
                    world.shapes[curr].filter.category_bits = cat;
                    world.shapes[curr].filter.mask_bits = mask;
                    world.shapes[curr].filter.group_index = group;
                    curr = world.shapes[curr].next;
                }
            };

        // ==========================================
        // 0. Scene Boundaries (Static Walls)
        // ==========================================
        world.create_rect(0.0, -100.0, 4000.0, 200.0, 0.0, 0.0);
        world.create_rect(-2100.0, 900.0, 200.0, 2200.0, 0.0, 0.0);
        world.create_rect(2100.0, 900.0, 200.0, 2200.0, 0.0, 0.0);

        // ==========================================
        // 1. Distance Joint + Inset Anchor
        // ==========================================
        let plank_w = 100.0;
        let plank_h = 24.0;

        let indent = plank_h / 2.0;
        let anchor_x = plank_w / 2.0 - indent;
        let rest_len = indent * 2.0;

        let bridge_start_x = -700.0;
        let bridge_y = 1100.0;
        let plank_count = 15;

        let pillar_l = world.create_circle(bridge_start_x - 40.0, bridge_y, 20.0, 0.0, 0.0);
        let pillar_r = world.create_circle(
            bridge_start_x + (plank_count as f64) * plank_w,
            bridge_y,
            20.0,
            0.0,
            0.0,
        );
        set_filter(&mut world, pillar_l, 1, 0xFFFF, -1);
        set_filter(&mut world, pillar_r, 1, 0xFFFF, -1);

        let mut prev_body = pillar_l;
        let mut prev_anchor = Vec2::new(20.0, 0.0);

        for i in 0..plank_count {
            let x = bridge_start_x + (i as f64) * plank_w + (plank_w / 2.0);
            let plank = world.create_rect(x, bridge_y, plank_w, plank_h, 0.0, 1.5);

            set_filter(&mut world, plank, 1, 0xFFFF, -1);

            world.create_distance_joint(
                prev_body,
                plank,
                prev_anchor,
                Vec2::new(-anchor_x, 0.0),
                if i == 0 { indent } else { rest_len },
                10.0,
                15.0,
            );
            prev_body = plank;
            prev_anchor = Vec2::new(anchor_x, 0.0);
        }

        world.create_distance_joint(
            prev_body,
            pillar_r,
            prev_anchor,
            Vec2::new(-20.0, 0.0),
            indent,
            15.0,
            5.0,
        );

        // ==========================================
        // 2. Ragdoll
        // ==========================================
        let rag_x = 0.0;
        let rag_y = 1600.0;

        let head = world.create_circle(rag_x, rag_y + 70.0, 15.0, 0.0, 1.0);
        let torso = world.create_rect(rag_x, rag_y + 15.0, 30.0, 60.0, 0.0, 1.5);
        let arm_l = world.create_rect(rag_x - 35.0, rag_y + 30.0, 40.0, 15.0, 0.0, 0.8);
        let arm_r = world.create_rect(rag_x + 35.0, rag_y + 30.0, 40.0, 15.0, 0.0, 0.8);
        let leg_l = world.create_rect(rag_x - 10.0, rag_y - 40.0, 15.0, 50.0, 0.0, 1.0);
        let leg_r = world.create_rect(rag_x + 10.0, rag_y - 40.0, 15.0, 50.0, 0.0, 1.0);

        world.create_revolute_joint(head, torso, Vec2::new(0.0, -15.0), Vec2::new(0.0, 30.0));
        world.create_revolute_joint(torso, arm_l, Vec2::new(-15.0, 15.0), Vec2::new(20.0, 0.0));
        world.create_revolute_joint(torso, arm_r, Vec2::new(15.0, 15.0), Vec2::new(-20.0, 0.0));
        world.create_revolute_joint(torso, leg_l, Vec2::new(-10.0, -30.0), Vec2::new(0.0, 25.0));
        world.create_revolute_joint(torso, leg_r, Vec2::new(10.0, -30.0), Vec2::new(0.0, 25.0));

        // ==========================================
        // 3. Box Pyramid
        // ==========================================
        let base_count = 15;
        let box_size = 40.0;
        let start_x = 1000.0;
        for row in 0..base_count {
            let y = 50.0 + (row as f64) * box_size;
            let cols = base_count - row;
            for col in 0..cols {
                let x = start_x + (col as f64) * box_size - (cols as f64 * box_size) / 2.0;
                world.create_rect(x, y, box_size - 2.0, box_size - 2.0, 0.0, 1.0);
            }
        }

        // ==========================================
        // 4. Static Giant Shapes (Circle, Triangle, Concave Funnel)
        // ==========================================
        world.create_circle(-1500.0, 500.0, 80.0, 0.0, 0.0);

        world.create_regular_polygon(-1000.0, 400.0, 3, 100.0, PI / 6.0, 0.0);

        world.create_rect(-1300.0, 250.0, 20.0, 150.0, PI / 6.0, 0.0);
        world.create_rect(-1100.0, 250.0, 20.0, 150.0, -PI / 6.0, 0.0);
        world.create_rect(-1200.0, 190.0, 145.0, 20.0, 0.0, 0.0);

        // ==========================================
        // 5. Dynamic Debris & Compound
        // ==========================================
        let car_body = world.create_body(-1500.0, 1400.0, -0.2, 2.0);
        world.add_shape(
            car_body,
            ShapeType::Rect(Rect::new(80.0, 30.0)),
            Vec2::zero(),
            0.0,
        );
        world.add_shape(
            car_body,
            ShapeType::Circle(Circle::new(20.0)),
            Vec2::new(-35.0, -15.0),
            0.0,
        );
        world.add_shape(
            car_body,
            ShapeType::Circle(Circle::new(20.0)),
            Vec2::new(35.0, -15.0),
            0.0,
        );
        world.finalize_body(car_body);

        // Randomly generate some debris shapes
        for i in 0..6 {
            for j in 0..5 {
                let x = -1600.0 + (j as f64) * 100.0 + gen_range(-10.0, 10.0);
                let y = 800.0 + (i as f64) * 80.0;
                let angle = gen_range(0.0, PI);

                match rand() % 3 {
                    0 => {
                        world.create_circle(x, y, gen_range(15.0, 25.0), angle, 1.0);
                    }
                    1 => {
                        world.create_rect(
                            x,
                            y,
                            gen_range(30.0, 50.0),
                            gen_range(30.0, 50.0),
                            angle,
                            1.0,
                        );
                    }
                    _ => {
                        world.create_regular_polygon(
                            x,
                            y,
                            gen_range(4, 7) as usize,
                            gen_range(20.0, 30.0),
                            angle,
                            1.0,
                        );
                    }
                }
            }
        }

        world
    }
}
