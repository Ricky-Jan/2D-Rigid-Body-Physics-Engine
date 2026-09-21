use crate::{
    math::{PI, Vec2},
    shape::{Circle, Polygon, Rect, ShapeType},
    world::World,
};
use macroquad::rand::{gen_range, rand};

pub struct TestScene;

impl TestScene {
    pub fn create_world() -> World {
        let mut world = World::new();

        // 1. 建立舞台：超大無邊界地板與防側漏圍牆
        world.create_rect(0.0, -100.0, 3200.0, 200.0, 0.0, 0.0);
        world.create_rect(-1500.0, 1100.0, 200.0, 3000.0, 0.0, 0.0);
        world.create_rect(1500.0, 1100.0, 200.0, 3000.0, 0.0, 0.0);

        // ==========================================
        // 2. 生成動態 Compound Letters: "FORCE2D"
        // ==========================================
        let letter_density = 2.0; // 給字母高一點的密度，讓它們有重量感
        let start_y = 150.0; // 讓字母開場時稍微離地，順勢落地站穩
        let spacing = 90.0; // 字母間距
        let mut current_x = -450.0; // 置中對齊的起始 X 座標

        // 字母 F
        let b_f = world.create_body(current_x, start_y, 0.0, letter_density);
        world.add_shape(
            b_f,
            ShapeType::Rect(Rect::new(20.0, 120.0)),
            Vec2::new(-30.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_f,
            ShapeType::Rect(Rect::new(60.0, 20.0)),
            Vec2::new(10.0, 50.0),
            0.0,
        );
        world.add_shape(
            b_f,
            ShapeType::Rect(Rect::new(40.0, 20.0)),
            Vec2::new(0.0, 0.0),
            0.0,
        );
        world.finalize_body(b_f);
        current_x += spacing;

        // 字母 O
        let b_o = world.create_body(current_x, start_y, 0.0, letter_density);
        world.add_shape(
            b_o,
            ShapeType::Rect(Rect::new(20.0, 120.0)),
            Vec2::new(-30.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_o,
            ShapeType::Rect(Rect::new(20.0, 120.0)),
            Vec2::new(30.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_o,
            ShapeType::Rect(Rect::new(40.0, 20.0)),
            Vec2::new(0.0, 50.0),
            0.0,
        );
        world.add_shape(
            b_o,
            ShapeType::Rect(Rect::new(40.0, 20.0)),
            Vec2::new(0.0, -50.0),
            0.0,
        );
        world.finalize_body(b_o);
        current_x += spacing;

        // 字母 R
        let b_r = world.create_body(current_x, start_y, 0.0, letter_density);
        world.add_shape(
            b_r,
            ShapeType::Rect(Rect::new(20.0, 120.0)),
            Vec2::new(-30.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_r,
            ShapeType::Rect(Rect::new(40.0, 20.0)),
            Vec2::new(0.0, 50.0),
            0.0,
        );
        world.add_shape(
            b_r,
            ShapeType::Rect(Rect::new(40.0, 20.0)),
            Vec2::new(0.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_r,
            ShapeType::Rect(Rect::new(20.0, 30.0)),
            Vec2::new(30.0, 25.0),
            0.0,
        );
        world.add_shape(
            b_r,
            ShapeType::Rect(Rect::new(20.0, 50.0)),
            Vec2::new(30.0, -35.0),
            0.0,
        );
        world.finalize_body(b_r);
        current_x += spacing;

        // 字母 C
        let b_c = world.create_body(current_x, start_y, 0.0, letter_density);
        world.add_shape(
            b_c,
            ShapeType::Rect(Rect::new(20.0, 120.0)),
            Vec2::new(-30.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_c,
            ShapeType::Rect(Rect::new(60.0, 20.0)),
            Vec2::new(10.0, 50.0),
            0.0,
        );
        world.add_shape(
            b_c,
            ShapeType::Rect(Rect::new(60.0, 20.0)),
            Vec2::new(10.0, -50.0),
            0.0,
        );
        world.finalize_body(b_c);
        current_x += spacing;

        // 字母 E
        let b_e = world.create_body(current_x, start_y, 0.0, letter_density);
        world.add_shape(
            b_e,
            ShapeType::Rect(Rect::new(20.0, 120.0)),
            Vec2::new(-30.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_e,
            ShapeType::Rect(Rect::new(60.0, 20.0)),
            Vec2::new(10.0, 50.0),
            0.0,
        );
        world.add_shape(
            b_e,
            ShapeType::Rect(Rect::new(40.0, 20.0)),
            Vec2::new(0.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_e,
            ShapeType::Rect(Rect::new(60.0, 20.0)),
            Vec2::new(10.0, -50.0),
            0.0,
        );
        world.finalize_body(b_e);
        current_x += spacing;

        // 數字 2
        let b_2 = world.create_body(current_x, start_y, 0.0, letter_density);
        world.add_shape(
            b_2,
            ShapeType::Rect(Rect::new(80.0, 20.0)),
            Vec2::new(0.0, 50.0),
            0.0,
        );
        world.add_shape(
            b_2,
            ShapeType::Rect(Rect::new(20.0, 40.0)),
            Vec2::new(30.0, 20.0),
            0.0,
        );
        world.add_shape(
            b_2,
            ShapeType::Rect(Rect::new(80.0, 20.0)),
            Vec2::new(0.0, -10.0),
            0.0,
        );
        world.add_shape(
            b_2,
            ShapeType::Rect(Rect::new(20.0, 20.0)),
            Vec2::new(-30.0, -30.0),
            0.0,
        );
        world.add_shape(
            b_2,
            ShapeType::Rect(Rect::new(80.0, 20.0)),
            Vec2::new(0.0, -50.0),
            0.0,
        );
        world.finalize_body(b_2);
        current_x += spacing;

        // 字母 D
        let b_d = world.create_body(current_x, start_y, 0.0, letter_density);
        world.add_shape(
            b_d,
            ShapeType::Rect(Rect::new(20.0, 120.0)),
            Vec2::new(-30.0, 0.0),
            0.0,
        );
        world.add_shape(
            b_d,
            ShapeType::Rect(Rect::new(40.0, 20.0)),
            Vec2::new(0.0, 50.0),
            0.0,
        );
        world.add_shape(
            b_d,
            ShapeType::Rect(Rect::new(40.0, 20.0)),
            Vec2::new(0.0, -50.0),
            0.0,
        );
        world.add_shape(
            b_d,
            ShapeType::Rect(Rect::new(20.0, 80.0)),
            Vec2::new(30.0, 0.0),
            0.0,
        ); // 右側封口
        world.finalize_body(b_d);

        // ==========================================
        // 3. 經過 1 秒後灑下來的物理雨 (垂直延遲法)
        // ==========================================
        for _ in 0..100 {
            let x = gen_range(-800.0, 500.0);

            // 巧妙之處：把 Y 軸初始點拉伸到 2500 ~ 15000 的高空。
            // 由於重力需要時間將它們拉下，這就天然形成了一個完美的 1~2 秒掉落延遲，
            // 讓底下剛生成的字母有時間「落地站穩」，等待迎接物理衝擊！
            let y = gen_range(1000.0, 2000.0);

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
