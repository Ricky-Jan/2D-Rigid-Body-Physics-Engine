use crate::{
    math::Vec2,
    settings::EPS,
    shape::{Circle, Shape},
    world::World,
};

pub struct Contact {
    pub pos: Vec2,
    pub normal: Vec2,
    pub penetration: f64,
}
pub struct Collision {
    pub contact1: Option<Contact>,
    pub contact2: Option<Contact>,
}
impl Collision {
    pub fn collide_pair(shape_a: Shape, shape_b: Shape, world: &World) -> Self {
        match (shape_a, shape_b) {
            (Shape::Circle(idx_a), Shape::Circle(idx_b)) => {
                let circle_a = &world.circle_all_list[idx_a];
                let circle_b = &world.circle_all_list[idx_b];
                Self::circle_vs_circle(circle_a, circle_b)
            }

            _ => Self::new_empty(),
        }
    }

    pub fn circle_vs_circle(circle_a: &Circle, circle_b: &Circle) -> Self {
        let delta = circle_b.pos.sub(&circle_a.pos);
        let dist_sq = delta.dist_sq();

        if dist_sq < EPS {
            return Self::new_empty();
        }

        let r_sum = circle_a.radius + circle_b.radius;
        if dist_sq < r_sum * r_sum {
            let dist = dist_sq.sqrt();
            let normal = delta.scale(1.0 / dist);
            let penetration = r_sum - dist;
            let pos = circle_a.pos.add(&normal.scale(circle_a.radius));
            Self {
                contact1: Some(Contact {
                    pos,
                    normal,
                    penetration,
                }),
                contact2: None,
            }
        } else {
            Self::new_empty()
        }
    }

    pub fn new_empty() -> Self {
        Self {
            contact1: None,
            contact2: None,
        }
    }
}
