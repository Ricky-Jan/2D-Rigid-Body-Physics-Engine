use crate::math::{NEG_INF, POS_INF, Vec2};

#[derive(Clone, Copy, Debug)]
pub struct AABB {
    pub min: Vec2,
    pub max: Vec2,
}

impl AABB {
    pub fn new(min: Vec2, max: Vec2) -> Self {
        Self { min, max }
    }

    pub fn empty() -> Self {
        Self {
            min: Vec2::new(NEG_INF, NEG_INF),
            max: Vec2::new(POS_INF, POS_INF),
        }
    }

    pub fn intersect(&self, other: &AABB) -> bool {
        self.min.x <= other.max.x
            && self.max.x >= other.min.x
            && self.min.y <= other.max.y
            && self.max.y >= other.min.y
    }

    pub fn union(&self, other: &AABB) -> AABB {
        AABB {
            min: Vec2::new(self.min.x.min(other.min.x), self.min.y.min(other.min.y)),
            max: Vec2::new(self.max.x.max(other.max.x), self.max.y.max(other.max.y)),
        }
    }

    pub fn perimeter(&self) -> f64 {
        let width = self.max.x - self.min.x;
        let height = self.max.y - self.min.y;
        2.0 * (width + height)
    }

    pub fn fat_aabb(&self, vel: &Vec2, predict_rate: f64, fatness: f64) -> Self {
        let dx = vel.x * predict_rate;
        let dy = vel.y * predict_rate;

        Self {
            min: Vec2::new(
                self.min.x + dx.min(0.0) - fatness,
                self.min.y + dy.min(0.0) - fatness,
            ),

            max: Vec2::new(
                self.max.x + dx.max(0.0) + fatness,
                self.max.y + dy.max(0.0) + fatness,
            ),
        }
    }

    pub fn contain(&self, other: &AABB) -> bool {
        self.min.x <= other.min.x
            && self.min.y <= other.min.y
            && self.max.x >= other.max.x
            && self.max.y >= other.max.y
    }
}
