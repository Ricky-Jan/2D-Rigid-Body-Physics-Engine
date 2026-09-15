pub const PI: f64 = std::f64::consts::PI;
pub const TWO_PI: f64 = 2.0 * PI;

#[derive(Clone, Copy, Debug)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn zero() -> Self {
        Self { x: 0f64, y: 0f64 }
    }

    pub fn add(&self, b: &Self) -> Self {
        Self::new(self.x + b.x, self.y + b.y)
    }

    pub fn sub(&self, b: &Self) -> Self {
        Self::new(self.x - b.x, self.y - b.y)
    }

    pub fn scale(&self, scalar: f64) -> Self {
        Self::new(self.x * scalar, self.y * scalar)
    }

    pub fn dist_sq(&self) -> f64 {
        self.x * self.x + self.y * self.y
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }
}
