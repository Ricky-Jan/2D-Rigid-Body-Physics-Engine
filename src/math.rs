pub const EPS: f64 = 1e-6f64;
pub const PI: f64 = std::f64::consts::PI;
pub const TWO_PI: f64 = 2.0 * PI;
pub const POS_INF: f64 = f64::INFINITY;
pub const NEG_INF: f64 = f64::NEG_INFINITY;
pub const PRIME_A: usize = 73856093;
pub const PRIME_B: usize = 19349663;

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

    pub fn dot(&self, b: &Self) -> f64 {
        self.x * b.x + self.y * b.y
    }

    pub fn cross(&self, b: &Self) -> f64 {
        self.x * b.y - self.y * b.x
    }

    pub fn cross_sv(scalar: f64, b: &Self) -> Self {
        Self::new(-scalar * b.y, scalar * b.x)
    }

    pub fn reverse(&self) -> Self {
        Self::new(-self.x, -self.y)
    }

    pub fn normal(&self) -> Self {
        Self::new(self.y, -self.x)
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

    #[inline(always)]
    pub fn rotate(&self, v: &Vec2) -> Vec2 {
        Vec2::new(v.x * self.re - v.y * self.im, v.x * self.im + v.y * self.re)
    }

    #[inline(always)]
    pub fn inv_rotate(&self, v: &Vec2) -> Vec2 {
        Vec2::new(
            v.x * self.re + v.y * self.im,
            -v.x * self.im + v.y * self.re,
        )
    }
}
