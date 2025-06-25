
#[derive(Copy, Clone, Default)]
pub struct Noise {
    pub last: u32,
    pub count: u8,
    pub noise: u32,
}

impl Noise {
    pub const fn new() -> Noise {Noise{last: 0, count: 0, noise: 0}}

    pub fn update(&mut self, value: u32) {
        let last = self.last;
        let count = self.count;
        self.last = value;
        if count < 128 {
            self.count += 1;
        }
        if count == 0 {
            return;
        }
        let delta = value.wrapping_sub(last);
        let delsq = delta.wrapping_mul(delta) << 8;
        let shift = count.leading_zeros();

        self.noise = self.noise.wrapping_add(
            (delsq << shift).wrapping_sub(self.noise >> (8 - shift)));
    }
    pub fn decimal(&self) -> u32 {
        let s = crate::usqrt::usqrt(self.noise);
        (s * 25 + 32) >> 6
    }
}
