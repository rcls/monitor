use crate::usqrt::usqrt;

#[derive(Copy, Clone)]
#[derive_const(Default)]
pub struct Noise {
    pub last: u32,
    pub count: u8,
    pub noise: u32,
}

impl Noise {
    pub fn update(&mut self, value: u32) {
        let last = self.last;
        let count = self.count;
        self.last = value;
        if count <= 128 {
            self.count += 1;
        }
        if count == 0 {
            return;
        }
        let delta = value.wrapping_sub(last);
        let delsq = delta.wrapping_mul(delta) << 8;
        let shift = (count - 1).leading_zeros();

        self.noise = self.noise.wrapping_add(
            (delsq << shift).wrapping_sub(self.noise >> (8 - shift)));
    }
    pub fn decimal(&self) -> u32 {
        let s = usqrt(self.noise);
        (s * 25 + 32) >> 6
    }
}

#[test]
fn noise1() {
    let mut noise = Noise::default();
    noise.update(11);
    for i in 0..10000 {
        noise.update(10 + (i & 1));
        assert_eq!(noise.noise, 65536);
        assert_eq!(noise.decimal(), 100);
    }
}

#[test]
fn noise2() {
    let mut noise = Noise::default();
    noise.update(10);
    noise.update(10);
    noise.update(11);
    assert_eq!(noise.noise, 32768);
    assert_eq!(noise.decimal(), 5000f64.sqrt().round() as u32);
}

#[test]
fn noise4() {
    let mut noise = Noise::default();
    [10, 10, 10, 10, 11].into_iter().for_each(|x| noise.update(x));
    assert_eq!(noise.noise, 16384);
    assert_eq!(noise.decimal(), 50);
}
