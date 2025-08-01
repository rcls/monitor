#![allow(dead_code)]

pub const fn make_mask(bits: &[u32]) -> u32 {
    let mut mask = 0;
    let mut i = 0;
    while i < bits.len() {
        mask |= 1 << bits[i];
        i += 1;
    }
    mask
}

pub fn unreachable() -> ! {
    #[cfg(target_os = "none")]
    unsafe {
        // This will cause a compiler error if not removed by the optimizer.
        unsafe extern "C" {fn nowayjose();}
        nowayjose();
    }
    panic!();
}

#[macro_export]
macro_rules!link_assert {
    ($e:expr) => { if !$e {$crate::utils::unreachable()} }
}

/// Convert the input to 8 digit BCD.  If the input exceeds 8 decimal digits
/// then you get the last 8 digits of the decimal.
pub fn to_bcd(mut v: u32) -> u32 {
    if v == 0 {
        return 0;
    }
    let mut remain = 32;
    while v & 15 << 28 == 0 {
        v <<= 4;
        remain -= 4;
    }
    let pos = 0x11111111;
    let mut bcd = 0u32;
    for _ in 0..remain {
        let overflow = bcd + 3 * pos & pos * 8;
        bcd = bcd.wrapping_add(bcd + (overflow >> 1) + (overflow >> 2));
        bcd += v >> 31;
        v <<= 1;
    }
    bcd
}

pub const fn spread16(v: u16) -> u32 {
    let v = v as u32;
    let v = v + (v & 0xff00) * 0xff;
    let v = v + (v & 0x00f000f0) * 0xf;
    let v = v + (v & 0x0c0c0c0c) * 3;
    v + (v & 0x22222222)
}

#[inline]
pub const fn round_shr(x: u32, s: u32) -> u32 {
    (x + if s > 0 {1 << s-1} else {0}) >> s
}

#[test]
fn test_spread() {
    for i in 0..16 {
        assert_eq!(spread16(1 << i), 1 << 2 * i);
        for j in 0..16 {
            assert_eq!(spread16(1 << i | 1 << j), 1 << 2 * i | 1 << 2 * j);
        }
    }
}

#[test]
fn test_to_bcd() {
    for i in 0..65536 {
        assert_eq!(format!("{i}"), format!("{:x}", to_bcd(i)));
        let j = i * 0x10001;
        assert_eq!(format!("{}", j % 100000000), format!("{:x}", to_bcd(j)));
    }
}
