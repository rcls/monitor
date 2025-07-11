
#[allow(dead_code)]
pub const fn make_mask(bits: &[u32]) -> u32 {
    let mut mask = 0;
    let mut i = 0;
    while i < bits.len() {
        mask |= 1 << bits[i];
        i += 1;
    }
    mask
}

#[allow(dead_code)]
pub fn unreachable() -> ! {
    #[cfg(target_os = "none")]
    unsafe {
        // This will cause a compiler error if not removed by the optimizer.
        unsafe extern "C" {fn nowayjose();}
        nowayjose();
    }
    panic!();
}

#[allow(dead_code)]
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
