
pub const fn make_mask(bits: &[u32]) -> u32 {
    let mut mask = 0;
    let mut i = 0;
    while i < bits.len() {
        mask |= 1 << bits[i];
        i += 1;
    }
    mask
}
