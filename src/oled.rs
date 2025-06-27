use crate::i2c;

const SH1106: u8 = 0x78;

#[inline]
fn nothing() {
    unsafe {core::arch::asm!("", options(nomem))}
}

pub fn init() {
    i2c::write_reg_start(
        SH1106, 0,
        &[0xaeu8, 0xad, 0x8b, 0xaf, 0xa5]);
    i2c::write_wait();

    for _ in 0.. 1<<21 {
        nothing();
    }
    i2c::write(SH1106, &[0u8, 0xa4]); // Display invert off.
    for _ in 0.. 1<<21 {
        nothing();
    }
    i2c::write(SH1106, &[0u8, 0xae]); // Display off.
}
