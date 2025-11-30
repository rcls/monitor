const ENS220: u8 = 0x40;

use crate::i2c;
use crate::vcell::UCell;

static PRESSURE: UCell<[u8; 3]> = UCell::new([0; _]);

pub fn init() -> core::result::Result<u32, ()> {
    // Set high power mode.
    i2c::write(ENS220, &[6u8, 0x83]).wait()?;
    // Wait 0.5ms...
    for _ in 0..crate::CONFIG.clk / 4000 {
        crate::cpu::nothing();
    }

    // Get a quick sample: One-shot, P_CONV=1ms, PT_RATE=1, OVSP=OVST=1
    i2c::write(ENS220, &[7u8, 0x00, 0x01, 0x00]).wait()?;
    // Start a conversion.
    i2c::write(ENS220, &[6u8, 0x93]).wait()?;

    // Wait 5ms for the conversion to complete.
    for _ in 0..crate::CONFIG.clk / 400 {
        crate::cpu::nothing();
    }

    // Set one-shot, P_CONV=4ms, PT_RATE=1, OVSP=32, OVST=8.
    i2c::write(ENS220, &[7u8, 0x10, 0x01, 0x2b]).wait()?;

    // Go back to low power mode.
    i2c::write(ENS220, &[6u8, 3]).wait()?;

    // Read the pressure.
    i2c::read_reg(ENS220, 0x17, unsafe {PRESSURE.as_mut()}).wait()?;

    Ok(get_pressure())
}

/// Start a conversion.
pub fn start() -> i2c::Wait<'static> {
    i2c::init();
    i2c::write(ENS220, &[6u8, 0x13])
}

/// Get a conversion result.
pub fn get() -> i2c::Wait<'static> {
    i2c::init();
    i2c::read_reg(ENS220, 0x17, unsafe {PRESSURE.as_mut()})
}

/// Retrieve the pressure.
pub fn get_pressure() -> u32 {
    let p = PRESSURE.as_ref();
    p[0] as u32 + p[1] as u32 * 256 + p[2] as u32 * 65536
}
