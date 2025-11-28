const ENS220: u8 = 0x40;

use crate::i2c;
use crate::vcell::UCell;

static PRESSURE: UCell<[u8; 3]> = UCell::new([0; _]);

pub fn init() -> core::result::Result<u32, ()> {
    i2c::init();
    // Set high power mode.
    i2c::write(ENS220, &[6u8, 0x83]).wait()?;
    // Wait 0.5ms...
    for _ in 0..crate::CONFIG.clk / 4000 {
        crate::cpu::nothing();
    }
    // Set oversampling right down...
    i2c::write(ENS220, &[9u8, 0]).wait()?;
    // Start a conversion.
    i2c::write(ENS220, &[6u8, 0x93]).wait()?;
    // Wait 1ms for the conversion to complete.
    for _ in 0..crate::CONFIG.clk / 2000 {
        crate::cpu::nothing();
    }

    // Set oversampling to max.
    i2c::write(ENS220, &[9u8, 0x3f]).wait()?;

    // Go back to low power mode.
    i2c::write(ENS220, &[6u8, 3]).wait()?;

    // Read the pressure.
    i2c::read_reg(ENS220, 0x17, unsafe {PRESSURE.as_mut()}).wait()?;

    Ok(get_pressure())
}

/// Start a conversion.
pub fn start() -> i2c::Wait<'static> {
    i2c::init();
    i2c::write(ENS220, &[6u8, 0x93])
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
