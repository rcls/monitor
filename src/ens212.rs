
use crate::i2c;

const ENS212: u8 = 0x8a;

macro_rules!dbgln {($($tt:tt)*) => {if false {stm_common::dbgln!($($tt)*)}};}

pub fn init() -> core::result::Result<u32, ()> {
    // Set low power.
    i2c::write(ENS212, &[0x10u8, 1]).wait()?;
    // The docs are confusing.  Set one-shot and "stop continuous."
    i2c::write(ENS212, &[0x21u8, 0, 0, 3]).wait()?;
    // Trigger a conversion.
    i2c::write(ENS212, &[0x22u8, 3]).wait()?;
    // Wait 32ms ≈ 1/30 seconds.
    for _ in 0 .. crate::CONFIG.clk / 60 {
        crate::cpu::nothing();
    }
    get().wait()?;
    Ok(get_humidity())
}

pub fn start() -> i2c::Wait<'static> {
    dbgln!("ENS212 start");
    i2c::init();
    i2c::write(ENS212, &[0x22u8, 3])
}

static HUMIDITY: stm_common::vcell::UCell<u16> = Default::default();

pub fn get() -> i2c::Wait<'static> {
    dbgln!("ENS212 get");
    i2c::init();
    i2c::read_reg(ENS212, 0x33, unsafe {HUMIDITY.as_mut()})
}

/// Return the humidity in units of 0.1%.
pub fn get_humidity() -> u32 {
    // 512 counts for each 1% humidity.
    ((*HUMIDITY.as_ref() as u32 * 10 + 256) / 512).min(999)
}
