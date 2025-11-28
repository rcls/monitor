
use crate::i2c;

/// TMP117 I²C address.
pub const TMP117: u8 = 0x92;

macro_rules!dbgln {($($tt:tt)*) => {if false {crate::dbgln!($($tt)*)}};}

/// Initialize the TMP117.  Trigger a conversion if trigger.
pub fn init() {
    dbgln!("TMP117 init");
    crate::i2c::init();

    // Trigger an initial conversion, interrupt on data ready.
    const COMMAND: [u8; 3] = [1u8, 12, 4];
    let _ = i2c::write(TMP117, &COMMAND).wait();
}

pub fn acquire() -> i2c::Wait<'static> {
    dbgln!("TMP117 acquire");
    let pwr = unsafe {&*stm32u031::PWR::ptr()};

    // Single shot conversion.  Arm and clear the wake-up flag.
    pwr.CR3.modify(|_,w| w.EIWUL().set_bit().EWUP2().set_bit());
    pwr.SCR.write(|w| w.CWUF2().set_bit());

    i2c::init();
    const COMMAND: [u8; 3] = [1, 12, 0];
    i2c::write(TMP117, &COMMAND)
}

pub fn alert() -> i32 {
    dbgln!("TMP117 alert");
    let pwr = unsafe {&*stm32u031::PWR::ptr()};

    // Disable the pullup on PC13 while we do the I2C.
    pwr.PUCRC.write(|w| w.PU13().clear_bit());
    i2c::init();

    // Read the temperature...
    let mut countsbe = 0i16;
    let _ = i2c::read_reg(TMP117, 0, &mut countsbe).wait();
    // The docs say we need a read of config to clear the alert.
    let _ = i2c::read_reg(TMP117, 1, &mut 0i16).wait();
    let counts = i16::from_be(countsbe) as i32;
    let temp = counts_to_temp(counts);

    // Write the alert high and low registers.
    let upper = next_temp(temp).max(counts + 3).min( 0x7fff);
    let lower = prev_temp(temp).min(counts - 3).max(-0x8000);
    let _ = i2c::write(TMP117, &[2u8, (upper >> 8) as u8, upper as u8]);
    let _ = i2c::write(TMP117, &[3u8, (lower >> 8) as u8, lower as u8]);

    dbgln!("T = {temp} from {counts}");
    // The alert pin should be released.  Reenable the pull-up.
    pwr.PUCRC.write(|w| w.PU13().set_bit());

    temp
}

fn counts_to_temp(c: i32) -> i32 {
    c * 5 + 32 >> 6
}

fn next_temp(t: i32) -> i32 {
    let nm = t * 64 + 32 + 4;
    div5floor(nm)
}

fn prev_temp(t: i32) -> i32 {
    let pm = t * 64 - 32 - 1;
    div5floor(pm)
}

// We don't cover the full range...
#[inline(never)]
fn div5floor(t: i32) -> i32 {
    const ADJUST: i32 = 1 << 24;
    let t = t + ADJUST * 5;
    debug_assert!(t > 0);
    const M: i32 = 0x10000 * 4 / 5;
    let hi = (t >> 14) * M >> 4;
    let tt = t - hi * 5;
    let lo = tt * (M + 1) >> 18;
    hi + lo - ADJUST
}

#[test]
fn check_next_temp() {
    for t in -2600 ..= 2600 {
        let n = next_temp(t);
        assert_eq!(counts_to_temp(n), t + 1);
        assert_eq!(counts_to_temp(n - 1), t);
    }
}

#[test]
fn check_prev_temp() {
    for t in -2600 ..= 2600 {
        let p = prev_temp(t);
        assert_eq!(counts_to_temp(p), t - 1);
        assert_eq!(counts_to_temp(p + 1), t);
    }
}
