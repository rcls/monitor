#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(const_default)]
#![feature(const_trait_impl)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]
#![allow(unpredictable_function_pointer_comparisons)]

mod cpu;
//#[path = "nodebug.rs"]
mod debug;
mod dma;
mod i2c;
#[allow(dead_code)]
mod lcd;
mod low_power;
mod utils;
mod vcell;

const LCD_BITS: u32 = 48;
type LcdBits = u64;

const I2C_LINES: i2c::I2CLines = i2c::I2CLines::B6_B7;

const CONFIG: cpu::Config = {
    let mut cfg = cpu::Config::new(2000000);
    cfg.pullup |= 1 << 0x2d;
    cfg.standby_pu |= 1 << 0x2d; // PC13.
    cfg.pupd_use_pwr = true;
    *cfg.lazy_debug().lazy_i2c().lcd()
};

const MAGIC: u32 = 0xc6ea33e;

const VERBOSE: bool = false;

macro_rules!verbose {
    ($($tt:tt)*) => {if VERBOSE {dbgln!($($tt)*);}}
}

fn cold_start() {
    let pwr  = unsafe {&*stm32u031::PWR ::ptr()};

    dbgln!("**** RESTART ****");
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};
    tamp.BKPR(0).write(|w| w.bits(MAGIC));
    low_power::ensure_options();
    low_power::rtc_setup_20Hz();

    // Initialize the TMP117 by requesting a conversion.
    i2c::init();
    const CONFIG: u16 = 0x0c04; // One shot, alert when ready.
    let _ = i2c::write(i2c::TMP117, &[1u8, (CONFIG >> 8) as u8, CONFIG as u8])
        .wait();

    // Alert is hot-wired to PC13, WKUP2.  Set it to falling edge triggered.
    pwr.CR4.write(|w| w.WP2().set_bit());
    pwr.CR3.modify(|_,w| w.EIWUL().set_bit().EWUP2().set_bit());
    // Clear the wake-up pin as it has probably gotten set during start-up.
    pwr.SCR.write(|w| w.CWUF2().set_bit());
}

// Alert is on PB8
fn display(temp: i32, seq: u32) {
    let com = seq & 1 != 0;
    let pos = seq / 8 % 6;

    // Convert temp to centidegrees.
    //sdbgln!("Counts = {} temp = {}", temp, (temp * 100 + 64) >> 7);
    let temp = counts_to_temp(temp);
    let neg = temp < 0;
    let bcd = utils::to_bcd(if !neg {temp as u32} else {-temp as u32});
    let mut segments: LcdBits = (lcd::DOT as u64) << 8;
    // FIXME insert minus, remove trailing zeros.
    for i in 0..4 {
        let d = bcd >> 4 * i & 15;
        segments |= (lcd::DIGITS[d as usize] as LcdBits) << 8 * i;
    }
    let shift = pos * 8;
    segments = segments << shift | segments >> 48 - shift;
    lcd::update_lcd(segments, com);
}

fn alert() {
    let pwr = unsafe {&*stm32u031::PWR::ptr()};
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};

    // Disable the pullup on PC13 while we do the I2C.
    pwr.PUCRC.write(|w| w.bits(0));
    i2c::init();

    // Read the temperature...
    let mut countsbe = 0i16;
    let _ = i2c::read_reg(i2c::TMP117, 0, &mut countsbe).wait();
    // The docs say we need a read of config to clear the alert.
    let _ = i2c::read_reg(i2c::TMP117, 1, &mut 0i16).wait();
    let counts = i16::from_be(countsbe) as i32;
    let temp = counts_to_temp(counts);
    //dbgln!("Alert! {} {}", temp, tamp.BKPR(1).read().bits());
    // Write the alert high and low registers.
    let upper = next_temp(temp).max(counts + 3).min( 0x7fff);
    let lower = prev_temp(temp).min(counts - 3).max(-0x8000);
    let _ = i2c::write(i2c::TMP117, &[2u8, (upper >> 8) as u8, upper as u8]);
    let _ = i2c::write(i2c::TMP117, &[3u8, (lower >> 8) as u8, lower as u8]);

    // The alert pin should be released.  Reenable the pull-up.  Writing the
    // display should give heaps of time for it to rise.
    pwr.PUCRC.write(|w| w.bits(1 << 13));

    tamp.BKPR(2).write(|w| w.bits(counts as u32));
    display(counts, tamp.BKPR(1).read().bits());
    //dbgln!("Alert done...");
}

fn tick() {
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};
    let seq = tamp.BKPR(1).read().bits();
    // Divide by 32...
    let seq = seq.wrapping_add(1);
    tamp.BKPR(1).write(|w| w.bits(seq));

    // On 0, command a one shot temperature conversion.  On 1 pick it up.
    let mut temp = tamp.BKPR(2).read().bits() as i32;
    if seq & 31 == 0 {
        // Single shot conversion.
        // Alert reflects limits, active low.
        i2c::init();
        let _ = i2c::write(i2c::TMP117, &[1u8, 0xc, 0]).wait().unwrap();
    }
    else if false && seq & 31 == 1 {
        let mut countsbe = 0i16;
        let _ = i2c::read_reg(i2c::TMP117, 0, &mut countsbe).wait().unwrap();
        let counts = i16::from_be(countsbe) as i32;
        tamp.BKPR(2).write(|w| w.bits(counts as u32));
        temp = counts;
    }
    display(temp, seq);
}

fn main() -> ! {
    let pwr   = unsafe {&*stm32u031::PWR  ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    let tamp  = unsafe {&*stm32u031::TAMP ::ptr()};

    // TODO - lazy set up of all the clocks!
    // Enable IO clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit().GPIOBEN().set_bit());

    // LED is on PA1.
    // let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    // gpioa.MODER.write(|w| w.MODE1().B_0x1());
    // gpioa.BSRR.write(|w| w.BS1().set_bit());

    // Enable PWR before CPU init for now, so we get pristine values of
    // registers.
    // FIXME - work out what is needed now!
    rcc.APBENR1.modify(|_, w| w.PWREN().set_bit().RTCAPBEN().set_bit());

    cpu::init1();

    // We use lazy debug!
    verbose!("Debug up");

    verbose!("CPU init2");
    cpu::init2();

    let rcc_csr = rcc.CSR.read().bits();
    verbose!("RCC CSR {rcc_csr:08x}");

    low_power::rtc_enable();

    //i2c::init();

    lcd::init();

    let sr1 = pwr.SR1.read();

    // If all of the reset reason, the magic number, and the woke-from-standby
    // flag agree, treat this as a restart.  Else do a cold-start.
    if rcc_csr & 0xfe000000 != 0 || tamp.BKPR(0).read().bits() != MAGIC
        || !sr1.SBF().bit() {
        cold_start();
    }
    if sr1.WUF2().bit() {
        alert();
        // Processing alert() should give the pin time to rise.
        pwr.SCR.write(|w| w.CWUF2().set_bit());
    }
    if sr1.WUFI().bit() {
        tick();
    }

    verbose!("Sleeping...");

    // FIXME handling of PC13.
    low_power::standby();
}

fn counts_to_temp(c: i32) -> i32 {
    (c * 5 + 32) >> 6
}

#[allow(dead_code)]
fn next_temp(t: i32) -> i32 {
    let nm = t * 64 + 32 + 4;
    let nm = if nm >= 0 {nm} else {nm - 4};
    nm / 5
}

#[allow(dead_code)]
fn prev_temp(t: i32) -> i32 {
    let pm = t * 64 - 32 - 1;
    let pm = if pm >= 0 {pm} else {pm - 4};
    pm / 5
}

#[test]
fn check_next_temp() {
    for t in -999 ..= 1499 {
        let n = next_temp(t);
        assert_eq!(counts_to_temp(n), t + 1);
        assert_eq!(counts_to_temp(n - 1), t);
    }
}

#[test]
fn check_prev_temp() {
    for t in -999 ..= 1499 {
        let p = prev_temp(t);
        assert_eq!(counts_to_temp(p), t - 1);
        assert_eq!(counts_to_temp(p + 1), t);
    }
}
