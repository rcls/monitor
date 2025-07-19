#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![allow(internal_features)]
#![allow(unpredictable_function_pointer_comparisons)]
#![feature(const_default)]
#![feature(const_trait_impl)]
#![feature(format_args_nl)]
#![feature(link_llvm_intrinsics)]
#![feature(sync_unsafe_cell)]

mod cpu;
mod debug;
mod dma;
mod i2c;
#[allow(dead_code)]
mod lcd;
mod low_power;
mod utils;
mod vcell;

type LcdBits = u64;
const LCD_BITS: u32
    = match size_of::<LcdBits>() {4 => 32, 8 => 48, _ => panic!()};

const I2C_LINES: i2c::I2CLines = i2c::I2CLines::B6_B7;

const CONFIG: cpu::Config = {
    let mut cfg = cpu::Config::new(2000000);
    cfg.pullup |= 1 << 0x2d;
    cfg.standby_pu |= 1 << 0x2d; // PC13.
    cfg.pupd_use_pwr = true;
    *cfg.lazy_debug().lazy_i2c().lcd()
};

const MAGIC: u32 = 0xc6ea33e;

fn cold_start() {
    let pwr = unsafe {&*stm32u031::PWR::ptr()};

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

fn segments(temp: i32) -> u32 {
    let mut segs = lcd::DOT as u32 * 256;
    let mut p = 0;
    let mut quo = temp.unsigned_abs() as u16;
    while p < 32 && quo != 0 || p < 16 {
        let rem = quo % 10;
        quo = quo / 10;
        segs += (lcd::DIGITS[rem as usize] as u32) << p;
        p += 8;
    }
    if p < 32 && temp < 0 {
        segs += (lcd::MINUS as u32) << p;
    }
    segs
}

fn display(segments: u32, seq: u32) {
    let com = seq & 1 != 0;
    if LCD_BITS == 48 {
        let shift = seq as u16 % 48 & !7;
        let segments = segments as LcdBits;
        let segments = segments << shift | segments >> 48 - shift;
        lcd::update_lcd(segments, com);
    }
    else {
        lcd::update_lcd(segments as LcdBits, com);
    }
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

    // The alert pin should be released.  Reenable the pull-up.
    pwr.PUCRC.write(|w| w.bits(1 << 13));

    let segments = segments(temp);
    tamp.BKPR(2).write(|w| w.bits(segments));
    // Updating the display can wait for the next tick.  Disable the wake-up.
    pwr.CR3.modify(|_,w| w.EIWUL().set_bit().EWUP2().clear_bit());
}

fn tick() {
    let pwr  = unsafe {&*stm32u031::PWR ::ptr()};
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};

    let seq = tamp.BKPR(1).read().bits().wrapping_add(1);
    tamp.BKPR(1).write(|w| w.bits(seq));

    // On /32, command a conversion.
    let w;
    if seq & 31 == 0 {
        // Single shot conversion.  Arm and clear the wake-up flag.
        pwr.CR3.modify(|_,w| w.EIWUL().set_bit().EWUP2().set_bit());
        pwr.SCR.write(|w| w.CWUF2().set_bit());

        i2c::init();
        w = i2c::write(i2c::TMP117, &[1u8, 0xc, 0]);
    }
    else {
        w = i2c::Wait::new();
    }
    let segments = tamp.BKPR(2).read().bits();
    lcd::init();
    display(segments, seq);
    let _ = w.wait();
}

fn main() -> ! {
    let pwr  = unsafe {&*stm32u031::PWR ::ptr()};
    let rcc  = unsafe {&*stm32u031::RCC ::ptr()};
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};

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

    cpu::init2();

    let rcc_csr = rcc.CSR.read().bits();

    low_power::rtc_enable();

    let sr1 = pwr.SR1.read();

    // If all of the reset reason, the magic number, and the woke-from-standby
    // flag agree, treat this as a normal restart.  Else do a cold-start.
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

    low_power::standby();
}

fn counts_to_temp(c: i32) -> i32 {
    (c * 5 + 32) >> 6
}

fn next_temp(t: i32) -> i32 {
    let nm = t * 64 + 32 + 4;
    let nm = if nm >= 0 {nm} else {nm - 4};
    nm / 5
}

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

#[test]
fn test_segments() {
    for i in 0 ..= 9999 {
        let text = format!("{i:02}");
        let mut segs = 0;
        let mut minus = lcd::MINUS as u32;
        for c in text.chars() {
            segs = segs * 256 + lcd::DIGITS[c as usize - 48] as u32;
            minus <<= 8;
        }
        segs += lcd::DOT as u32 * 256;
        assert_eq!(segments(i), segs, "{i} {:x} {segs:x}", segments(i));
        if 0 < i && i <= 999 {
            assert_eq!(segments(-i), segs + minus);
        }
    }
}
