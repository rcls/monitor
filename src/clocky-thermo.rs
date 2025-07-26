#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![allow(internal_features)]
#![allow(unpredictable_function_pointer_comparisons)]
#![feature(const_default)]
#![feature(const_trait_impl)]
#![feature(derive_const)]
#![feature(format_args_nl)]
#![feature(link_llvm_intrinsics)]

mod cpu;
mod debug;
mod dma;
mod i2c;
#[allow(unused)]
mod lcd;
#[allow(unused)]
mod rtc;
mod tmp117;
mod utils;
mod vcell;

const LCD_WIDTH: usize = 6;

const I2C_LINES: i2c::I2CLines = i2c::I2CLines::B6_B7;

const CONFIG: cpu::Config = {
    let mut cfg = cpu::Config::new(2000000);
    cfg.pullup |= 1 << 0x2d;
    cfg.standby_pu |= 1 << 0x2d; // PC13.
    cfg.fll = false;
    *cfg.lazy_debug().lazy_i2c().lcd().rtc()
};

const MAGIC: u32 = 0xc6ea33e;

fn cold_start() {
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};

    debug::banner("**** RESTART 0x", tamp.BKPR[8].read().bits(), " ****\n");
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};
    tamp.BKPR[0].write(|w| w.bits(MAGIC));
    rtc::ensure_options();
    rtc_setup_20Hz();

    tmp117::init();
}

#[allow(non_snake_case)]
fn rtc_setup_20Hz() {
    rtc::setup_start();

    // Clock input to WUT is LSE / {2,4,8,16}.
    // 32768 / 20 = 1638.4 ≈ 16 * 102.
    rtc::set_wakeup(101);

    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    rtc.CR.write(
        |w| w.WUTE().set_bit().WUTIE().set_bit().WUCKSEL().B_0x0()
            . BYPSHAD().set_bit());

    rtc::relock();
}

fn segments(temp: i32) -> u32 {
    let mut segs = lcd::DOT as u32 * 256;
    let mut p = 0;
    let mut quo = temp.unsigned_abs() as u16;
    while p < 32 && quo != 0 || p < 16 {
        let rem = quo % 10;
        quo /= 10;
        segs += (lcd::DIGITS[rem as usize] as u32) << p;
        p += 8;
    }
    if p < 32 && temp < 0 {
        segs += (lcd::MINUS as u32) << p;
    }
    if segs < 1 << 24 {
        segs = segs << 8 | lcd::DEG as u32
    }
    segs
}

fn display(segments: u32, seq: u32) {
    let com = seq & 1 != 0;
    if LCD_WIDTH == 6 {
        let shift = seq as u16 % 48 & !7;
        let segments = segments as lcd::Segments;
        let segments = segments << shift | segments >> 48 - shift;
        lcd::update_lcd(segments, com);
    }
    else {
        lcd::update_lcd(segments as lcd::Segments, com);
    }
}

fn alert() {
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};

    let temp = tmp117::alert();
    let segments = segments(temp);
    tamp.BKPR[2].write(|w| w.bits(segments));
    // Updating the display can wait for the next tick.
}

fn tick() {
    let rtc  = unsafe {&*stm32u031::RTC ::ptr()};
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};
    rtc.SCR.write(|w| w.bits(!0));

    let seq = tamp.BKPR[1].read().bits().wrapping_add(1);
    tamp.BKPR[1].write(|w| w.bits(seq));

    // On /32, command a conversion.
    let w = if seq & 31 == 0 {
        tmp117::acquire()
    }
    else {
        i2c::Wait::new()
    };
    let segments = tamp.BKPR[2].read().bits();
    lcd::init();
    display(segments, seq);
    let _ = w.wait();
}

fn main() -> ! {
    let pwr  = unsafe {&*stm32u031::PWR ::ptr()};
    let rcc  = unsafe {&*stm32u031::RCC ::ptr()};
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};

    // Enable IO clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit().GPIOBEN().set_bit());

    cpu::init1();
    cpu::init2();

    let rcc_csr = rcc.CSR.read().bits();

    let sr1 = pwr.SR1.read();

    // If all of the reset reason, the magic number, and the woke-from-standby
    // flag agree, treat this as a normal restart.  Else do a cold-start.
    if rcc_csr & 0xfe000000 != 0 || tamp.BKPR[0].read().bits() != MAGIC
        || !sr1.SBF().bit() {
        cold_start();
    }

    if sr1.WUF2().bit() {
        alert();
        pwr.SCR.write(|w| w.CWUF2().set_bit());
    }
    if sr1.WUFI().bit() {
        tick();
    }

    rtc::standby(sr1.WUFI().bit());
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
        let plus = if i <= 999 {segs * 256 + lcd::DEG as u32} else {segs};
        assert_eq!(segments(i), plus, "{i} {:x} {plus:x}", segments(i));
        if 0 < i && i <= 999 {
            let neg = segs + minus;
            let neg = if i <= 99 {neg * 256 + lcd::DEG as u32} else {neg};
            assert_eq!(segments(-i), neg);
        }
    }
}
