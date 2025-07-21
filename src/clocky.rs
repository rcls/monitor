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
mod lcd;
mod rtc;
mod utils;
mod vcell;

type LcdBits = u64;
const LCD_BITS: usize = 48;

const CONFIG: cpu::Config = *cpu::Config::new(2000000).lazy_debug().lcd().rtc();

const MAGIC: u32 = 0xd6ea33e;

fn cold_start() {
    //let pwr = unsafe {&*stm32u031::PWR::ptr()};
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};

    dbgln!("**** CLOCKY {:#x} ****", tamp.BKPR[8].read().bits());
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};
    tamp.BKPR[0].write(|w| w.bits(MAGIC));
    rtc::ensure_options();
    rtc_setup_20Hz_plus_1Hz();

    // TODO - send the TMP117 into shutdown.
}

#[allow(non_snake_case)]
pub fn rtc_setup_20Hz_plus_1Hz() {
    rtc::setup_start();
    rtc::set_wakeup(101);                // ≈20.07 Hz
    // Set the alarm A to fire once / sec.
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    rtc.ALRMAR.write(|w| w.bits(!0));

    rtc.CR.write(
        |w| w.WUTE().set_bit().WUTIE().set_bit().WUCKSEL().B_0x0()
            . ALRAE().set_bit().ALRAIE().set_bit().BYPSHAD().set_bit());
    rtc::rtc_setup_end();
}

fn main() -> ! {
    let pwr  = unsafe {&*stm32u031::PWR ::ptr()};
    let rcc  = unsafe {&*stm32u031::RCC ::ptr()};
    let rtc  = unsafe {&*stm32u031::RTC ::ptr()};
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

    let sr = rtc.SR.read();

    let mut seq = tamp.BKPR[1].read().bits();
    if sr.WUTF().bit() {
        seq += 1;
        tamp.BKPR[1].write(|w| w.bits(seq));
    }

    let time1 = rtc.TR.read().bits() as usize;
    let date = rtc.DR.read().bits();
    let time = rtc.TR.read().bits() as usize;
    let date = if time == time1 {date} else {rtc.DR.read().bits()};

    lcd::init();
    let mut segments = 0;
    if seq & 15 < 8 {
        segments |= lcd::DOT << 48 | lcd::DOT;
    }
    for i in (0..24).step_by(4) {
        segments |= (lcd::DIGITS[time >> i & 0xf] as u64) << i * 2;
    }
    lcd::update_lcd(segments, seq & 1 != 0);

    if sr.ALRAF().bit() {
        dbgln!("{:02x} {:02x} {:02x} ({:x}) {:02x} {:02x} {:02x} {:#x}",
               date >> 16 & 0xff, date >> 8 & 0x1f, date & 0xff,
               date >> 13 & 7,
               time >> 16 & 0xff, time >> 8 & 0xff, time & 0xff,
               rtc.SSR().read().bits());
    }

    rtc::standby(true);
}
