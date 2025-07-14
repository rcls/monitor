#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]
#![allow(unpredictable_function_pointer_comparisons)]

mod cpu;
//#[path = "nodebug.rs"]
mod debug;
#[allow(dead_code)]
mod lcd;
mod low_power;
mod utils;
mod vcell;

const LCD_BITS: u32 = 48;
type LcdBits = u64;

const I2C_LINES: i2c::I2CLines = i2c::I2CLines::B6_B7;

//const CPU_CLK: u32 = 2000000;
const CONFIG: cpu::CpuConfig = *cpu::CpuConfig::new(2000000)
    .debug_isr().i2c_isr();

const MAGIC: u32 = 0xc6ea33e;

const VERBOSE: bool = false;

const STANDBY_PRESERVE: u64 = lcd::STANDBY_PRESERVE;

macro_rules!verbose {
    ($($tt:tt)*) => {if VERBOSE {dbgln!($($tt)*);}}
}

fn tick() {
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};
    let seq = tamp.BKPR(1).read().bits();
    let seq = seq.wrapping_add(1); // As if!
    tamp.BKPR(1).write(|w| w.bits(seq));

    let com = seq & 1 != 0;
    let pos = (seq as u32 & 12) >> 2;
    let n = seq as u32 / 16 & 524287;
    let bcd = utils::to_bcd(n);
    let mut segments: LcdBits = 0;
    for i in 0..4 {
        let d = bcd >> 4 * i & 15;
        segments |= (lcd::DIGITS[d as usize] as LcdBits) << 8 * i;
    }
    let shift = pos * 8;
    segments = segments << shift | segments >> 48 - shift;
    lcd::update_lcd(segments, com);
}

fn main() -> ! {
    let pwr  = unsafe {&*stm32u031::PWR ::ptr()};
    let rcc  = unsafe {&*stm32u031::RCC ::ptr()};
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};

    // Enable IO clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit().GPIOBEN().set_bit());

    // Enable PWR before CPU init for now, so we get pristine values of
    // registers.
    rcc.APBENR1.modify(|_, w| w.PWREN().set_bit().RTCAPBEN().set_bit());

    cpu::init1();

    // Release standby pull-ups on GPIOs, they will look after themselves for
    // a few microseconds.
    pwr.CR3.write(|w| w.EIWUL().set_bit().APC().clear_bit());

    debug::init();

    verbose!("Debug up");

    verbose!("CPU init2");
    cpu::init2();

    let rcc_csr = rcc.CSR.read().bits();
    verbose!("RCC CSR {rcc_csr:08x}");

    low_power::rtc_enable();

    lcd::init();

    // If all of the reset reason, the magic number, and the woke-from-standby
    // flag agree, treat this as a restart.
    if rcc_csr & 0xfe000000 != 0 || tamp.BKPR(0).read().bits() != MAGIC
        || !pwr.SR1.read().SBF().bit() {
        dbgln!("**** RESTART ****");
        let tamp = unsafe {&*stm32u031::TAMP::ptr()};
        tamp.BKPR(0).write(|w| w.bits(MAGIC));
        low_power::ensure_options();
        low_power::rtc_setup_20Hz();
    }

    tick();

    verbose!("Sleeping...");

    low_power::standby();
}

#[used]
#[unsafe(link_section = ".vectors")]
static VECTORS: cpu::VectorTable = *cpu::VectorTable::new().debug_isr();
