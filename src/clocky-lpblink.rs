#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]
#![allow(unpredictable_function_pointer_comparisons)]

mod cpu;
mod debug;
#[allow(dead_code)]
mod lcd;
mod utils;
mod vcell;

const LCD_BITS: u32 = 48;
type LcdBits = u64;

const CPU_CLK: u32 = 2000000;

const MAGIC: u32 = 0xc6ea33e;

const VERBOSE: bool = false;

macro_rules!verbose {
    ($($tt:tt)*) => {if VERBOSE {dbgln!($($tt)*)}}
}

fn rtc_setup() {
    let rtc  = unsafe {&*stm32u031::RTC ::ptr()};

    // To wake up from Stop mode with an RTC alarm event, it is necessary to:
    // •Configure the EXTI Line 18 to be sensitive to rising edge
    // •Configure the RTC to generate the RTC alarm
    // To wake up from Standby mode, there is no need to configure the EXTI Line 18.
    //
    // To wake up from Stop mode with an RTC wake-up event, it is necessary to:
    // •Configure the EXTI Line 20 to be sensitive to rising edge
    // •Configure the RTC to generate the RTC alarm
    //
    // .... bit that doesn't match the docs, its EXTI line 28.
    //exti.RTSR1.write(|w| w.RT28().set_bit());
    // None of these need from standby or shutdown?

    rtc.WPR.write(|w| w.bits(0xca));
    rtc.WPR.write(|w| w.bits(0x53));

    rtc.CR.write(|w| w.WUTE().clear_bit());
    while !rtc.ICSR.read().WUTWF().bit() {
    }

    // Clock input to WUT is LSE / {2,4,8,16}.
    // 32768 / 20 = 1638.4 ≈ 16 * 102.
    rtc.WUTR.write(|w| w.WUT().bits(101));

    rtc.CR.write(|w| w.WUTE().set_bit().WUTIE().set_bit().WUCKSEL().B_0x0());
    // Is this needed?
    //while rtc.ICSR.read().WUTWF().bit() {
    //}

    // Reprotect.
    rtc.WPR.write(|w| w.bits(0));
    rtc.WPR.write(|w| w.bits(0));
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

fn check_options() {
    let flash = unsafe {&*stm32u031::FLASH::ptr()};

    let options = flash.OPTR.read();
    dbgln!("Options = {:#010x}", options.bits());
    if options.NRST_MODE().bits() == 1 {
        return;
    }
    dbgln!("... needs to be rewritten!");

    // 1.Write KEY1 = 0x4567 0123 in the FLASH key register (FLASH_KEYR)
    // 2.Write KEY2 = 0xCDEF 89AB in the FLASH key register (FLASH_KEYR).
    flash.KEYR.write(|w| w.bits(0x45670123));
    flash.KEYR.write(|w| w.bits(0xcdef89ab));
    // 1.Unlock the FLASH_CR with the LOCK clearing sequence (refer to
    // Unlocking the flash memory ).
    // 2.Write OPTKEY1 = 0x0819 2A3B of the FLASH option key register
    //  (FLASH_OPTKEYR).
    // 3.Write OPTKEY2 = 0x4C5D 6E7F of the FLASH option key register
    // (FLASH_OPTKEYR)
    flash.OPTKEYR.write(|w| w.bits(0x08192a3b));
    flash.OPTKEYR.write(|w| w.bits(0x4c5d6e7f));

    // Write the options...
    flash.OPTR.modify(|_,w| w.NRST_MODE().B_0x1());

    while flash.SR.read().BSY1().bit() {}
    flash.CR.write(|w| w.OPTSTRT().set_bit());
    while flash.SR.read().BSY1().bit() {}
    dbgln!("Options written!");
}

fn main() -> ! {
    // let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    // let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let pwr  = unsafe {&*stm32u031::PWR ::ptr()};
    let rcc  = unsafe {&*stm32u031::RCC ::ptr()};
    let rtc  = unsafe {&*stm32u031::RTC ::ptr()};
    let tamp = unsafe {&*stm32u031::TAMP::ptr()};
    // let tsc   = unsafe {&*stm32u031::TSC  ::ptr()};

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

    // Clear any wakeup bits.
    //dbgln!("Pre clear {}", rtc.SR.read().bits());
    //rtc.SCR.write(|w| w.bits(4));
    //dbgln!("Pst clear {}", rtc.SR.read().bits());

    let rcc_csr = rcc.CSR.read().bits();
    verbose!("RCC CSR {rcc_csr:08x}");

    rcc.APBENR1.modify(|_, w| w.PWREN().set_bit().RTCAPBEN().set_bit());

    rcc.BDCR.modify(|_,w| w.RTCEN().set_bit().RTCSEL().B_0x1());
    pwr.CR1.modify(|_,w| w.LPMS().bits(3)); // 3=Standby, 4=Shutdown.

    lcd::init();

    // If all of the reset reason, the magic number, and the woke-from-standby
    // flag agree, treat this as a restart.
    if rcc_csr & 0xfe000000 != 0 || tamp.BKPR(0).read().bits() != MAGIC
        || !pwr.SR1.read().SBF().bit() {
        dbgln!("**** RESTART ****");
        let tamp = unsafe {&*stm32u031::TAMP::ptr()};
        tamp.BKPR(0).write(|w| w.bits(MAGIC));
        check_options();
        rtc_setup();
    }

    tick();

    // Set the backup registers.
    lcd::backup();
    pwr.CR3.write(|w| w.EIWUL().set_bit().APC().set_bit());

    // Clear the reset causes & set restart MSI. Note that we need to do this
    // late in the game else the WUFI bits ressurrects itself!
    rcc.CSR.write(|w| w.RMVF().set_bit().MSISRANGE().bits(5));
    verbose!("Sleeping...");

    // Wait for the UART to become idle before entering deep sleep.
    // Poll for TC on the UART.  (Ugh.)
    let uart = unsafe {&*stm32u031::LPUART1::ptr()};
    while !uart.ISR.read().TC().bit() {
    }

    // Clear the wake-up flags.  This seems to need to be some time after the
    // wake-up!  Presumably a clock-tick of the RTC domain.
    rtc.SCR.write(|w| w.bits(!0));

    //pwr.CR3.write(|w| w.EIWUL().set_bit()); // Default!
    // Deep sleep.
    let scb = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    unsafe {scb.scr.write(4)};

    loop {
        cpu::WFE();
    }
}

#[used]
#[unsafe(link_section = ".vectors")]
static VECTORS: cpu::VectorTable = *cpu::VectorTable::new()
    .rcc_isr().debug_isr();
