#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]
#![feature(str_from_raw_parts)]

use adc::{adc_isr, dma1_isr};
use vcell::{UCell, VCell, WFE};
use noise::Noise;

mod adc;
mod dma;
mod debug;
mod i2c;
mod noise;
mod oled;
mod usqrt;
mod vcell;

type I2C = stm32u031::I2C1;

static TEMP: VCell<i16> = VCell::new(0);
static NOISE: UCell<[Noise; 4]> = UCell::new([Noise::new(); 4]);

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

fn systick_handler() {
    // sdbgln!("SYSTICK enter");

    adc::start();

    i2c::read_reg_start(i2c::TMP117, 0, TEMP.as_ptr());

    // Wait for both I2C and ADC done.
    while !i2c::CONTEXT.done.read() || !adc::DONE.read() {
        WFE();
    }

    // Log the ADC & TEMP values....
    let isense_counts = adc::DMA_BUF[0].read() as i32;
    let vsense_counts = adc::DMA_BUF[1].read() as u32;
    let v3v3_counts   = adc::DMA_BUF[2].read() as u32;
    let temp_counts   = i16::from_be(TEMP.read()) as i32;

    let isense = ((isense_counts - 2034 * 16) * 25000 + 32768) >> 16;
    let vsense = vsense_counts * (3300 * 118 / 24) / (65536 * 18 / 24);
    let v3v3;
    if v3v3_counts >= 45000 && v3v3_counts <= 55000 {
        v3v3 = 65536 * 2500 / v3v3_counts;
    }
    else {
        v3v3 = 3300;
    }

    let noise = unsafe {NOISE.as_mut()};
    noise[0].update(vsense);
    noise[1].update(isense as u32);
    noise[2].update(v3v3);
    noise[3].update(temp_counts as u32);

    // sdbg!("?");

    dbgln!("{vsense:6}mV{isense:6}mA{v3v3:6}mV{:6} d°C{:7} m°C {} {} {} {}",
           (temp_counts * 5 + 32) >> 6, temp_counts * 1000 >> 7,
           noise[0].decimal(), noise[1].decimal(),
           noise[2].decimal(), noise[3].decimal());

    // sdbg!("!");
}

pub fn main() -> ! {
    let gpioa = unsafe {&*stm32u031::GPIOA ::ptr()};
    let pwr   = unsafe {&*stm32u031::PWR   ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC   ::ptr()};
    let usart = unsafe {&*stm32u031::USART2::ptr()};
    let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    let nvic  = unsafe {&*cortex_m::peripheral::NVIC::PTR};

    // Go to 16MHz.
    rcc.CR.write(
        |w| w.MSIRANGE().B_0x8().MSIRGSEL().set_bit().MSION().set_bit());

    if !cfg!(test) {
        let bss_start = &raw mut __bss_start;
        let bss_end   = &raw mut __bss_end;
        let bss_size = bss_end.addr() - bss_start.addr();
        unsafe {
            core::ptr::write_bytes(&raw mut __bss_start, 0u8, bss_size);
        }
    }

    // Enable clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit());
    rcc.AHBENR.write(|w| w.DMA1EN().set_bit());
    rcc.APBENR1.write(
        |w| w.USART2EN().set_bit().I2C1EN().set_bit()
            . PWREN().set_bit());
    rcc.APBENR2.write(|w| w.ADCEN().set_bit());

    // Set-up the UART TX.  TODO - we should enable RX at some point.  The dbg*
    // macros will work after this.
    usart.BRR.write(|w| unsafe {w.BRR().bits(139)});
    usart.CR1.write(
        |w| w.FIFOEN().set_bit().TE().set_bit().UE().set_bit());

    // Enable backup domain access.
    pwr.CR1.write(|w| w.DBP().set_bit());

    // Enable the LSE.  Set high drive strength for crystal start-up.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x3());

    // Pullups on I2C and USART RX pin.
    gpioa.PUPDR.write(
        |w| w.PUPD3().B_0x1().PUPD9().B_0x1().PUPD10().B_0x1());

    // Set I2C pins to open drain.
    gpioa.OTYPER.write(|w| w.OT9().set_bit().OT10().set_bit());

    // Configure pin functions.
    // PA2,3, USART2, AF7.
    // PA9,10, I2C1, SCL, SDA, AF4.
    gpioa.AFRL.write(|w| w.AFSEL2().B_0x7().AFSEL3().B_0x7());
    gpioa.AFRH.write(|w| w.AFSEL9().B_0x4().AFSEL10().B_0x4());

    gpioa.MODER.modify(
        |_, w| w
            .MODE2 ().B_0x2().MODE3 ().B_0x2()   // USART.
            .MODE9 ().B_0x2().MODE10().B_0x2()   // I2C.
            .MODE11().B_0x1().MODE12().B_0x1()); // GPO.

    debug::init();

    sdbgln!("Debug up");
    dbgln!("Async debug up");

    adc::init1();

    i2c::init();

    // FIXME get the set-up correct.

    // Set the systick interrupt priority to a high value (other interrupts
    // pre-empt).
    // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
    // address.  We want SHPR3.
    assert_eq!(&scb.shpr[1] as *const _ as usize, 0xE000ED20);
    #[cfg(not(test))]
    unsafe {
        scb.shpr[1].write(0xc0 << 24);
    }

    // Wait for LSE and then enable the MSI FLL.
    while !rcc.BDCR.read().LSERDY().bit() {
    }
    rcc.CR.write(
        |w| w.MSIRANGE().B_0x8().MSIRGSEL().set_bit().MSION().set_bit()
            . MSIPLLEN().set_bit());
    // Reduce drive strength.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x0());

    // Enable interrupts for I2C1, ADC and DMA.  FIXME - correct channels!
    use stm32u031::Interrupt::*;
    fn bit(i: stm32u031::Interrupt) -> u32 {1 << i as u16}
    unsafe {
        nvic.iser[0].write(
            bit(I2C1) | bit(ADC_COMP) | bit(USART2_LPUART2) |
            bit(DMA1_CHANNEL1) | bit(DMA1_CHANNEL2_3));
    }

    // systick counts at 16MHz / 8 = 2MHz, divide by 20 to give 100kHz for
    // now.
    dbgln!("Going!");

    oled::init();

    dbgln!("Oled init!");

    adc::init2();

    unsafe {
        let syst = &*cortex_m::peripheral::SYST::PTR;
        syst.rvr.write(199999);
        syst.cvr.write(199999);
        syst.csr.write(3);
    }

    loop {
        WFE();
    }
}

#[repr(C)]
pub struct VectorTable {
    stack     : u32,
    reset     : fn() -> !,
    nmi       : fn(),
    hard_fault: fn(),
    reserved1 : [u32; 7],
    svcall    : fn(),
    reserved2 : [u32; 2],
    pendsv    : fn(),
    systick   : fn(),
    interrupts: [fn(); 32],
}

#[used]
#[unsafe(link_section = ".vectors")]
pub static VTORS: VectorTable = VectorTable {
    stack     : 0x20000000 + 12 * 1024,
    reset     : main,
    nmi       : bugger,
    hard_fault: bugger,
    reserved1 : [0; 7],
    svcall    : bugger,
    reserved2 : [0; 2],
    pendsv    : bugger,
    systick   : systick_handler,
    interrupts: [
        bugger, bugger, bugger, bugger, bugger, bugger, bugger, bugger,
        bugger, dma1_isr, i2c::dma23_isr, bugger,
        adc_isr, bugger, bugger, bugger,
        bugger, bugger, bugger, bugger, bugger, bugger, bugger, i2c::i2c_isr,
        bugger, bugger, bugger, bugger,
        debug::debug_isr, bugger, bugger, bugger,
    ],
};

fn bugger() {
    panic!("Unexpected interrupt");
}

#[test]
fn check_vtors() {
    use stm32u031::Interrupt::*;

    assert!(std::ptr::fn_addr_eq(VTORS.reset, main as fn()->!));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[ADC_COMP as usize],
                                 adc_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[DMA1_CHANNEL1 as usize],
                                 dma1_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[DMA1_CHANNEL2_3 as usize],
                                 i2c::dma23_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[I2C1 as usize],
                                 i2c::i2c_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[USART2_LPUART2 as usize],
                                 debug::debug_isr as fn()));
}
