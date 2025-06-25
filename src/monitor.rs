#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]
#![feature(str_from_raw_parts)]

use crate::vcell::{UCell, VCell};
use crate::noise::Noise;

mod debug;
mod i2c;
mod noise;
mod usqrt;
mod vcell;

type I2C = stm32u031::I2C1;

// DMA MUX lines.
// Use DMA1 Ch1
const ADC_MUXIN: u32 = 5;

static ADC_DMA_BUF: [VCell<u16>; 3] = [const {VCell::new(0)}; 3];

static TEMP: VCell<i16> = VCell::new(0);
static ADC_DONE: VCell<bool> = VCell::new(false);
static NOISE: UCell<[Noise; 4]> = UCell::new([Noise::new(); 4]);

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

fn adc_isr() {
    let adc = unsafe {&*stm32u031::ADC ::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    // Get EOC and OVR bits.
    let isr = adc.ISR.read().bits();
    adc.ISR.write(|w| unsafe{w.bits(isr & 0x18)});
    // sdbgln!("ADC ISR {isr:#x}");
    // FIXME - abort if OVR is set.
    if dma.CNDTR1.read().NDT().bits() == 0 {
        // sdbgln!("ADC done");
        ADC_DONE.write(true);
    }
    // sdbgln!("{:#x} {:#x} {:#x} {:#x} {:#x}",
    //        dma.ISR.read().bits(), dma.CCR1.read().bits(),
    //        dma.CNDTR1.read().bits(), dma.CPAR1.read().bits(),
    //        dma.CMAR1.read().bits());
    // let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    // sdbgln!(" {:#x}", dmamux.C0CR.read().bits());
}

fn dma1_isr() {
    let adc = unsafe {&*stm32u031::ADC::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    let status = dma.ISR.read().bits();
    dma.IFCR.write(|w| w.CGIF1().set_bit());
    // sdbgln!("DMA1 ISR {status:#x}");
    if status & 10 != 0 && adc.CR.read().ADSTART().bit() {
        // sdbgln!("ADC DONE");
        ADC_DONE.write(true);
    }
}

fn systick_handler() {
    // sdbgln!("SYSTICK enter");
    let adc = unsafe {&*stm32u031::ADC ::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};

    ADC_DONE.write(false);

    // Set-up DMA for the ADC.
    // FIXME - do we need to do this every time?
    dma.CCR1.write(|w| unsafe {w.bits(0)});
    dma.CMAR1.write(|w| unsafe {w.bits(ADC_DMA_BUF.as_ptr() as u32)});
    dma.CPAR1.write(|w| unsafe {w.bits(adc.DR.as_ptr() as u32)});
    dma.CNDTR1.write(|w| unsafe {w.bits(3)});
    dma.CCR1.write(
        |w| w.EN().set_bit().TCIE().set_bit().TEIE().set_bit().MINC().set_bit()
            . MSIZE().B_0x1().PSIZE().B_0x1());

    // Trigger ADC conversions and temperature conversion (via I2C).
    adc.CR.write(|w| w.ADSTART().set_bit());

    // Start the I2C transfer.  Use DMA3.
    static ZERO: [u8; 1] = [0];
    i2c::write_start(i2c::TMP117, &ZERO);

    //dbgln!("I2C wait 1");
    // FIXME - what happens on errors?
    i2c::write_wait();

    i2c::read_start(i2c::TMP117, TEMP.as_ptr());

    // Wait for both I2C and ADC done.
    while !i2c::I2C_DONE.read() || !ADC_DONE.read() {
        cortex_m::asm::wfe();
    }

    // Log the ADC & TEMP values....
    let isense_counts = ADC_DMA_BUF[0].read() as i32;
    let vsense_counts = ADC_DMA_BUF[1].read() as u32;
    let v3v3_counts   = ADC_DMA_BUF[2].read() as u32;
    let temp_counts   = i16::from_be(TEMP.read()) as i32;

    let isense = ((isense_counts - 2034 * 16) * 25000 + 32768) >> 16;
    let vsense = vsense_counts * (3300 * 118 / 24) / (65536 * 18 / 24);
    let v3v3 = 65536 * 2500 / v3v3_counts;

    let noise = unsafe {NOISE.as_mut()};
    noise[0].update(vsense as u32);
    noise[1].update(isense as u32);
    noise[2].update(v3v3 as u32);
    noise[3].update(temp_counts as u32);

    // sdbg!("?");

    dbgln!("{vsense:6}mV{isense:6}mA{v3v3:6}mV{:6} d°C{:7} m°C {} {} {} {}",
           (temp_counts * 5 + 32) >> 6, temp_counts * 1000 >> 7,
           noise[0].decimal(), noise[1].decimal(),
           noise[2].decimal(), noise[3].decimal());

    // sdbg!("!");
}

#[inline]
fn nothing() {
    unsafe {core::arch::asm!("", options(nomem))}
}

pub fn main() -> ! {
    let bss_start = &raw mut __bss_start;
    let bss_end   = &raw mut __bss_end;
    let bss_size = bss_end.addr() - bss_start.addr();

    let adc    = unsafe {&*stm32u031::ADC   ::ptr()};
    let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    let gpioa  = unsafe {&*stm32u031::GPIOA ::ptr()};
    let pwr    = unsafe {&*stm32u031::PWR   ::ptr()};
    let rcc    = unsafe {&*stm32u031::RCC   ::ptr()};
    let usart  = unsafe {&*stm32u031::USART2::ptr()};
    let scb    = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    let nvic   = unsafe {&*cortex_m::peripheral::NVIC::PTR};

    // Go to 16MHz.
    rcc.CR.write(
        |w| w.MSIRANGE().B_0x8().MSIRGSEL().set_bit().MSION().set_bit());

    unsafe {
        core::ptr::write_bytes(&raw mut __bss_start, 0u8, bss_size);
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
    //dbgln!("BSS {bss_start:?} {bss_end:?} {bss_size}");
    sdbgln!("Debug up");
    dbgln!("Async debug up");

    // Calibrate the ADC.  First enable the voltage reg.
    adc.CR.write(|w| w.ADVREGEN().set_bit());
    // Wait for LDO ready... 20µs.
    for _ in 0..200 {
        nothing();
    }
    // Start the calibration.
    adc.CR.write(|w| w.ADCAL().set_bit().ADVREGEN().set_bit());

    i2c::init();

    // DMAMUX mapping.  Note the zero-based v. one-based f**kup.
    // ADC to DMA1 channel 1,
    dmamux.C0CR.write(|w| unsafe {w.bits(ADC_MUXIN)});

    // FIXME get the set-up correct.
    //dma.CPAR1.write(|w| unsafe {w.bits(&adc.DR as *const _ as u32)});
    //dma.CMAR1.write(|w| unsafe {w.bits(&ADC_DMA_BUF as *const _ as u32)});

    // Set the systick interrupt priority to a high value (other interrupts
    // pre-empt).
    // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
    // address.  We want SHPR3.
    assert_eq!(&scb.shpr[1] as *const _ as usize, 0xE000ED20);
    #[cfg(not(test))]
    unsafe {
        scb.shpr[1].write(0xc0 << 24);
    }

    // Wait for ADC calibration complete.
    dbgln!("Eocal wait");
    while !adc.ISR.read().EOCAL().bit() {
    }
    adc.ISR.write(|w| w.EOCAL().set_bit());
    dbgln!("Eocal done");
    dbgln!("ADC CR = {:#x}, ISR = {:#x}", adc.CR.read().bits(),
           adc.ISR.read().bits());

    // Enable the ADC and wait for ready...
    dbgln!("ADC CR = {:#x}, ISR = {:#x}", adc.CR.read().bits(),
           adc.ISR.read().bits());
    adc.CR.write(|w| w.ADVREGEN().set_bit().ADEN().set_bit());
    dbgln!("ADC CR = {:#x}, ISR = {:#x}", adc.CR.read().bits(),
           adc.ISR.read().bits());
    while !adc.ISR.read().ADRDY().bit() {
    }

    // Use DMAEN, and set AUTOFF for intermittent conversions.
    adc.CFGR1.write(|w| w.AUTOFF().set_bit().DMAEN().set_bit());
    // Oversample config.
    adc.CFGR2.write(|w| w.OVSS().B_0x4().OVSR().B_0x7().OVSE().set_bit());

    // Configure ADC chanels and wait for ready.  The datasheet appears to lie.
    // We actually get Isense on AIN9, VSENSE on A17 and VREF on A18 (0 based
    // v. 1 based?)
    adc.CHSELR.write(
        |w| w.CHSEL9().set_bit().CHSEL17().set_bit().CHSEL18().set_bit());
    // Wait for ready.
    dbgln!("Ccrdy wait");
    while !adc.ISR.read().CCRDY().bit() {
    }
    dbgln!("Ccrdy done");
    // Enable ADC EOC and OVR interrupts.
    adc.IER.write(|w| w.EOSIE().set_bit().OVRIE().set_bit());
    adc.SMPR.write(|w| w.SMP1().B_0x7()); // ≈10µs sample gate.

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

    unsafe {
        let syst = &*cortex_m::peripheral::SYST::PTR;
        syst.rvr.write(199999);
        syst.cvr.write(199999);
        syst.csr.write(3);
    }

    loop {
        cortex_m::asm::wfe();
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
