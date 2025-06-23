#![no_std]
#![no_main]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]

use crate::vcell::VCell;

mod debug;
mod vcell;

// DMA MUX lines.
// Use DMA1 Ch1
const ADC_MUXIN: u32 = 5;
// Use DMA1 Ch2
const I2C_RX_MUXIN: u32 = 9;
// Use DMA1 Ch3
const I2C_TX_MUXIN: u32 = 10;

const TMP117: u16 = 0x92;

static ADC_DMA_BUF: [VCell<u16>; 3] = [const {VCell::new(0)}; 3];

static TEMP: VCell<i16> = VCell::new(0);
static I2C_DONE: VCell<bool> = VCell::new(false);
static ADC_DONE: VCell<bool> = VCell::new(false);

fn adc_isr() {
    let adc = unsafe {&*stm32u031::ADC ::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    // Get EOC and OVR bits.
    let isr = adc.ISR.read().bits();
    adc.ISR.write(|w| unsafe{w.bits(isr & 0x18)});
    // dbgln!("ADC ISR {isr:#x}");
    // FIXME - abort if OVR is set.
    if dma.CNDTR1.read().NDT().bits() == 0 {
        // dbgln!("ADC done");
        ADC_DONE.write(true);
    }
    // dbgln!("{:#x} {:#x} {:#x} {:#x} {:#x}",
    //        dma.ISR.read().bits(), dma.CCR1.read().bits(),
    //        dma.CNDTR1.read().bits(), dma.CPAR1.read().bits(),
    //        dma.CMAR1.read().bits());
    // let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    // dbgln!(" {:#x}", dmamux.C0CR.read().bits());
}

fn i2c_isr() {
    // let i2c = unsafe {&*stm32u031::I2C1 ::ptr()};
    // Don't handle TC here, just errors!
    // let status = i2c.ISR.read().bits();
    // dbgln!("I2C ISR {status:#x}");
}

fn dma1_isr() {
    let adc = unsafe {&*stm32u031::ADC::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    let status = dma.ISR.read().bits();
    dma.IFCR.write(|w| w.CGIF1().set_bit());
    // dbgln!("DMA1 ISR {status:#x}");
    if status & 10 != 0 && adc.CR.read().ADSTART().bit() {
        // dbgln!("ADC DONE");
        ADC_DONE.write(true);
    }
}

fn dma23_isr() {
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    // I2C TX or RX.
    // FIXME - do we need to wait for TC when appropriate?
    let status = dma.ISR.read().bits();
    dma.IFCR.write(|w| w.CGIF2().set_bit().CGIF3().set_bit());
    // TODO - definitive way to wait for I2C done?
    // dbgln!("DMA23 ISR {status:#x}");
    if status & 0xaa0 != 0 {
        // dbgln!("I2C DONE");
        I2C_DONE.write(true);
    }
}

fn systick_handler() {
    // dbgln!("SYSTICK enter");
    let adc   = unsafe {&*stm32u031::ADC  ::ptr()};
    let dma   = unsafe {&*stm32u031::DMA1 ::ptr()};
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let i2c   = unsafe {&*stm32u031::I2C1 ::ptr()};

    // Gray code blink on bits 11, 12 for now.
    let bsrr = match gpioa.ODR.read().bits() & 0x1800 {
        0x0000 => 0x0800,
        0x0800 => 0x1000,
        0x1800 => 0x08000000,
        _      => 0x10000000,
    };
    gpioa.BSRR.write(|w| unsafe{w.bits(bsrr)});

    ADC_DONE.write(false);
    I2C_DONE.write(false);

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
    static ZERO: u16 = 0;
    // FIXME - what needs to be done every time?
    dma.CCR3.write(|w| unsafe {w.bits(0)});
    dma.CMAR3.write(|w| unsafe {w.bits(&ZERO as *const _ as u32)});
    dma.CPAR3.write(|w| unsafe {w.bits(i2c.TXDR.as_ptr() as u32)});
    dma.CNDTR3.write(|w| unsafe {w.bits(1)});
    // dbgln!("DMA CCR3 {:#x}", dma.CCR3.read().bits());
    dma.CCR3.write(
        |w| w.EN().set_bit().TCIE().set_bit().TEIE().set_bit().MINC().set_bit()
            .DIR().set_bit());
    i2c.CR2.write(unsafe{
        |w| w.START().set_bit().SADD().bits(TMP117).NBYTES().bits(1)});

    //dbgln!("I2C wait 1");
    // FIXME - what happens on errors?
    if true {
        while !I2C_DONE.read() {
            cortex_m::asm::wfe();
        }
    }
    if false {
        let mut i = 0;
        loop {
            if I2C_DONE.read() {
                break;
            }
            i += 1;
            if i == 1048576 {
                i = 0;
                dbgln!("? {:#x} {:#x} {:#x}",
                       dma.ISR.read().bits(),
                       dma.CNDTR3.read().bits(),
                       dma.CCR3.read().bits());
            }
        }
    }
    //dbgln!("I2C wait 1 done");
    //while !i2c.ISR.read().TC().bit() {
    //}
    //i2c.ISR.write(|w| w.TC().set_bit());

    I2C_DONE.write(false);
    // Start the I2C read.
    dma.CCR2.write(|w| unsafe {w.bits(0)});
    dma.CMAR2.write(|w| unsafe {w.bits(&TEMP as *const _ as u32)});
    dma.CPAR2.write(|w| unsafe {w.bits(&i2c.RXDR as *const _ as u32)});
    dma.CNDTR2.write(|w| unsafe {w.bits(2)});
    dma.CCR2.write(
        |w| w.EN().set_bit().TCIE().set_bit().TEIE().set_bit().MINC().set_bit());
    i2c.CR2.write(unsafe{
        |w| w.START().set_bit().SADD().bits(TMP117).NBYTES().bits(2)
            .AUTOEND().set_bit().RD_WRN().set_bit()});

    // Wait for both I2C and ADC done.
    while !I2C_DONE.read() || !ADC_DONE.read() {
        cortex_m::asm::wfe();
    }

    // Log the ADC & TEMP values....
    let isense
        = ((ADC_DMA_BUF[0].read() as i32 - 2034 * 16) * 25000 + 32768) >> 16;
    let vsense = ADC_DMA_BUF[1].read() as u32 * (3300 * 118 / 24)
        / (65536 * 18 / 24);
    let v3v3 = 65536 * 2500 / ADC_DMA_BUF[2].read() as u32;

    let temp = i16::from_be(TEMP.read()) as i32;
    dbgln!("! {vsense}mV ({}) {isense}mA {v3v3}mV {} d°C {} m°C",
           ADC_DMA_BUF[1].read(),
           //(i16::from_be(TEMP.read()) as i32 * 5 + 32) >> 6);
           (temp * 5 + 32) >> 6, temp * 1000 >> 7);
    // dbgln!("SYSTICK exit");
}

#[inline]
fn nothing() {
    unsafe {core::arch::asm!("", options(nomem))}
}

pub fn entry() -> ! {
    let adc    = unsafe {&*stm32u031::ADC   ::ptr()};
    //let dma    = unsafe {&*stm32u031::DMA1  ::ptr()};
    let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    let gpioa  = unsafe {&*stm32u031::GPIOA ::ptr()};
    let i2c    = unsafe {&*stm32u031::I2C1  ::ptr()};
    let pwr    = unsafe {&*stm32u031::PWR   ::ptr()};
    let rcc    = unsafe {&*stm32u031::RCC   ::ptr()};
    let usart  = unsafe {&*stm32u031::USART2::ptr()};
    let scb    = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    let nvic   = unsafe {&*cortex_m::peripheral::NVIC::PTR};

    // Go to 16MHz.
    rcc.CR.write(
        |w| w.MSIRANGE().B_0x8().MSIRGSEL().set_bit().MSION().set_bit());

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

    // Calibrate the ADC.  First enable the voltage reg.
    adc.CR.write(|w| w.ADVREGEN().set_bit());
    // Wait for LDO ready... 20µs.
    for _ in 0..200 {
        nothing();
    }
    // Start the calibration.
    adc.CR.write(|w| w.ADCAL().set_bit().ADVREGEN().set_bit());

    // Configure I2C.
    // 50kb/s from 16MHz.
    #[cfg(true)]
    i2c.TIMINGR.write(unsafe{
        |w| w.PRESC().bits(7)
            . SCLL().bits(19)
            . SCLH().bits(15)
            . SDADEL().bits(2)
            . SCLDEL().bits(4)});
    // 400kb/s from 16MHz, STM documented timings.
    #[cfg(false)]
    i2c.TIMINGR.write(unsafe{
        |w| w.PRESC().bits(1)
            . SCLL().bits(9)
            . SCLH().bits(3)
            . SDADEL().bits(2)
            . SCLDEL().bits(3)});
    // 400kb/s from 16MHz, longer clock pulse.
    #[cfg(false)]
    i2c.TIMINGR.write(unsafe{
        |w| w.PRESC().bits(1)
            . SCLL().bits(3)
            . SCLH().bits(9)
            . SDADEL().bits(1)
            . SCLDEL().bits(1)});

    // Enable everything.
    i2c.CR1.write(
        |w| w.TXDMAEN().set_bit().RXDMAEN().set_bit().PE().set_bit()
            . NACKIE().set_bit().ERRIE().set_bit());

    // DMAMUX mapping.  Note the zero-based v. one-based f**kup.
    // ADC to DMA1 channel 1,
    dmamux.C0CR.write(|w| unsafe {w.bits(ADC_MUXIN)});
    // I2C1_RX to DMA2 channel 2.
    dmamux.C1CR.write(|w| unsafe {w.bits(I2C_RX_MUXIN)});
    // I2C1_TX to DMA2 channel 3.
    dmamux.C2CR.write(|w| unsafe {w.bits(I2C_TX_MUXIN)});

    // FIXME get the set-up correct.
    //dma.CPAR1.write(|w| unsafe {w.bits(&adc.DR as *const _ as u32)});
    //dma.CMAR1.write(|w| unsafe {w.bits(&ADC_DMA_BUF as *const _ as u32)});

    // Set the systick interrupt priority to a high value (other interrupts
    // pre-empt).
    // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
    // address.  We want SHPR3.
    assert_eq!(&scb.shpr[1] as *const _ as usize, 0xE000ED20);
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
    unsafe {
        use stm32u031::Interrupt::*;
        nvic.iser[0].write(
            1 << I2C1 as u16 | 1 << ADC_COMP as u16
                | 1 << DMA1_CHANNEL1 as u16 | 1 << DMA1_CHANNEL2_3 as u16);
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
    reset     : entry,
    nmi       : bugger,
    hard_fault: bugger,
    reserved1 : [0; 7],
    svcall    : bugger,
    reserved2 : [0; 2],
    pendsv    : bugger,
    systick   : systick_handler,
    interrupts: [
        bugger, bugger, bugger, bugger, bugger, bugger, bugger, bugger,
        bugger, dma1_isr, dma23_isr, bugger, adc_isr, bugger, bugger, bugger,
        bugger, bugger, bugger, bugger, bugger, bugger, bugger, i2c_isr,
        bugger, bugger, bugger, bugger, bugger, bugger, bugger, bugger,
    ],
};

fn bugger() {
    panic!("Unexpected interrupt");
}
