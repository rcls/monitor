#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]
#![allow(unpredictable_function_pointer_comparisons)]

mod cpu;
mod debug;
mod dma;
mod i2c;
mod lcd;
mod vcell;

use vcell::{UCell, VCell, WFE};

type I2C = stm32u031::I2C1;
const CPU_CLK: u32 = 2000000;

type LcdBits = u64;
const LCD_BITS: u32 = 48;

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

fn scrounge() -> i2c::Result {
    let mut temp: i16 = 0;
    i2c::read_reg(i2c::TMP117, 0, &mut temp).wait()?;
    i2c::write(123, &[1u8,2]).wait()?;
    i2c::waiter(&mut temp).wait()
}

fn systick_handler() {
    static I: UCell<u32> = UCell::new(0);

    let i = unsafe{I.as_mut()};
    *i = i.wrapping_add(1);

    let lcd = unsafe{lcd::LCD.as_mut()};
    if *i & 7 == 0 {
        advance(lcd, *i >> 3);
    }
    lcd.tick();

    if *i & 15 == 0 {
        tsc_start();
    }
}

fn advance(lcd: &mut lcd::LCD, num: u32) {
    let mut segments = (lcd::DOT as u64) << 8;
    let mut temp: i16 = 0;
    let _ = i2c::read_reg(i2c::TMP117, 0, &mut temp);

    let temp = i16::from_be(temp);

    let mut n = 0;
    let mut abs: usize = temp.abs() as usize * 10 / 128;
    while n < 24 || abs != 0 {
        segments |= (lcd::DIGITS[abs % 10] as u64) << n;
        n += 8;
        abs = abs / 10;
        if abs == 0 {
            break;
        }
    }
    if temp < 0 {
        segments |= (lcd::MINUS as u64) << n;
    }

    segments |= 255 << 32;
    let rot = num % 6 * 8;
    segments = (segments << rot) & 0xffffffffffff | segments >> (48 - rot);

    lcd.segments = segments;
}

static TSC_BUSY: VCell<u8> = VCell::new(0);

fn tsc_isr() {
    let tsc   = unsafe {&*stm32u031::TSC::ptr()};
    let sr = tsc.ISR.read().bits();
    tsc.ICR.write(|w| w.bits(sr));
    let ccr = tsc.IOCCR.read();
    let gsr = tsc.IOGCSR().read().bits();
    let val;
    let ch;
    if ccr.G7_IO3().bit() {
        ch = '+';
        val = tsc.IOG7CR.read().bits();
        tsc.IOCCR.write(|w| w.G7_IO1().set_bit());
        tsc.IOSCR.write(|w| w.G7_IO2().set_bit());
        tsc.IOGCSR.write(|w| w.G7E().set_bit());
    }
    else if ccr.G7_IO1().bit() {
        ch = '-';
        val = tsc.IOG7CR.read().bits();
        tsc.IOCCR.write(|w| w.G5_IO4().set_bit());
        tsc.IOSCR.write(|w| w.G5_IO2().set_bit());
        tsc.IOGCSR.write(|w| w.G5E().set_bit());
    }
    else {
        ch = 'O';
        val = tsc.IOG5CR.read().bits();
        tsc.IOCCR.write(|w| w.G7_IO3().set_bit());
        tsc.IOSCR.write(|w| w.G7_IO2().set_bit());
        tsc.IOGCSR.write(|w| w.G7E().set_bit());
    }

    // Log some results...
    dbgln!("CR {:#x} SR {:#x} GSR {gsr:#08x} {ch} {val}",
           tsc.CR.read().bits(), sr);
    TSC_BUSY.write(0);
}

fn tsc_start() {
    let tsc   = unsafe {&*stm32u031::TSC  ::ptr()};
    if TSC_BUSY.read() == 0 {
        TSC_BUSY.write(1);
        tsc.CR.modify(|_,w| w.START().set_bit());
    }
}

fn main() -> ! {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    let tsc   = unsafe {&*stm32u031::TSC  ::ptr()};

    //let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    let nvic  = unsafe {&*cortex_m::peripheral::NVIC::PTR};

    // Enable clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit().GPIOBEN().set_bit());
    rcc.AHBENR.write(|w| w.DMA1EN().set_bit().TSCEN().set_bit());
    rcc.APBENR1.write(
        |w| w.I2C1EN().set_bit().PWREN().set_bit().LPUART1EN().set_bit());
    rcc.APBENR2.write(|w| w.SPI1EN().set_bit());

    cpu::init1();

    // Pullups on UART RX pin.
    gpioa.PUPDR.write(|w| w.PUPD3().B_0x1());

    debug::init();

    sdbgln!("Debug up");

    // Configure I2C (B6, B7) lines and wait for them to rise.
    gpiob.PUPDR.write(|w| w.PUPD6().B_0x1().PUPD7().B_0x1());
    gpiob.AFRL.write(|w| w.AFSEL6().B_0x4().AFSEL7().B_0x4());
    gpiob.MODER.modify(|_, w| w.MODE6().B_0x2().MODE7().B_0x2());

    lcd::init();

    sdbgln!("I2C line wait");

    // Poll for the I2C lines to rise.  They may take a while if we are relying
    // on only the internal pull-ups.
    while gpiob.IDR.read().bits() & 0xc0 != 0xc0 {
    }
    sdbgln!("I2C line wait done; i2c init");

    i2c::init();

    sdbgln!("CPU init2");
    cpu::init2();

    // Enable interrupts.
    use stm32u031::Interrupt::*;
    fn bit(i: stm32u031::Interrupt) -> u32 {1 << i as u16}
    unsafe {
        nvic.iser[0].write(
            bit(I2C1) | bit(ADC_COMP) | bit(debug::UART_ISR) |
            bit(DMA1_CHANNEL2_3) | bit(DMA1_CHANNEL4_5_6_7) | bit(TSC));
    }

    // TSC has
    // Plus: G7_IO1 (A8)
    // Cap : G7_IO2 (A9)
    // Minus: G7_IO3 (A10)
    // Menu: G5_IO4 (PB11)
    // Cap:  G5_IO2 (PB0)

    // TODO - do we do this or just leave pins at analog?
    gpioa.AFRH.modify(
        |_,w| w.AFSEL8().B_0x9().AFSEL9().B_0x9().AFSEL10().B_0x9());
    gpiob.AFRL.modify(|_,w| w.AFSEL0().B_0x9());
    gpiob.AFRH.modify(|_,w| w.AFSEL11().B_0x9());
    // Set open drain mode for the sampling cap pins.
    gpioa.OTYPER.write(|w| w.OT9().B_0x1());
    gpiob.OTYPER.write(|w| w.OT0().B_0x1());
    //gpioa.OTYPER.write(|w| w.OT8().B_0x1().OT9().B_0x1().OT10().B_0x1());
    //gpiob.OTYPER.write(|w| w.OT0().B_0x1().OT11().B_0x1());
    // Enable AF.
    gpioa.MODER.modify(|_,w| w.MODE8().B_0x2().MODE9().B_0x2().MODE10().B_0x2());
    gpiob.MODER.modify(|_,w| w.MODE0().B_0x2().MODE11().B_0x2());

    // TODO - timings are guesses...
    // Docs talk about frequency 250kHz = 4µs = 64 clocks.
    // Standard pulse range "500ns to 2µs".
    // Use 2µs pulses for now.
    // /16 prescaler.
    // Max counts is set as high as possible.
    tsc.CR.write(
        |w| w.CTPH().bits(4).CTPL().bits(4)
            . SSD().bits(1). SSE().set_bit().SSPSC().set_bit()
            . PGPSC().B_0x5().MCV().B_0x6().TSCE().set_bit());
    // Hysterysis & analog control?
    // tsc.IOHCR.write(
    //     |w| w.G7_IO1().set_bit().G7_IO2().set_bit().G7_IO3().set_bit()
    //         . G5_IO2().set_bit().G5_IO4().set_bit());
    // tsc.IOASCR.write(
    //     |w| w.G7_IO1().set_bit().G7_IO2().set_bit().G7_IO3().set_bit()
    //         . G5_IO2().set_bit().G5_IO4().set_bit());
    // Sampling caps.
    tsc.IOSCR().write(|w| w.G7_IO2().set_bit().G5_IO2().set_bit());
    // Channels.  TODO - do we just leave all set, or do we toggle?
    tsc.IOCCR.write(
        |w| w.G7_IO1().set_bit().G7_IO3().set_bit().G5_IO4().set_bit());
    // Use groups 5 and 7.
    tsc.IOGCSR.write(|w| w.G5E().set_bit().G7E().set_bit());
    // Enable end-of-acquire interrupt.
    tsc.IER.write(|w| w.EOAIE().set_bit());

    if false {
        let _ = scrounge();
    }

    sdbgln!("Ok, should be flying!");
    dbgln!("Going!");

    // systick counts at 16MHz / 8 = 2MHz.
    unsafe {
        let syst = &*cortex_m::peripheral::SYST::PTR;
        syst.rvr.write(CPU_CLK / 8 / 25 - 1);
        syst.cvr.write(CPU_CLK / 8 / 25 - 1);
        syst.csr.write(3);
    }

    // Reduce priority of the TSC interrupt.
    #[cfg(not(test))]
    unsafe {
        assert_eq!(&nvic.ipr[5] as *const _ as usize, 0xE000E400 + 20);
        //nvic.ipr[21].write(0xc0);
        nvic.ipr[5].write(0xc0 << 8);
    }

    loop {
        WFE();
    }
}

#[used]
#[unsafe(link_section = ".vectors")]
static VECTORS: cpu::VectorTable = *cpu::VectorTable::new()
    .systick(systick_handler)
    .isr(stm32u031::Interrupt::TSC, tsc_isr)
    .debug_isr().i2c_isr().lcd_isr();

#[test]
fn check_vtors() {
    use stm32u031::Interrupt::*;

    assert!(std::ptr::fn_addr_eq(VECTORS.reset, main as fn()->!));
    assert!(VECTORS.systick == systick_handler);
    assert!(std::ptr::fn_addr_eq(VECTORS.isr[TSC as usize], tsc_isr as fn()));
}
