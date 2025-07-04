#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]

mod cpu;
mod debug;
mod dma;
mod i2c;
mod lcd;
mod vcell;

use vcell::{UCell, WFE};

type I2C = stm32u031::I2C1;

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
}

fn advance(lcd: &mut lcd::LCD, num: u32) {

    let mut segments = (lcd::DOT as u64) << 16;
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

fn main() -> ! {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};

    //let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    let nvic  = unsafe {&*cortex_m::peripheral::NVIC::PTR};

    // Enable clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit().GPIOBEN().set_bit());
    rcc.AHBENR.write(|w| w.DMA1EN().set_bit());
    rcc.APBENR1.write(
        |w| w.USART2EN().set_bit().I2C1EN().set_bit()
            . PWREN().set_bit());
    rcc.APBENR2.write(|w| w.SPI1EN().set_bit());

    cpu::init1();

    // Pullups on USART RX pin.
    gpioa.PUPDR.write(|w| w.PUPD3().B_0x1());

    // Configure UART lines.  sdbg! should now work.
    gpioa.AFRL.write(|w| w.AFSEL2().B_0x7().AFSEL3().B_0x7());
    gpioa.MODER.modify(|_, w| w.MODE2().B_0x2().MODE3().B_0x2()); // USART.

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
            bit(I2C1) | bit(ADC_COMP) | bit(USART2_LPUART2) |
            bit(DMA1_CHANNEL2_3));
    }

    if false {
        let _ = scrounge();
    }

    sdbgln!("Ok, should be flying!");
    dbgln!("Going!");

    // systick counts at 16MHz / 8 = 2MHz.
    unsafe {
        let syst = &*cortex_m::peripheral::SYST::PTR;
        syst.rvr.write(79999); // 2MHz / 80000 = 25Hz
        syst.cvr.write(79999);
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
        bugger, bugger, i2c::dma23_isr, bugger,
        bugger, bugger, bugger, bugger,
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
    //assert!(std::ptr::fn_addr_eq(VTORS.interrupts[ADC_COMP as usize],
    //                             adc_isr as fn()));
    //assert!(std::ptr::fn_addr_eq(VTORS.interrupts[DMA1_CHANNEL1 as usize],
    //                             dma1_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[DMA1_CHANNEL2_3 as usize],
                                 i2c::dma23_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[I2C1 as usize],
                                 i2c::i2c_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[USART2_LPUART2 as usize],
                                 debug::debug_isr as fn()));
}
