#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]
#![feature(str_from_raw_parts)]
#![feature(trait_alias)]

use adc::{adc_isr, dma1_isr};
use noise::Noise;
use static_assertions::const_assert;
use vcell::{UCell, WFE};

mod adc;
mod decimal;
mod dma;
mod debug;
mod font;
mod i2c;
mod noise;
mod oled;
mod usqrt;
mod vcell;

type I2C = stm32u031::I2C1;

static TEMP: UCell<i16> = UCell::new(0);
static NOISE: UCell<[Noise; 4]> = UCell::new([Noise::new(); 4]);

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

fn systick_handler() {
    let _ = do_systick_handler();
}

fn do_systick_handler() -> Result<(), ()> {
    //sdbgln!("SYSTICK enter");

    adc::start();

    let temp = unsafe {TEMP.as_mut()};
    *temp = 0;
    let i2c_wait = i2c::read_reg(i2c::TMP117, 0, temp);

    // Wait for both I2C and ADC done.
    while !adc::DONE.read() {
        WFE();
    }

    drop(i2c_wait);

    // Log the ADC & TEMP values....
    let isense_counts = adc::DMA_BUF[0].read() as i32;
    let vsense_counts = adc::DMA_BUF[1].read() as u32;
    let v3v3_counts   = adc::DMA_BUF[2].read() as u32;
    let temp_counts   = i16::from_be(*temp) as i32;

    let isense_raw = isense_counts - 2034 * 16;
    let isense = (isense_raw * 25000 + 32768) >> 16;

    const V_FULL_SCALE: f64 = 118.0 / 18.0 * 3.3;
    const V_SHIFT: u32 = 11;
    const V_MULT: u32 = (V_FULL_SCALE * (1 << V_SHIFT) as f64 + 0.5) as u32;
    const_assert!(V_MULT >= 32768);
    const_assert!(V_MULT < 65536);
    let vsense = (vsense_counts * V_MULT + 32768) >> 16;

    const COUNTS_V3: u32 = (2.5 / 3.0 * 65536.0) as u32;
    const_assert!(65536 * 2500 / COUNTS_V3 >= 3000);
    const_assert!(65536 * 2500 / (COUNTS_V3 + 1) < 3000);
    let v3v3;
    if v3v3_counts > 45000 && v3v3_counts <= COUNTS_V3 {
        v3v3 = 65536 * 2500 / v3v3_counts;
    }
    else {
        v3v3 = 3300;
    }
    //let v3v3 = v3v3::v3v3_est(v3v3_counts);

    // u32::MAX is 270W.
    // 65536 is 4mW, resolution on (v_counts * i_counts) is ≈ 0.1µW.
    // Drop 12 bits, this gives us resolution of around 0.5mW.
    #[allow(non_upper_case_globals)]
    const cW_PER_COUNT: f64 = 118.0 / 18.0 * 3.3 / 65536.0 // Voltage
        * 25.0 / 65536.0                                   // Current
        * 100.0;
    const POWER_SCALE: f64 = 65536.0 * 65536.0 * cW_PER_COUNT;
    const MULT: u32 = POWER_SCALE as u32;
    const_assert!(MULT < 65536 && MULT > 50000);
    let power_counts = vsense_counts * isense_raw.unsigned_abs();
    // Power in centiwatts * 65536.
    let power = (power_counts >> 16) * MULT
        + ((power_counts & 0xffff) * MULT + 32768 >> 16);
    // Power in cW.
    let power = power >> 16;

    let noise = unsafe {NOISE.as_mut()};
    noise[0].update(vsense);
    noise[1].update(isense as u32);
    noise[2].update(v3v3);
    noise[3].update(temp_counts as u32);

    // sdbg!("?");
    //   VV.VVV V
    //  ±II.III A
    // n.nnn -nn.n
    dbgln!("{vsense:6}mV{isense:6}mA{v3v3:6}mV{:7} m°C {} {} {} {}",
           temp_counts * 1000 >> 7,
           noise[0].decimal(), noise[1].decimal(),
           noise[2].decimal(), noise[3].decimal());

    static FRAME: UCell<[oled::Line; 4]> = UCell::new([[0; _]; _]);
    let frame = unsafe{FRAME.as_mut()};
    let mut line = [b' '; 12];

    line[8..].copy_from_slice(b" F  "); // F -> V
    decimal::format_u32(&mut line[..8], vsense, 3);
    oled::refresh_line(&mut frame[0], line[..].try_into().unwrap(), 0)?;

    line[8..].copy_from_slice(b" A  ");
    if isense > 0 {
        line[..2].copy_from_slice(b"<=");
    }
    if isense < 0 {
        line[10..].copy_from_slice(b"?>");
    }
    let usense = isense.unsigned_abs();
    decimal::format_u32(&mut line[2..8], usense, 3);
    oled::refresh_line(&mut frame[1], line[..].try_into().unwrap(), 2)?;

    line[7..].copy_from_slice(b"  l  ");
    decimal::format_u32(&mut line[..7], power, 2);
    oled::refresh_line(&mut frame[2], line[..].try_into().unwrap(), 4)?;
    
    let centic = (temp_counts * 100 + 50) >> 7;
    decimal::format_u32(&mut line[..4], v3v3, 0);
    line[4] = b'j';
    decimal::format_i32(&mut line[5..11], centic, false, 2);
    line[11..].copy_from_slice(b"o");

    oled::refresh_line(&mut frame[3], &line, 6)?;

    static CYCLE: UCell<u8> = UCell::new(0);
    let cycle = unsafe{CYCLE.as_mut()};
    if *cycle < 4 {
        let content = frame[*cycle as usize];
        frame[*cycle as usize] = [0; _];
        oled::refresh_line(&mut frame[*cycle as usize], &content, *cycle * 2)?;
        *cycle += 1;
    }
    else {
        oled::reset()?;
        *cycle = 0;
    }
    
    Ok(())
}

pub fn main() -> ! {
    let gpioa = unsafe {&*stm32u031::GPIOA ::ptr()};
    let pwr   = unsafe {&*stm32u031::PWR   ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC   ::ptr()};
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

    // Poll for the I2C lines to rise.  They may take a while if we are relying
    // on only the internal pull-ups.
    while gpioa.IDR.read().bits() & 0x600 != 0x600 {
    }
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

    let _ = oled::init();

    dbgln!("Oled init!");

    adc::init2();

    unsafe {
        let syst = &*cortex_m::peripheral::SYST::PTR;
        syst.rvr.write(399999);
        syst.cvr.write(399999);
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
