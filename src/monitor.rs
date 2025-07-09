#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]

use adc::{adc_isr, dma1_isr};
use noise::Noise;
use static_assertions::const_assert;
use vcell::{UCell, WFE};

mod adc;
mod cpu;
mod decimal;
mod dma;
mod debug;
mod font;
mod i2c;
mod noise;
mod oled;
mod usqrt;
mod utils;
mod vcell;

type I2C = stm32u031::I2C1;
const CPU_CLK: u32 = 16000000;

const ADC_CHANNELS: [u32; 3] = [9, 17, 18];

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

struct Monitor {
    divide: u8,
    cycle_refresh: u8,
    cycle_arrow: u8,
    temp: i16,
    isense: i32,
    noise: [Noise; 4],
    frame: [oled::Line; 4],
}

static MONITOR: UCell<Monitor> = UCell::new(Monitor::new());

fn systick_handler() {
    let _ = unsafe{MONITOR.as_mut().systick_handler()};
}

impl Monitor {
    const fn new() -> Monitor {
        Monitor{
            divide: 0,
            cycle_refresh: 0,
            cycle_arrow: 0,
            temp: 0,
            isense: 0,
            noise: [Noise::new(); _],
            frame: [[0; _]; _]
        }
    }

    fn systick_handler(&mut self) {
        // Update arrows every tick.
        let _ = self.update_arrows();
        let divide = if self.divide == 0 {4} else {self.divide - 1};
        self.divide = divide;
        if divide == 0 {
            adc::start();
        }
        else if adc::DONE.read() {
            adc::DONE.write(false);
            vcell::barrier();
            let _ = self.analog_update();
        }
        else if divide == 4 {
            let _ = self.refresh();
        }
    }

    fn analog_update(&mut self) -> i2c::Result {
        i2c::read_reg(i2c::TMP117, 0, &mut self.temp).wait()?;

        // Log the ADC & TEMP values....
        let isense_counts = adc::DMA_BUF[0].read() as i32;
        let vsense_counts = adc::DMA_BUF[1].read() as u32;
        let v3v3_counts   = adc::DMA_BUF[2].read() as u32;
        let temp_counts   = i16::from_be(self.temp) as i32;

        let isense_raw = isense_counts - 32646;
        self.isense = (isense_raw * 25000 + 32768) >> 16;

        let vsense = vconvert(vsense_counts);

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

        self.noise[0].update(vsense);
        self.noise[1].update(self.isense as u32);
        self.noise[2].update(v3v3);
        self.noise[3].update(temp_counts as u32);

        // sdbg!("?");
        //   VV.VVV V
        //  ±II.III A
        // n.nnn -nn.n
        if false {
            let isense =self.isense;
        dbgln!("{vsense:6}mV{isense:6}mA({isense_counts:5}){v3v3:6}mV{:7} m°C {} {} {} {}",
            temp_counts * 1000 >> 7,
            self.noise[0].decimal(), self.noise[1].decimal(),
            self.noise[2].decimal(), self.noise[3].decimal());
        }

        let mut line = [0; 10];
        decimal::format_u32(&mut line[..7], power, 2);
        line[7..].copy_from_slice(&CHARS_MAP!("  W"));
        oled::update_text(&mut self.frame[0], &line, 0, 0)?;

        let mut line = [0; 8];
        decimal::format_u32(&mut line[..6], vsense, 3);
        line[6..].copy_from_slice(&CHARS_MAP!(" V"));
        oled::update_text(&mut self.frame[1], &line, 2, 2)?;

        let usense = self.isense.unsigned_abs();
        decimal::format_u32(&mut line[..6], usense, 3);
        line[6..].copy_from_slice(&CHARS_MAP!(" A"));
        oled::update_text(&mut self.frame[2], &line, 2, 4)?;

        let mut line = [0; 5];
        let centic = (temp_counts * 100 + 50) >> 7;
        decimal::format_u32(&mut line[..4], v3v3, 0);
        line[4] = font::LETTER_m;
        oled::update_text(&mut self.frame[3], &line, 0, 6)?;

        let mut line = [0; 7];
        decimal::format_i32(&mut line[..6], centic, 2);
        line[6] = font::DEGREE; // °
        oled::update_text(&mut self.frame[3], &line, 5, 6)
    }

    fn refresh(&mut self) -> i2c::Result {
        if self.cycle_refresh < 4 {
            oled::draw_chars(&self.frame[self.cycle_refresh as usize],
                             0, self.cycle_refresh * 2)?;
            self.cycle_refresh += 1;
        }
        else {
            oled::reset()?;
            self.cycle_refresh = 0;
        }
        Ok(())
    }

    fn update_arrows(&mut self) -> i2c::Result {
        let cycle = self.cycle_arrow + 1 & 7;
        self.cycle_arrow = cycle;

        let mut base = font::SPACE;
        let mut inc  = 0;
        let mut dir  = 0;
        if self.isense > 0 {
            base = font::LEFT_TOP_0;
            inc = 1;
            dir = 2;
        }
        else if self.isense < 0 {
            base = font::RIGHT_TOP_0;
            inc = 1;
            dir = 6;
        }
        let i0 = base + inc * cycle;
        let i1 = base + inc * ((cycle + dir) & 7);
        let d = 8 * inc;
        let t = [i0, i1];
        let b = [i0 + d, i1 + d];

        oled::update_text(&mut self.frame[1], &t, 0, 2)?;
        oled::update_text(&mut self.frame[1], &t, 10, 2)?;
        oled::update_text(&mut self.frame[2], &b, 0, 4)?;
        oled::update_text(&mut self.frame[2], &b, 10, 4)
    }
}

pub fn main() -> ! {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    let nvic  = unsafe {&*cortex_m::peripheral::NVIC::PTR};

    // Enable clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit());
    rcc.AHBENR.write(|w| w.DMA1EN().set_bit().TSCEN().set_bit());
    rcc.APBENR1.write(
        |w| w.LPUART1EN().set_bit().I2C1EN().set_bit().PWREN().set_bit());
    rcc.APBENR2.write(|w| w.ADCEN().set_bit());

    cpu::init1();

    // Pullups on I2C and UART RX pin.
    gpioa.PUPDR.write(|w| w.PUPD3().B_0x1().PUPD9().B_0x1().PUPD10().B_0x1());

    // Set I2C pins to open drain.
    gpioa.OTYPER.write(|w| w.OT9().set_bit().OT10().set_bit());

    // Configure pin functions.
    // PA9,10, I2C1, SCL, SDA, AF4.
    gpioa.AFRH.write(|w| w.AFSEL9().B_0x4().AFSEL10().B_0x4());

    gpioa.MODER.modify(
        |_, w| w
            .MODE9 ().B_0x2().MODE10().B_0x2()   // I2C.
            .MODE11().B_0x1().MODE12().B_0x1()); // GPO LEDs.

    debug::init();

    sdbgln!("Debug up");
    dbgln!("Async debug up");

    adc::init1();

    // Poll for the I2C lines to rise.  They may take a while if we are relying
    // on only the internal pull-ups.
    while gpioa.IDR.read().bits() & 0x600 != 0x600 {
    }
    i2c::init();

    // Set the systick interrupt priority to a high value (other interrupts
    // pre-empt).
    // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
    // address.  We want SHPR3.
    assert_eq!(&scb.shpr[1] as *const _ as usize, 0xE000ED20);
    #[cfg(not(test))]
    unsafe {
        scb.shpr[1].write(0xc0 << 24);
    }

    cpu::init2();

    // Enable interrupts for I2C1, ADC and DMA.  FIXME - correct channels!
    use stm32u031::Interrupt::*;
    fn bit(i: stm32u031::Interrupt) -> u32 {1 << i as u16}
    unsafe {
        nvic.iser[0].write(
            bit(I2C1) | bit(ADC_COMP) | bit(debug::UART_ISR) |
            bit(DMA1_CHANNEL1) | bit(DMA1_CHANNEL2_3));
    }

    dbgln!("Going!");

    let _ = oled::init();

    dbgln!("Oled init!");

    adc::init2();

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

fn vconvert(counts: u32) -> u32 {
    const V_FULL_SCALE: f64 = 118.0 / 18.0 * 3.3 * 1000.0; // millivolts.
    const V_SHIFT: u32 = 1;
    const V_MULT: u32 = (V_FULL_SCALE * (1 << V_SHIFT) as f64 + 0.5) as u32;
    const_assert!(V_MULT >= 32768);
    const_assert!(V_MULT < 65536);
    (counts * V_MULT + (1 << V_SHIFT-1)) >> (16 + V_SHIFT)
}

#[test]
fn test_vconvert() {
    const FS: f64 = 118.0 / 18.0 * 3.3 * 1000.0;
    assert!(20000. <= FS && FS <= 25000., "{FS}");
    assert_eq!(vconvert(0), 0);
    let high = vconvert(0xffff);
    assert!((high as f64 - FS * 65535./65536.).abs() <= 1.);
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
        bugger, debug::debug_isr, bugger, bugger,
    ],
};

fn bugger() {
    panic!("Unexpected interrupt");
}

#[test]
fn char_checks() {
    assert_eq!(font::SPACE, 0);
}

#[test]
fn check_vtors() {
    use stm32u031::Interrupt::*;

    assert!(std::ptr::fn_addr_eq(VTORS.reset, main as fn() -> !));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[ADC_COMP as usize],
                                 adc_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[DMA1_CHANNEL1 as usize],
                                 dma1_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[DMA1_CHANNEL2_3 as usize],
                                 i2c::dma23_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[I2C1 as usize],
                                 i2c::i2c_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VTORS.interrupts[debug::UART_ISR as usize],
                                 debug::debug_isr as fn()));
}
