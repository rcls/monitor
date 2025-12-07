
use stm_common::{dma::Channel, vcell::UCell};
use stm_common::i2c;

pub use i2c::Result;

#[derive_const(Default)]
pub struct I2CMeta;

impl i2c::Meta for I2CMeta {
    fn i2c(&self) -> &'static stm32u031::i2c1::RegisterBlock {unsafe {&*I2C::ptr()}}
    fn rx_channel(&self) -> &'static Channel {crate::dma::dma().CH(RX_CHANNEL)}
    fn tx_channel(&self) -> &'static Channel {crate::dma::dma().CH(TX_CHANNEL)}

    // Use DMA1 Ch2
    fn rx_muxin(&self) -> u8 {9}
    // Use DMA1 Ch3
    fn tx_muxin(&self) -> u8 {10}
}

pub static CONTEXT: UCell<i2c::I2cContext<I2CMeta>> = UCell::default();

pub type I2C = stm32u031::I2C1;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
pub enum I2CLines {
    B6_B7,
    A9_A10,
}

stm_common::implement_i2c_api!(CONTEXT);

/// I2C receive channel.  Note that there is 0-based v. 1-based confusion.
const RX_CHANNEL: usize = 1;

/// I2C transmit channel.  Note that there is 0-based v. 1-based confusion.
const TX_CHANNEL: usize = 2;

macro_rules!dbgln {($($tt:tt)*) => {if false {stm_common::dbgln!($($tt)*)}};}

pub fn init() {
    let i2c = unsafe {&*I2C::ptr()};
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};

    if crate::CONFIG.apb1_clocks & 1 << 21 == 0 {
        // Lazy initialization.  Enable the clocks.
        let rcc = unsafe {&*stm32u031::RCC::ptr()};
        let apbenr1 = rcc.APBENR1.read();
        if apbenr1.I2C1EN().bit() {
            return;                     // Already enabled.
        }
        rcc.APBENR1.write(|w| w.bits(apbenr1.bits()).I2C1EN().set_bit());
        rcc.AHBENR.modify(|_,w| w.DMA1EN().set_bit());
    }

    // Drive the lines up briefly.
    match crate::I2C_LINES {
        I2CLines::B6_B7 => {
            gpiob.BSRR.write(|w| w.BS6().set_bit().BS7().set_bit());
            gpiob.MODER.modify(|_, w| w.MODE6().B_0x1().MODE7().B_0x1());
        },
        I2CLines::A9_A10 => {
            gpioa.BSRR.write(|w| w.BS9().set_bit().BS10().set_bit());
            gpioa.MODER.modify(|_, w| w.MODE9().B_0x1().MODE10().B_0x1());
        }
    }

    // 400kb/s from 16MHz, longer clock pulse.
    if crate::CONFIG.clk == 16000000 {
        i2c.TIMINGR.write(
            |w| w.PRESC().bits(1)
                . SCLL().bits(3).SCLH().bits(9)
                . SDADEL().bits(1).SCLDEL().bits(1));
    }
    else if crate::CONFIG.clk == 2000000 {
        i2c.TIMINGR.write(
            |w| w.PRESC().bits(0)
                . SCLL().bits(1).SCLH().bits(2)
                . SDADEL().bits(0).SCLDEL().bits(1));
    }
    else {
        stm_common::utils::unreachable();
    }

    // Configure the lines for use.
    match crate::I2C_LINES {
        I2CLines::B6_B7 => {
            gpiob.AFRL.modify(|_,w| w.AFSEL6().B_0x4().AFSEL7().B_0x4());
            gpiob.OTYPER.modify(|_,w| w.OT6().set_bit().OT7().set_bit());
            gpiob.MODER.modify(|_, w| w.MODE6().B_0x2().MODE7().B_0x2());
        },
        I2CLines::A9_A10 => {
            gpioa.AFRH.modify(|_,w| w.AFSEL9().B_0x4().AFSEL10().B_0x4());
            gpioa.OTYPER.modify(|_,w| w.OT9().set_bit().OT10().set_bit());
            gpioa.MODER.modify(|_, w| w.MODE9().B_0x2().MODE10().B_0x2());
        }
    }

    if false {
        write_reg(0, 0, &0i16).defer();
        read(0, &mut 0i16).defer();
        write_read(0, &0u32, &mut 0i16).defer();
    }

    // Enable everything.
    CONTEXT.initialize();
}

pub fn dma23_isr() {
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    // I2C TX or RX.
    // FIXME - do we need to wait for TC when appropriate?
    let status = dma.ISR.read();
    dma.IFCR.write(|w| w.CGIF2().set_bit().CGIF3().set_bit());
    if status.GIF2().bit() { // one-based v. zero based...
        dbgln!("I2C DMA RX ISR");
        dma.CH(1).CR.write(|w| w.EN().clear_bit());
        unsafe {*CONTEXT.as_mut().outstanding.as_mut() &= !i2c::F_DMA_RX};
    }
    if status.GIF3().bit() {
        dbgln!("I2C DMA TX ISR");
        dma.CH(2).CR.write(|w| w.EN().clear_bit());
        unsafe {*CONTEXT.as_mut().outstanding.as_mut() &= !i2c::F_DMA_TX};
    }
}

impl crate::cpu::Config {
    pub const fn lazy_i2c(&mut self) -> &mut Self {
        use stm32u031::Interrupt::*;
        let pullup = match crate::I2C_LINES {
            I2CLines::B6_B7  => 1 << 0x16 | 1 << 0x17,
            I2CLines::A9_A10 => 1 << 9    | 1 << 10,
        };
        self.pullup |= pullup;
        self.standby_pu |= pullup;
        self.isr(I2C1, i2c_isr).isr(DMA1_CHANNEL2_3, dma23_isr)
    }
    #[allow(dead_code)]
    pub const fn i2c(&mut self) -> &mut Self {
        self.lazy_i2c().clocks(1 << 0, 1 << 21, 0)
    }
}

#[test]
fn check_vtors() {
    use stm32u031::Interrupt::*;
    use crate::cpu::VECTORS;

    assert!(VECTORS.isr[DMA1_CHANNEL2_3 as usize] == dma23_isr);
    assert!(VECTORS.isr[I2C1 as usize] == i2c_isr);
}
