
use stm_common::dma::{Channel, DMA_Channel};

pub use i2c_core::{Result, Wait, read_reg, write};

#[path = "../stm-common/i2c_core.rs"]
mod i2c_core;

pub type I2C = stm32u031::I2C1;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
pub enum I2CLines {
    B6_B7,
    A9_A10,
}

/// I2C receive channel.  Note that there is 0-based v. 1-based confusion.
const RX_CHANNEL: usize = 1;

/// I2C transmit channel.  Note that there is 0-based v. 1-based confusion.
const TX_CHANNEL: usize = 2;

// Use DMA1 Ch2
const RX_MUXIN: u8 = 9;
// Use DMA1 Ch3
const TX_MUXIN: u8 = 10;

fn rx_channel() -> &'static Channel {crate::dma::dma().CH(RX_CHANNEL)}
fn tx_channel() -> &'static Channel {crate::dma::dma().CH(TX_CHANNEL)}

macro_rules!dbgln {($($tt:tt)*) => {if false {crate::dbgln!($($tt)*)}};}

use i2c_core::CONTEXT;

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
        crate::utils::unreachable();
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

    // Enable everything.
    i2c.CR1.write(
        |w| w.TXDMAEN().set_bit().RXDMAEN().set_bit().PE().set_bit()
            . NACKIE().set_bit().ERRIE().set_bit().TCIE().set_bit()
            . STOPIE().set_bit());

    rx_channel().read_from(i2c.RXDR.as_ptr() as *const u8, RX_MUXIN);
    tx_channel().writes_to(i2c.TXDR.as_ptr() as *mut   u8, TX_MUXIN);

    if false {
        i2c_core::write_reg(0, 0, &0i16).defer();
        i2c_core::read(0, &mut 0i16).defer();
        i2c_core::write_read(0, &0u32, &mut 0i16).defer();
    }
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
        unsafe {*CONTEXT.as_mut().outstanding.as_mut() &= !i2c_core::F_DMA_RX};
    }
    if status.GIF3().bit() {
        dbgln!("I2C DMA TX ISR");
        dma.CH(2).CR.write(|w| w.EN().clear_bit());
        unsafe {*CONTEXT.as_mut().outstanding.as_mut() &= !i2c_core::F_DMA_TX};
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
        self.isr(I2C1, i2c_core::i2c_isr).isr(DMA1_CHANNEL2_3, dma23_isr)
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
    assert!(VECTORS.isr[I2C1 as usize] == i2c_core::i2c_isr);
}
