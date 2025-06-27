use crate::dma::{Channel, DMA};
use crate::vcell::{UCell, VCell};

use super::I2C;

// Use DMA1 Ch2
const RX_MUXIN: u32 = 9;
// Use DMA1 Ch3
const TX_MUXIN: u32 = 10;

pub const TMP117: u8 = 0x92;

const RX_CHANNEL: usize = 1;
const TX_CHANNEL: usize = 2;

fn rx_channel() -> &'static Channel {crate::dma::dma().CH(RX_CHANNEL)}
fn tx_channel() -> &'static Channel {crate::dma::dma().CH(TX_CHANNEL)}

pub struct I2cContext {
    pub done: VCell<bool>,
    pending_len: VCell<usize>,
}

//pub static I2C_DONE: VCell<bool> = VCell::new(false);
pub static CONTEXT: UCell<I2cContext> = UCell::new(I2cContext{
    done: VCell::new(false),
    pending_len: VCell::new(0),
});

pub fn init() {
    let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    let i2c = unsafe {&*I2C::ptr()};

    // 50kb/s from 16MHz.
    #[cfg(false)]
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
    #[cfg(true)]
    i2c.TIMINGR.write(unsafe{
        |w| w.PRESC().bits(1)
            . SCLL().bits(3)
            . SCLH().bits(9)
            . SDADEL().bits(1)
            . SCLDEL().bits(1)});

    // Enable everything.
    i2c.CR1.write(
        |w| w.TXDMAEN().set_bit().RXDMAEN().set_bit().PE().set_bit()
            . NACKIE().set_bit().ERRIE().set_bit().TCIE().set_bit());

    // I2C1_RX to DMA2 channel 2.
    rx_channel().PAR.write(|w| unsafe {w.bits(i2c.RXDR.as_ptr().addr() as u32)});
    // I2C1_TX to DMA2 channel 3.
    tx_channel().PAR.write(|w| unsafe {w.bits(i2c.TXDR.as_ptr().addr() as u32)});

    dmamux.CCR(RX_CHANNEL).write(|w| unsafe {w.bits(RX_MUXIN)});
    dmamux.CCR(TX_CHANNEL).write(|w| unsafe {w.bits(TX_MUXIN)});
}

pub fn i2c_isr() {
    let i2c = unsafe {&*I2C::ptr()};
    let context = unsafe {CONTEXT.as_mut()};

    // let status = i2c.ISR.read().bits();
    // dbgln!("I2C ISR {status:#x}");
    let pl = context.pending_len.as_mut();
    let todo = (*pl).min(255);
    *pl -= todo;
    if todo != 0 && i2c.ISR().read().TC().bit() {
        // crate::sdbgln!("I2C cont. {}", context.pending_len);
        i2c.CR2.modify(unsafe{
            |_, w| w.NBYTES().bits(todo as u8).RELOAD().bit(*pl != 0)
                .START().set_bit().AUTOEND().set_bit().RD_WRN().set_bit()});
    }
    else if false && todo != 0 && i2c.ISR().read().TCR().bit() {
        // Continuation.  No start, just more data.
        let cr2 = i2c.CR2.read();
        i2c.CR2.write(unsafe{
            |w| w.bits(cr2.bits()).RELOAD().bit(*pl != 0).NBYTES()
                .bits(todo as u8)});
    }
    else {
        crate::sdbgln!("Unexpected I2C ISR");
    }
}

pub fn dma23_isr() {
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    // I2C TX or RX.
    // FIXME - do we need to wait for TC when appropriate?
    let status = dma.ISR.read();
    dma.IFCR.write(|w| w.CGIF2().set_bit().CGIF3().set_bit());
    // TODO - definitive way to wait for I2C done?
    // dbgln!("DMA23 ISR {status:#x}");
    if status.GIF2().bit() {
        dma.CH(1).CR.write(|w| unsafe {w.bits(0)});
    }
    if status.GIF3().bit() {
        dma.CH(2).CR.write(|w| unsafe {w.bits(0)});
    }
    if status.GIF2().bit() || status.GIF3().bit() {
        // dbgln!("I2C DONE");
        unsafe {*CONTEXT.as_mut().done.as_mut() = true};
    }
}

impl I2cContext {
    fn read_reg_start(&self, addr: u8, reg: u8, data: usize, len: usize) {
        // Should only be called while I2C idle...
        let i2c = unsafe {&*I2C::ptr()};
        self.done.write(false);
        self.pending_len.write(len);

        // Synchronous I2C start for the reg ptr write.
        // No DMA write is active so the dma req. hopefully just gets ignored.
        i2c.CR2.write(unsafe{
            |w| w.START().set_bit().SADD().bits(addr as u16).NBYTES().bits(1)});
        i2c.TXDR.write(|w| unsafe{w.bits(reg as u32)});

        rx_channel().read(data, len, 0);
    }
    fn write_reg_start(&self, addr: u8, reg: u8, data: usize, len: usize) {
        let i2c = unsafe {&*I2C::ptr()};

        cortex_m::interrupt::disable();
        i2c.CR2.write(unsafe{
            |w| w.START().set_bit().AUTOEND().set_bit()
                . SADD().bits(addr as u16).NBYTES().bits(len as u8 + 1)});
        i2c.TXDR.write(|w| unsafe{w.bits(reg as u32)});
        tx_channel().write(data, len, 0);
        self.done.write(false);
        unsafe {cortex_m::interrupt::enable()};
    }
    fn write_start(&self, addr: u8, data: usize, len: usize,
                   last: bool, reload: bool) {
        let i2c = unsafe {&*I2C::ptr()};

        cortex_m::interrupt::disable();
        i2c.CR2.write(unsafe{
            |w| w.START().set_bit().AUTOEND().bit(last).RELOAD().bit(reload)
                . SADD().bits(addr as u16).NBYTES().bits(len as u8)});
        // Do the DMA set-up in the shadow of the address handling.  In case
        // we manage to get an I2C error before the DMA set-up is done, we have
        // interrupts disabled.
        tx_channel().write(data, len, 0);
        self.done.write(false);
        unsafe {cortex_m::interrupt::enable()};
    }
    fn wait(&self) {
        while !self.done.read() {
            cortex_m::asm::wfe();
        }
    }
}

#[inline]
pub fn write_start<T: Flat>(addr: u8, data: *const T, last: bool) {
    CONTEXT.write_start(addr & !1, data.addr(), size_of::<T>(), last, false)
}

pub fn write_wait() {
    CONTEXT.wait();
}

pub fn write<T: Flat>(addr: u8, data: &T) {
    write_start(addr & !1, data as *const T, true);
    write_wait();
}

#[inline]
pub fn write_reg_start<T: Flat>(addr: u8, reg: u8, data: *const T) {
    CONTEXT.write_reg_start(addr & !1, reg, data.addr(), size_of::<T>());
}

#[inline]
pub fn read_reg_start<T: Flat>(addr: u8, reg: u8, data: *mut T) {
    // Should only be called while I2C idle...
    //let context = unsafe {CONTEXT.as_mut()};
    CONTEXT.read_reg_start(addr, reg, data.addr(), size_of::<T>());
}

#[cfg(false)]
pub fn read_wait() {
    CONTEXT.wait();
}

/// Trait Flat is used to check that we pass sane types to read/write.
pub trait Flat {}

impl Flat for [u8] {}
impl<const N: usize> Flat for [u8; N] {}
impl Flat for i16 {}
