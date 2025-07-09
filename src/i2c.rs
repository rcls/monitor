use core::marker::PhantomData;

use crate::dma::{Channel, DMA};
use crate::vcell::{UCell, VCell, barrier, interrupt};

use super::I2C;

pub type Result = core::result::Result<(), ()>;

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
    outstanding: VCell<u8>,
    error: VCell<u8>,
    pending_len: VCell<usize>,
}

#[must_use]
pub struct Wait<'a>(PhantomData<&'a()>);

//pub static I2C_DONE: VCell<bool> = VCell::new(false);
pub static CONTEXT: UCell<I2cContext> = UCell::new(I2cContext{
    outstanding: VCell::new(0),
    error      : VCell::new(0),
    pending_len: VCell::new(0),
});

const F_I2C: u8 = 1;
const F_DMA: u8 = 2;

pub fn init() {
    let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    let i2c = unsafe {&*I2C::ptr()};

    // 50kb/s from 16MHz.
    #[cfg(false)]
    i2c.TIMINGR.write(
        |w| w.PRESC().bits(7)
            . SCLL().bits(19)
            . SCLH().bits(15)
            . SDADEL().bits(2)
            . SCLDEL().bits(4));
    // 400kb/s from 16MHz, STM documented timings.
    #[cfg(false)]
    i2c.TIMINGR.write(
        |w| w.PRESC().bits(1)
            . SCLL().bits(9)
            . SCLH().bits(3)
            . SDADEL().bits(2)
            . SCLDEL().bits(3));
    // 400kb/s from 16MHz, longer clock pulse.
    if super::CPU_CLK == 16000000 {
        i2c.TIMINGR.write(
            |w| w.PRESC().bits(1)
                . SCLL().bits(3)
                . SCLH().bits(9)
                . SDADEL().bits(1)
                . SCLDEL().bits(1));
    }
    else if super::CPU_CLK == 2000000 {
        i2c.TIMINGR.write(
            |w| w.PRESC().bits(0)
                . SCLL().bits(1)
                . SCLH().bits(2)
                . SDADEL().bits(0)
                . SCLDEL().bits(1));
    }
    else {
        crate::vcell::unreachable();
    }

    // Enable everything.
    i2c.CR1.write(
        |w| w.TXDMAEN().set_bit().RXDMAEN().set_bit().PE().set_bit()
            . NACKIE().set_bit().ERRIE().set_bit().TCIE().set_bit()
            . STOPIE().set_bit());

    // I2C1_RX to DMA2 channel 2.
    rx_channel().PAR.write(|w| w.bits(i2c.RXDR.as_ptr().addr() as u32));
    // I2C1_TX to DMA2 channel 3.
    tx_channel().PAR.write(|w| w.bits(i2c.TXDR.as_ptr().addr() as u32));

    dmamux.CCR(RX_CHANNEL).write(|w| w.bits(RX_MUXIN));
    dmamux.CCR(TX_CHANNEL).write(|w| w.bits(TX_MUXIN));

    if false {
        write_reg(0, 0, &0i16).defer();
        // read_wait();
    }
}

pub fn i2c_isr() {
    let i2c = unsafe {&*I2C::ptr()};
    let context = unsafe {CONTEXT.as_mut()};

    let status = i2c.ISR.read();
    // dbgln!("I2C ISR {status:#x}");
    let pl = *context.pending_len.as_mut();
    let todo = pl.min(255);
    let pl = pl - todo;
    *context.pending_len.as_mut() = pl;
    let more = false && pl != 0;        // We don't need this yet.

    if todo != 0 && status.TC().bit() {
        // Assume write -> read transition.
        i2c.CR2.modify(
            |_, w| w.NBYTES().bits(todo as u8).RELOAD().bit(more)
                .START().set_bit().AUTOEND().set_bit().RD_WRN().set_bit());
    }
    else if false && todo != 0 && status.TCR().bit() {
        // Continuation.  No start, just more data.
        let cr2 = i2c.CR2.read();
        i2c.CR2.write(
            |w| w.bits(cr2.bits()).RELOAD().bit(more).NBYTES()
                .bits(todo as u8));
        crate::sdbgln!("Reload {todo} {:#x} {:#x}", status.bits(), cr2.bits());
    }
    else if status.STOPF().bit() {
        i2c.ICR.write(|w| w.STOPCF().set_bit());
        *context.outstanding.as_mut() &= !F_I2C;
    }
    else if status.ARLO().bit() || status.BERR().bit() || status.NACKF().bit() {
        i2c.ICR.write(
            |w| w.ARLOCF().set_bit().BERRCF().set_bit().NACKCF().set_bit());
        *context.outstanding.as_mut() = 0;
        *context.error.as_mut() = 1;
        rx_channel().CR.write(|w| w);
    }
    else {
        crate::sdbgln!("Unexpected I2C ISR {:#x} {:#x}", status.bits(),
                       i2c.CR2.read().bits());
        loop{}
    }
}

pub fn dma23_isr() {
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    // I2C TX or RX.
    // FIXME - do we need to wait for TC when appropriate?
    let status = dma.ISR.read();
    dma.IFCR.write(|w| w.CGIF2().set_bit().CGIF3().set_bit());
    if status.GIF2().bit() { // one-based v. zero based...
        dma.CH(1).CR.write(|w| w.EN().clear_bit());
    }
    if status.GIF3().bit() {
        dma.CH(2).CR.write(|w| w.EN().clear_bit());
    }
    if status.GIF2().bit() || status.GIF3().bit() {
        // dbgln!("I2C DONE");
        unsafe {*CONTEXT.as_mut().outstanding.as_mut() &= !F_DMA};
    }
}

impl I2cContext {
    fn read_reg_start(&self, addr: u8, reg: u8, data: usize, len: usize) {
        // Should only be called while I2C idle...
        let i2c = unsafe {&*I2C::ptr()};
        self.arm();
        self.pending_len.write(len);

        // Synchronous I2C start for the reg ptr write.
        // No DMA write is active so the dma req. hopefully just gets ignored.
        i2c.CR2.write(
            |w| w.START().set_bit().SADD().bits(addr as u16).NBYTES().bits(1));
        i2c.TXDR.write(|w| w.bits(reg as u32));

        rx_channel().read(data, len, 0);
    }
    #[inline(never)]
    fn write_reg_start(&self, addr: u8, reg: u8, data: usize, len: usize) {
        let i2c = unsafe {&*I2C::ptr()};

        interrupt::disable();
        i2c.CR2.write(
            |w| w.START().set_bit().AUTOEND().set_bit()
                . SADD().bits(addr as u16).NBYTES().bits(len as u8 + 1));
        i2c.TXDR.write(|w| w.bits(reg as u32));
        tx_channel().write(data, len, 0);
        self.arm();
        interrupt::enable();
    }
    #[inline(never)]
    fn write_start(&self, addr: u8, data: usize, len: usize,
                   last: bool, reload: bool) {
        let i2c = unsafe {&*I2C::ptr()};

        interrupt::disable();
        i2c.CR2.write(
            |w| w.START().set_bit().AUTOEND().bit(last).RELOAD().bit(reload)
                . SADD().bits(addr as u16).NBYTES().bits(len as u8));
        // Do the DMA set-up in the shadow of the address handling.  In case
        // we manage to get an I2C error before the DMA set-up is done, we have
        // interrupts disabled.
        tx_channel().write(data, len, 0);
        self.arm();
        interrupt::enable();
    }
    pub fn arm(&self) {
        self.error.write(0);
        self.outstanding.write(F_I2C | F_DMA);
    }
    pub fn done(&self) -> bool {self.outstanding.read() == 0}
    fn wait(&self) -> Result {
        while !self.done() {
            crate::vcell::WFE();
        }
        barrier();
        if self.error.read() == 0 {Ok(())} else {Err(())}
    }
}

impl Wait<'_> {
    pub fn defer(self) {core::mem::forget(self);}
    pub fn wait(self) -> Result {CONTEXT.wait()}
}

impl Drop for Wait<'_> {
    fn drop(&mut self) {let _ = CONTEXT.wait();}
}

pub fn waiter<'a, T>(_: &'a mut T) ->Wait<'a> {
    Wait(PhantomData)
}

pub fn write<'a, T: Flat + ?Sized>(addr: u8, data: &'a T) -> Wait<'a> {
    CONTEXT.write_start(addr & !1, data.addr(), size_of_val(data),
                        true, false);
    Wait(PhantomData)
}

pub fn write_reg<'a, T: Flat + ?Sized>(addr: u8, reg: u8, data: &'a T) -> Wait<'a> {
    CONTEXT.write_reg_start(addr & !1, reg, data.addr(), size_of_val(data));
    Wait(PhantomData)
}

pub fn read_reg<'a, T: Flat + ?Sized>(addr: u8, reg: u8, data: &'a mut T) -> Wait<'a> {
    // Should only be called while I2C idle...
    //let context = unsafe {CONTEXT.as_mut()};
    CONTEXT.read_reg_start(addr | 1, reg, data.addr(), size_of_val(data));
    Wait(PhantomData)
}

/// Trait Flat is used to check that we pass sane types to read/write.
pub trait Flat {
    fn addr(&self) -> usize {(self as *const Self).addr()}
}

impl Flat for [u8] {}
impl<const N: usize> Flat for [u8; N] {}
impl<const N: usize> Flat for [u16; N] {}
impl Flat for i16 {}
impl<T: Flat> Flat for VCell<T> {}
