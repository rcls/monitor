use core::marker::PhantomData;

use crate::dma::{Channel, DMA, Flat};
use crate::vcell::{UCell, VCell, barrier, interrupt};

pub type I2C = stm32u031::I2C1;

pub type Result = core::result::Result<(), ()>;

#[allow(dead_code)]
#[allow(non_camel_case_types)]
pub enum I2CLines {
    B6_B7,
    A9_A10,
}

// Use DMA1 Ch2
const RX_MUXIN: u32 = 9;
// Use DMA1 Ch3
const TX_MUXIN: u32 = 10;

pub const TMP117: u8 = 0x92;

const RX_CHANNEL: usize = 1;
const TX_CHANNEL: usize = 2;

fn rx_channel() -> &'static Channel {crate::dma::dma().CH(RX_CHANNEL)}
fn tx_channel() -> &'static Channel {crate::dma::dma().CH(TX_CHANNEL)}

#[derive_const(Default)]
pub struct I2cContext {
    outstanding: VCell<u8>,
    error: VCell<u8>,
    pending_len: VCell<usize>,
}

#[must_use]
pub struct Wait<'a>(PhantomData<&'a()>);

pub static CONTEXT: UCell<I2cContext> = UCell::default();

const F_I2C: u8 = 1;
const F_DMA: u8 = 2;

pub fn init() {
    let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    let i2c = unsafe {&*I2C::ptr()};
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};

    if crate::CONFIG.apb1_clocks & 1 << 21 == 0 {
        // Lazy initialization.  Enable the clocks.
        let rcc = unsafe {&*stm32u031::RCC::ptr()};
        rcc.AHBENR.modify(|_,w| w.DMA1EN().set_bit());
        rcc.APBENR1.modify(|_,w| w.I2C1EN().set_bit());
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

    // I2C1_RX to DMA2 channel 2.
    rx_channel().PAR.write(|w| w.bits(i2c.RXDR.as_ptr().addr() as u32));
    // I2C1_TX to DMA2 channel 3.
    tx_channel().PAR.write(|w| w.bits(i2c.TXDR.as_ptr().addr() as u32));

    dmamux.CCR[RX_CHANNEL].write(|w| w.bits(RX_MUXIN));
    dmamux.CCR[TX_CHANNEL].write(|w| w.bits(TX_MUXIN));

    if false {
        write_reg(0, 0, &0i16).defer();
    }
}

pub fn i2c_isr() {
    let i2c = unsafe {&*I2C::ptr()};
    let context = unsafe {CONTEXT.as_mut()};

    let status = i2c.ISR.read();
    // dbgln!("I2C ISR {status:#x}");
    let todo = *context.pending_len.as_mut();
    *context.pending_len.as_mut() = 0;

    if todo != 0 && status.TC().bit() {
        // Assume write -> read transition.
        i2c.CR2.modify(
            |_, w| w.NBYTES().bits(todo as u8)
                .START().set_bit().AUTOEND().set_bit().RD_WRN().set_bit());
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
        panic!("Unexpected I2C ISR {:#x} {:#x}", status.bits(),
               i2c.CR2.read().bits());
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
    fn write_start(&self, addr: u8, data: usize, len: usize, last: bool) {
        let i2c = unsafe {&*I2C::ptr()};

        interrupt::disable();
        i2c.CR2.write(
            |w| w.START().set_bit().AUTOEND().bit(last)
                . SADD().bits(addr as u16).NBYTES().bits(len as u8));
        // Do the DMA set-up in the shadow of the address handling.  In case
        // we manage to get an I2C error before the DMA set-up is done, we have
        // interrupts disabled.
        tx_channel().write(data, len, 0);
        self.arm();
        interrupt::enable();
    }
    fn arm(&self) {
        self.error.write(0);
        self.outstanding.write(F_I2C | F_DMA);
    }
    fn done(&self) -> bool {self.outstanding.read() == 0}
    fn wait(&self) {
        while !self.done() {
            crate::cpu::WFE();
        }
        barrier();
    }
}

impl Wait<'_> {
    pub fn new() -> Self {Wait(PhantomData)}
    pub fn defer(self) {core::mem::forget(self);}
    pub fn wait(self) -> Result {
        CONTEXT.wait();
        if CONTEXT.error.read() == 0 {Ok(())} else {Err(())}
    }
}

impl Drop for Wait<'_> {
    fn drop(&mut self) {let _ = CONTEXT.wait();}
}

pub fn waiter<'a, T: ?Sized>(_: &'a T) ->Wait<'a> {
    Wait::new()
}

pub fn write<'a, T: Flat + ?Sized>(addr: u8, data: &'a T) -> Wait<'a> {
    CONTEXT.write_start(addr & !1, data.addr(), size_of_val(data), true);
    waiter(data)
}

pub fn write_reg<'a, T: Flat + ?Sized>(addr: u8, reg: u8, data: &'a T) -> Wait<'a> {
    CONTEXT.write_reg_start(addr & !1, reg, data.addr(), size_of_val(data));
    waiter(data)
}

pub fn read_reg<'a, T: Flat + ?Sized>(addr: u8, reg: u8, data: &'a mut T) -> Wait<'a> {
    CONTEXT.read_reg_start(addr | 1, reg, data.addr(), size_of_val(data));
    Wait::new()
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
