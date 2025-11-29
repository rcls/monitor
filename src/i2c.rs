use core::marker::PhantomData;

use crate::cpu::barrier;
use crate::dma::{Channel, DMA_Channel, Flat};
use crate::vcell::{UCell, VCell};

pub type I2C = stm32u031::I2C1;

pub type Result = core::result::Result<(), ()>;

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

#[derive_const(Default)]
pub struct I2cContext {
    outstanding: VCell<u8>,
    error: VCell<u8>,
    pending_len: VCell<usize>,
}

/// Marker struct to indicate that we are waiting upon an I2C transaction.
///
/// The phantoms make sure we bind the lifetime, with the correct mutability.
/// We would much rather just have a PhantomData of the correct reference type,
/// but then Wait would be different depending on the data in flight!
#[must_use]
pub struct Wait<'a>(PhantomData<&'a [u8]>, PhantomData<&'a mut [u8]>);

pub static CONTEXT: UCell<I2cContext> = UCell::default();

const F_I2C: u8 = 1;
const F_DMA_RX: u8 = 2;
const F_DMA_TX: u8 = 4;

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
        write_reg(0, 0, &0i16).defer();
        read(0, &mut 0i16).defer();
        write_read(0, &0u32, &mut 0i16).defer();
    }
}

fn i2c_isr() {
    let i2c = unsafe {&*I2C::ptr()};
    let context = unsafe {CONTEXT.as_mut()};

    let status = i2c.ISR.read();
    dbgln!("I2C ISR {:#x}", status.bits());
    let todo = *context.pending_len.as_mut();
    *context.pending_len.as_mut() = 0;

    if todo != 0 && status.TC().bit() {
        // Assume write -> read transition.
        dbgln!("I2C now read {todo} bytes [{:#x}]", status.bits());
        let cr2 = i2c.CR2.read();
        i2c.CR2.write(
            |w|w.NBYTES().bits(todo as u8).START().set_bit()
                .AUTOEND().set_bit().RD_WRN().set_bit()
                .SADD().bits(cr2.SADD().bits()));
    }
    else if status.STOPF().bit() {
        // FIXME - if we see a stop when waiting for the above, we'll hang.
        dbgln!("I2C STOPF");
        i2c.ICR.write(|w| w.STOPCF().set_bit());
        *context.outstanding.as_mut() &= !F_I2C;
    }
    else if status.ARLO().bit() || status.BERR().bit() || status.NACKF().bit() {
        dbgln!("I2C Error");
        i2c.ICR.write(
            |w| w.ARLOCF().set_bit().BERRCF().set_bit().NACKCF().set_bit());
        *context.outstanding.as_mut() = 0;
        *context.error.as_mut() = 1;
    }
    else {
        panic!("Unexpected I2C ISR {:#x} {:#x}", status.bits(),
               i2c.CR2.read().bits());
    }
    // Stop the ISR from prematurely retriggering.  Otherwise we may return
    // from the ISR before the update has propagated through the I2C subsystem,
    // leaving the interrupt line high.
    i2c.ISR.read();

    dbgln!("I2C ISR done, {}", context.outstanding.read());
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
        unsafe {*CONTEXT.as_mut().outstanding.as_mut() &= !F_DMA_RX};
    }
    if status.GIF3().bit() {
        dbgln!("I2C DMA TX ISR");
        dma.CH(2).CR.write(|w| w.EN().clear_bit());
        unsafe {*CONTEXT.as_mut().outstanding.as_mut() &= !F_DMA_TX};
    }
}

impl I2cContext {
    fn read_reg_start(&self, addr: u8, reg: u8, data: usize, len: usize) {
        // Should only be called while I2C idle...
        let i2c = unsafe {&*I2C::ptr()};
        self.arm(F_I2C | F_DMA_RX);
        self.pending_len.write(len);

        // Synchronous I2C start for the reg ptr write.
        // No DMA write is active so the dma req. hopefully just gets ignored.
        i2c.CR2.write(
            |w| w.START().set_bit().SADD().bits(addr as u16).NBYTES().bits(1));
        i2c.TXDR.write(|w| w.bits(reg as u32));

        rx_channel().read(data, len, 0);
    }
    #[inline(never)]
    fn read_start(&self, addr: u8, data: usize, len: usize) {
        let i2c = unsafe {&*I2C::ptr()};

        rx_channel().read(data, len, 0);
        self.arm(F_I2C | F_DMA_RX);
        i2c.CR2.write(
            |w|w.START().set_bit().AUTOEND().bit(true).SADD().bits(addr as u16)
                .RD_WRN().set_bit().NBYTES().bits(len as u8));
    }
    #[inline(never)]
    fn write_reg_start(&self, addr: u8, reg: u8, data: usize, len: usize) {
        let i2c = unsafe {&*I2C::ptr()};

        self.arm(F_I2C | F_DMA_TX);
        i2c.CR2.write(
            |w| w.START().set_bit().AUTOEND().set_bit()
                . SADD().bits(addr as u16).NBYTES().bits(len as u8 + 1));
        i2c.TXDR.write(|w| w.TXDATA().bits(reg));
        tx_channel().write(data, len, 0);
    }
    #[inline(never)]
    fn write_start(&self, addr: u8, data: usize, len: usize, last: bool) {
        let i2c = unsafe {&*I2C::ptr()};

        self.arm(F_I2C | F_DMA_TX);
        i2c.CR2.write(
            |w| w.START().set_bit().AUTOEND().bit(last)
                . SADD().bits(addr as u16).NBYTES().bits(len as u8));
        tx_channel().write(data, len, 0);
    }

    #[inline(never)]
    fn write_read_start(&self, addr: u8, wdata: usize, wlen: usize,
                        rdata: usize, rlen: usize) {
        let i2c = unsafe {&*I2C::ptr()};
        tx_channel().write(wdata, wlen, 0);
        rx_channel().read (rdata, rlen, 0);
        self.pending_len.write(rlen);
        self.arm(F_I2C | F_DMA_TX | F_DMA_RX);
        i2c.CR2.write(
            |w|w.START().set_bit().SADD().bits(addr as u16)
                .NBYTES().bits(wlen as u8));
    }
    fn arm(&self, flags: u8) {
        self.error.write(0);
        self.outstanding.write(flags);
        barrier();
    }

    fn done(&self) -> bool {self.outstanding.read() == 0}
    fn wait(&self) {
        while !self.done() {
            crate::cpu::WFE();
        }
        barrier();
        if self.error.read() != 0 {
            self.error_cleanup();
        }
    }
    fn error_cleanup(&self) {
        dbgln!("I2C error cleanup");
        let i2c = unsafe {&*I2C::ptr()};
        // Clean-up the DMA and reset the I2C.
        i2c.CR1.write(|w| w.PE().clear_bit());
        tx_channel().abort();
        rx_channel().abort();
        rx_channel().read_from(i2c.RXDR.as_ptr() as *const u8, RX_MUXIN);
        tx_channel().writes_to(i2c.TXDR.as_ptr() as *mut   u8, TX_MUXIN);
        i2c.CR1.write(
            |w|w.TXDMAEN().set_bit().RXDMAEN().set_bit().PE().set_bit()
                .NACKIE().set_bit().ERRIE().set_bit().TCIE().set_bit()
                .STOPIE().set_bit());
    }
}

impl<'a> Wait<'a> {
    pub fn default() -> Self {
        barrier();
        Wait(PhantomData, PhantomData)
    }
    pub fn new<T: ?Sized>(_ : &'a T) -> Self {Self::default()}
    pub fn defer(self) {core::mem::forget(self);}
    pub fn wait(self) -> Result {
        CONTEXT.wait();
        let result = CONTEXT.error.read();
        core::mem::forget(self);
        if result == 0 {Ok(())} else {Err(())}
    }
}

impl Drop for Wait<'_> {
    fn drop(&mut self) {let _ = CONTEXT.wait();}
}

pub fn write<T: Flat + ?Sized>(addr: u8, data: &T) -> Wait<'_> {
    CONTEXT.write_start(addr & !1, data.addr(), size_of_val(data), true);
    Wait::new(data)
}

pub fn write_reg<T: Flat + ?Sized>(addr: u8, reg: u8, data: &T) -> Wait<'_> {
    CONTEXT.write_reg_start(addr & !1, reg, data.addr(), size_of_val(data));
    Wait::new(data)
}

pub fn read<T: Flat + ?Sized>(addr: u8, data: &mut T) -> Wait<'_> {
    CONTEXT.read_start(addr | 1, data.addr(), size_of_val(data));
    Wait::new(data)
}

pub fn read_reg<T: Flat + ?Sized>(addr: u8, reg: u8, data: &mut T) -> Wait<'_> {
    CONTEXT.read_reg_start(addr | 1, reg, data.addr(), size_of_val(data));
    Wait::new(data)
}

pub fn write_read<'a, T: Flat + ?Sized, U: Flat + ?Sized>(
    addr: u8, wdata: &'a T, rdata: &'a mut U) -> Wait<'a> {
    CONTEXT.write_read_start(addr, wdata.addr(), size_of_val(wdata),
                             rdata.addr(), size_of_val(rdata));
    Wait::new(rdata)
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
