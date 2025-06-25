use crate::vcell::{VCell, VCellAccess};

use super::I2C;

// Use DMA1 Ch2
const I2C_RX_MUXIN: u32 = 9;
// Use DMA1 Ch3
const I2C_TX_MUXIN: u32 = 10;

pub const TMP117: u16 = 0x92;

pub static I2C_DONE: VCell<bool> = VCell::new(false);

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
            . NACKIE().set_bit().ERRIE().set_bit());

    // I2C1_RX to DMA2 channel 2.
    dmamux.C1CR.write(|w| unsafe {w.bits(I2C_RX_MUXIN)});
    // I2C1_TX to DMA2 channel 3.
    dmamux.C2CR.write(|w| unsafe {w.bits(I2C_TX_MUXIN)});
}

pub fn i2c_isr() {
    // let i2c = unsafe {&*I2C ::ptr()};
    // Don't handle TC here, just errors!
    // let status = i2c.ISR.read().bits();
    // dbgln!("I2C ISR {status:#x}");
}

pub fn dma23_isr() {
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    // I2C TX or RX.
    // FIXME - do we need to wait for TC when appropriate?
    let status = dma.ISR.read().bits();
    dma.IFCR.write(|w| w.CGIF2().set_bit().CGIF3().set_bit());
    // TODO - definitive way to wait for I2C done?
    // dbgln!("DMA23 ISR {status:#x}");
    if status & 0xaa0 != 0 {
        // dbgln!("I2C DONE");
        I2C_DONE.write(true);
    }
}

fn write_start_raw(addr: u16, data: *const u8, len: usize) {
    let i2c = unsafe {&*I2C::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};

    I2C_DONE.write(false);
    // FIXME - what needs to be done every time?
    dma.CCR3.write(|w| unsafe {w.bits(0)});
    dma.CMAR3.write(|w| unsafe {w.bits(data as u32)});
    dma.CPAR3.write(|w| unsafe {w.bits(i2c.TXDR.as_ptr() as u32)});
    // FIXME - len must be <256.
    dma.CNDTR3.write(|w| unsafe {w.bits(len as u32)});
    // dbgln!("DMA CCR3 {:#x}", dma.CCR3.read().bits());
    dma.CCR3.write(
        |w| w.EN().set_bit().TCIE().set_bit().TEIE().set_bit().MINC().set_bit()
            .DIR().set_bit());
    i2c.CR2.write(unsafe{
        |w| w.START().set_bit().SADD().bits(addr).NBYTES()
            .bits(len as u8)});
}

pub fn write_start<T>(addr: u16, data: *const T) {
    write_start_raw(addr, data as *const u8, size_of::<T>())
}

pub fn write_wait() {
    while !I2C_DONE.read() {
        cortex_m::asm::wfe();
    }
}

fn read_start_raw(addr: u16, data: *mut u8, len: usize) {
    let i2c = unsafe {&*I2C::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};

    I2C_DONE.write(false);
    // Start the I2C read.
    dma.CCR2.write(|w| unsafe {w.bits(0)});
    dma.CMAR2.write(|w| unsafe {w.bits(data as u32)});
    dma.CPAR2.write(|w| unsafe {w.bits(i2c.RXDR.as_ptr() as u32)});
    dma.CNDTR2.write(|w| unsafe {w.bits(len as u32)});
    dma.CCR2.write(
        |w| w.EN().set_bit().TCIE().set_bit().TEIE().set_bit().MINC().set_bit());
    i2c.CR2.write(unsafe{
        |w| w.START().set_bit().SADD().bits(addr).NBYTES().bits(len as u8)
            .AUTOEND().set_bit().RD_WRN().set_bit()});
}

pub fn read_start<T>(addr: u16, data: *mut T) {
    read_start_raw(addr, data as *mut u8, size_of::<T>());
}

pub fn _i2c_read_wait() {
    write_wait();
}
