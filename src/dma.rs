use crate::cpu::barrier;

pub type Dma = stm32u031::dma1::RegisterBlock;
pub type Channel = stm32u031::dma1::ch::CH;

pub fn dma() -> &'static Dma {unsafe {&*stm32u031::DMA1::ptr()}}

macro_rules!dbgln {($($tt:tt)*) => {if false {crate::dbgln!($($tt)*)}};}

#[allow(non_camel_case_types)]
pub trait DMA_Channel {
    /// Write to peripheral.  The channel should be initialised by writes_to().
    fn write(&self, data: usize, len: usize, size: u8);

    /// Read from peripheral.  The channel should be initialised by read_from().
    fn read(&self, data: usize, len: usize, size: u8);

    /// Configure to write to a peripheral from memory.
    fn writes_to(&self, dst: *mut   u8, request: u8);
    /// Configure to read from a peripheral to memory.
    fn read_from(&self, src: *const u8, request: u8);

    /// Stop and cancel an in-process transfer.
    fn abort(&self);
}

impl DMA_Channel for Channel {
    fn write(&self, data: usize, len: usize, size: u8) {
        setup(self, data, len, size, true)}

    fn read(&self, data: usize, len: usize, size: u8) {
        setup(self, data, len, size, false)}

    fn writes_to(&self, dst: *mut u8, request: u8) {
        self.read_from(dst, request);
    }
    fn read_from(&self, src: *const u8, request: u8) {
        self.PAR.write(|w| w.bits(src as u32));
        // For some reason unsigned_offset_from here leads to crashes.  So
        // do it by hand.
        let me = self as *const Self;
        let ch0 = dma().CH(0) as *const Self;
        let index = (me.addr() - ch0.addr()) / size_of::<Self>();
        dbgln!("DMA Index = {index}");
        let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
        dmamux.CCR[index].write(|w| w.bits(request as u32));
    }

    fn abort(&self) {
        self.CR.write(|w| w);
    }
}

fn setup(ch: &Channel, data: usize, len: usize, size: u8, write: bool) {
    ch.MAR .write(|w| w.bits(data as u32));
    ch.NDTR.write(|w| w.bits(len as u32));
    barrier();
    ch.CR.write(
        |w|w.EN().set_bit().TCIE().set_bit().TEIE().set_bit().MINC().set_bit()
            .DIR().bit(write).PSIZE().bits(size).MSIZE().bits(size));
}

/// Trait Flat is used to check that we pass sane types to read/write.
pub trait Flat {
    fn addr(&self) -> usize {(self as *const Self).addr()}
}

impl Flat for u8 {}
impl<T: Flat> Flat for [T] {}
impl<const N: usize, T: Flat> Flat for [T; N] {}
impl Flat for i16 {}
impl Flat for u16 {}
impl Flat for u32 {}
impl Flat for u64 {}
