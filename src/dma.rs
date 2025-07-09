use crate::vcell::barrier;

pub type Dma = stm32u031::dma1::RegisterBlock;
pub type Channel = stm32u031::dma1::ch::CH;

pub fn dma() -> &'static Dma {unsafe {&*stm32u031::DMA1::ptr()}}

pub trait DMA {
    fn setup(&self, data: usize, len: usize, size: u8, write: bool);
    // Write to peripheral.
    fn write(&self, data: usize, len: usize, size: u8) {
        self.setup(data, len, size, true)}
    // Read from peripheral.
    fn read(&self, data: usize, len: usize, size: u8) {
        self.setup(data, len, size, false)}
}

impl DMA for Channel {
    // size is 0 = 1byte, 1=2byte.
    fn setup(&self, data: usize, len: usize, size: u8, write: bool) {
        self.MAR .write(|w| w.bits(data as u32));
        self.NDTR.write(|w| w.bits(len as u32));
        barrier();
        self.CR.write(
            |w| w.EN().set_bit().TCIE().set_bit().TEIE().set_bit()
                .MINC().set_bit().DIR().bit(write)
                .PSIZE().bits(size).MSIZE().bits(size));
    }
}

/// Trait Flat is used to check that we pass sane types to read/write.
pub trait Flat {
    fn addr(&self) -> usize {(self as *const Self).addr()}
}

impl Flat for [u8] {}
impl<const N: usize> Flat for [u8; N] {}
impl<const N: usize> Flat for [u16; N] {}
impl Flat for i16 {}
impl Flat for u32 {}
impl Flat for u64 {}
//impl<T: Flat> Flat for VCell<T> {}
