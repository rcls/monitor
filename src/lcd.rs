#![allow(non_upper_case_globals)]

use crate::cpu::WFE;

use crate::LCD_BITS;

pub const SEG_A: u8 = 32;
pub const SEG_B: u8 = 16;
pub const SEG_C: u8 = 2;
pub const SEG_D: u8 = 4;
pub const SEG_E: u8 = 8;
pub const SEG_F: u8 = 64;
pub const SEG_G: u8 = 128;
pub const DOT: u8 = 1;

pub const D0: u8 = D8 & !SEG_G;
pub const D1: u8 = SEG_B | SEG_C;
pub const D2: u8 = D8 & !SEG_C & !SEG_F;
pub const D3: u8 = D9 & !SEG_F;
pub const D4: u8 = D1 | SEG_F | SEG_G;
pub const D5: u8 = D9 & !SEG_B;
pub const D6: u8 = D5 | SEG_E;
pub const D7: u8 = D1 | SEG_A;
pub const D8: u8 = !DOT;
pub const D9: u8 = D8 & !SEG_E;

pub const DEG: u8 = SEG_A | SEG_B | SEG_F | SEG_G;
pub const Db: u8 = D6 & !SEG_A;
pub const Dc: u8 = SEG_D | SEG_E | SEG_G;
pub const Dd: u8 = Dc | SEG_B | SEG_C;

pub const MINUS: u8 = SEG_G;
#[allow(unused)]
pub const COL1: Segments = 1;
#[allow(unused)]
pub const COL2: Segments = if LCD_BITS >= 48 {1 << 48} else {0};

pub trait SegmentsTrait<const N: usize> {type Segments;}
impl SegmentsTrait<48> for () {type Segments = u64;}
impl SegmentsTrait<32> for () {type Segments = u32;}
pub type Segments = <() as SegmentsTrait<{crate::LCD_BITS}>>::Segments;

pub static DIGITS: [u8; 16] = [
    D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, DEG, Db, Dc, Dd, DOT, MINUS];

/// Initialize the I/O to the LCD.
pub fn init() {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let spi   = unsafe {&*stm32u031::SPI1 ::ptr()};

    // Remove any standby pull-up/pull-downs.
    crate::cpu::Config::new(0).lcd().clear_pupd();

    // OE pin A11, CP=B3, DAT=B5, STR=A15, COM = B9.
    // Set OE to output low.
    // Set PWR=PB4 to be high.
    // Leave COM and COL1 (A12) as hi-z until we set the output.
    // gpioa.BSRR.write(|w| w.BR11().set_bit());
    gpioa.MODER.modify(|_,w| w.MODE11().B_0x1().MODE15().B_0x1());
    gpiob.BSRR.write(|w| w.BS4().set_bit());
    gpiob.MODER.modify(|_,w| w.MODE3().B_0x1().MODE4().B_0x1().MODE5().B_0x1());

    // Set the function (AF5) for the SPI pins CP (B3), DAT(B5).
    gpiob.AFRL.modify(|_,w| w.AFSEL3().B_0x5().AFSEL5().B_0x5());
    gpiob.MODER.modify(|_,w| w.MODE3().B_0x2().MODE5().B_0x2());

    // Set-up SPI1.
    spi.CR2.write(|w| w.DS().B_0xF().SSOE().set_bit());
    let br = (crate::CONFIG.clk / 1000000).ilog2() as u8;
    spi.CR1.write(
        |w| w.LSBFIRST().set_bit().SPE().set_bit().BR().bits(br)
            .MSTR().set_bit());
}

pub fn update_lcd(bits: Segments, comm: bool) {
    // Invert segment bits if comm is high.
    let bits = if comm {!bits} else {bits};

    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let spi   = unsafe {&*stm32u031::SPI1 ::ptr()};

    // Set OE (A11) and STR (A15) low.
    gpioa.BSRR.write(|w| w.BR11().set_bit().BR15().set_bit());

    // Set COL1 (A12) to high impedance (input).
    gpioa.MODER.modify(|_, w| w.MODE12().B_0x0());
    // Set COM (PB9) to high impedance.
    gpiob.MODER.modify(|_, w| w.MODE9().B_0x0());

    // Do one dummy data read before transmitting.  This will (eventually!)
    // clear out fifo-not-clear problems.
    spi.DR.read();

    // We have a 32 bit fifo.
    spi.DR.write(|w| w.bits((bits & 0xffff) as u16));
    spi.DR.write(|w| w.bits((bits >> 16) as u16));
    // If we have have a third word to write, wait for one RX word then send it.
    if LCD_BITS > 32 {
        rx_word(spi);
        spi.DR.write(|w| w.bits((bits >> 16 >> 16) as u16));
    }
    // Wait for 2 words RX.
    rx_word(spi);
    rx_word(spi);

    // Set STR (A15) high.  Set col1 (A12), it's not driven yet.  (If both S and
    // R are set, then S wins.)
    let col1 = if LCD_BITS > 32 {bits as u64 & 1 << 48 != 0} else {comm};
    gpioa.BSRR.write(|w|
        w.BS15().set_bit().BS12().bit(col1).BR12().set_bit());

    // Set COM polarity.
    gpiob.BSRR.write(|w| w.BS9().bit(comm).BR9().set_bit());
    // Drive com.
    gpiob.MODER.modify(|_, w| w.MODE9().B_0x1());

    // Set OE & STR.
    gpioa.BSRR.write(|w| w.BS11().set_bit().BS15().set_bit());
    // Drive COL1.
    gpioa.MODER.modify(|_, w| w.MODE12().B_0x1());
}

#[inline(never)]
fn rx_word(spi: &stm32u031::spi1::RegisterBlock) {
    let nvic = unsafe {&*cortex_m::peripheral::NVIC::PTR};
    // SEVONPEND only wakes up on the rising edge of the pending flag.  So we
    // need to clear the flag.
    unsafe {
        nvic.icpr[0].write(1 << stm32u031::Interrupt::SPI1 as u32);
    }
    // Make sure we actually get a rising edge on the interrupt!
    let cr2 = spi.CR2.read().bits();
    spi.CR2.write(|w| w.bits(cr2).RXNEIE().set_bit());
    loop {
        WFE(); // We rely on SEVONPEND to wake us up.
        if spi.SR.read().RXNE().bit() {
            break;
        }
    }
    spi.CR2.write(|w| w.bits(cr2).RXNEIE().clear_bit());
    spi.DR.read();
}

impl crate::cpu::Config {
    pub const fn lcd(&mut self) -> &mut Self {
        // Preserve the control lines.
        self.standby_pu |= 1 << 11 | 1 << 0x14; // OE, DPWR.
        self.standby_pd |= 1 << 15 | 1 << 0x13; // STR, CP
        self.keep_pu    |= 1 << 12 | 1 << 0x15 | 1 << 0x19; // COL, DAT, COM.
        self.keep_pd    |= 1 << 12 | 1 << 0x15 | 1 << 0x19; // COL, DAT, COM.
        self.clocks(0, 0, 1 << 12)
    }
}

#[test]
fn check_config() {
    assert!(crate::CONFIG.apb2_clocks & 1 << 12 != 0);
}
