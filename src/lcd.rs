#![allow(non_upper_case_globals)]

use crate::dma::{DMA, Flat};
use crate::vcell::UCell;
use crate::cpu::WFE;

use super::{LCD_BITS, LcdBits};

pub static LCD: crate::vcell::UCell<LCD> = crate::vcell::UCell::new(LCD::new());

pub struct LCD {
    comm: bool,
    pub segments: LcdBits,
}

const DMA_RX: usize = 3;
const DMA_TX: usize = 4;
const DMA_RX_MUX: u32 = 36;
const DMA_TX_MUX: u32 = 37;

const USE_TX_DMA: bool = LCD_BITS > 32;

// ABCDEFG
pub const SEG_A: u8 = 32;
pub const SEG_B: u8 = 16;
pub const SEG_C: u8 = 2;
pub const SEG_D: u8 = 4;
pub const SEG_E: u8 = 8;
pub const SEG_F: u8 = 64;
pub const SEG_G: u8 = 128;
pub const DOT: u8 = 1;

pub const D0: u8 = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F;
pub const D1: u8 = SEG_B | SEG_C;
pub const D2: u8 = SEG_A | SEG_B | SEG_D | SEG_E | SEG_G;
pub const D3: u8 = SEG_A | SEG_B | SEG_C | SEG_D | SEG_G;
pub const D4: u8 = SEG_B | SEG_C | SEG_F | SEG_G;
pub const D5: u8 = SEG_A | SEG_C | SEG_D | SEG_F | SEG_G;
pub const D6: u8 = SEG_A | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
pub const D7: u8 = SEG_A | SEG_B | SEG_C;
pub const D8: u8 = SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F | SEG_G;
pub const D9: u8 = SEG_A | SEG_B | SEG_C | SEG_D | SEG_F | SEG_G;

// Hex!
pub const DA: u8 = D8 & !SEG_D;
pub const Db: u8 = D6 & !SEG_A;
pub const Dc: u8 = SEG_D | SEG_E | SEG_G;
pub const Dd: u8 = SEG_B | SEG_C | SEG_D | SEG_G;
pub const DE: u8 = D8 & !SEG_B & !SEG_C;
pub const DF: u8 = DE & !SEG_D;

pub const MINUS: u8 = SEG_G;

pub static DIGITS: [u8; 16] = [
    D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, DA, Db, Dc, Dd, DE, DF];

impl LCD {
    pub const fn new() -> LCD {LCD{comm: false, segments: 0}}

    pub fn tick(&mut self) {
        self.comm = !self.comm;
        update_lcd(self.segments, self.comm);
    }
}

/// Initialize the I/O to the LCD.  This should be good when waking up from
/// standby, as we leave the LCD high impedance until we write something to it.
pub fn init() {
    let dma    = crate::dma::dma();
    let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    let gpioa  = unsafe {&*stm32u031::GPIOA ::ptr()};
    let gpiob  = unsafe {&*stm32u031::GPIOB ::ptr()};
    let spi    = unsafe {&*stm32u031::SPI1  ::ptr()};
    let rcc    = unsafe {&*stm32u031::RCC   ::ptr()};

    // Enable SPI1 clock.
    rcc.APBENR2.modify(|_, w| w.SPI1EN().set_bit());

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
    spi.CR2.write(
        |w| w.DS().B_0xF().SSOE().set_bit()
            .RXDMAEN().set_bit().TXDMAEN().bit(USE_TX_DMA));
    let br = (crate::CPU_CLK / 1000000).ilog2() as u8;
    spi.CR1.write(
        |w| w.LSBFIRST().set_bit().SPE().set_bit().BR().bits(br)
            .MSTR().set_bit());

    dma.CH(DMA_RX).PAR.write(
        |w| w.bits(spi.DR.as_ptr().addr() as u32));
    dmamux.CCR(DMA_RX).write(|w| w.bits(DMA_RX_MUX));
    if USE_TX_DMA {
        dma.CH(DMA_TX).PAR.write(|w| w.bits(spi.DR.as_ptr().addr() as u32));
        dmamux.CCR(DMA_TX).write(|w| w.bits(DMA_TX_MUX));
    }
}

pub fn update_lcd(bits: LcdBits, comm: bool) {
    // Invert segment bits if comm is high.
    let bits = if comm {!bits} else {bits};

    let dma   = crate::dma::dma();
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let spi   = unsafe {&*stm32u031::SPI1 ::ptr()};

    // Set OE (A11) and STR (A15) low.
    gpioa.BSRR.write(|w| w.BR11().set_bit().BR15().set_bit());

    // Set COL1 (A12) to high impedance (input).
    gpioa.MODER.modify(|_, w| w.MODE12().B_0x0());
    // Set COM (PB9) to high impedance.
    gpiob.MODER.modify(|_, w| w.MODE9().B_0x0());

    static DUMMY: UCell<LcdBits> = UCell::new(0);
    let num_xfers = match LCD_BITS {32 => 2, 48 => 3, _ => panic!()};
    dma.CH(DMA_RX).read(unsafe {DUMMY.as_mut().addr()}, num_xfers, 1);

    // We have a 32 bit fifo.
    if USE_TX_DMA {
        dma.CH(DMA_TX).write(bits.addr(), num_xfers, 1);
    }
    else {
        spi.DR.write(|w| w.bits((bits & 0xffff) as u16));
        spi.DR.write(|w| w.bits((bits >> 16) as u16));
    }

    // Wait for RX DMA complete.
    loop {
        WFE();
        if !dma.CH(DMA_RX).CR.read().EN().bit() {
            break;
        }
    }

    // Set STR (A15) high.  Set col1 (A12), it's not driven yet.  (If both S and
    // R are set, then S wins.)
    let col1 = if LCD_BITS > 32 {bits & 1 << 48 != 0} else {false};
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

pub fn lcd_dma_isr() {
    let dma = crate::dma::dma();
    let status = dma.ISR.read();
    dma.IFCR.write(|w| w.bits(status.bits()));
    // Zero-based v. one-based, sigh!
    if status.GIF4().bit() {
        dma.CH(3).CR.write(|w| w.EN().clear_bit());
    }
    if status.GIF5().bit() {
        dma.CH(4).CR.write(|w| w.EN().clear_bit());
    }
}

#[allow(dead_code)]
pub fn backup() {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let pwr   = unsafe {&*stm32u031::PWR  ::ptr()};

    // Backup the IO pins into the standby pullup/down state:
    // OE pin A11, CP=B3, DAT=B5, STR=A15, COM = B9, COL = A12.
    // PWR=PB4.
    let mask_a = 1 << 11 | 1 << 12 | 1 << 15;
    let mask_b = 1 << 3 | 1 << 4 | 1 << 5 | 1 << 9;
    let bits_a = gpioa.IDR.read().bits();
    let bits_b = gpiob.IDR.read().bits();
    // Pull down wins.
    pwr.PUCRA.write(|w| w.bits(mask_a));
    pwr.PUCRB.write(|w| w.bits(mask_b));
    pwr.PDCRA.write(|w| w.bits(mask_a & !bits_a));
    pwr.PDCRB.write(|w| w.bits(mask_b & !bits_b));
}

impl crate::cpu::VectorTable {
    pub const fn lcd_isr(&mut self) -> &mut Self {
        use stm32u031::Interrupt::*;
        self.isr(DMA1_CHANNEL4_5_6_7, lcd_dma_isr)
    }
}

#[test]
fn check_isr() {
    use stm32u031::Interrupt::*;
    use crate::VECTORS;
    assert!(VECTORS.isr[DMA1_CHANNEL4_5_6_7 as usize] == lcd_dma_isr);
    assert!(VECTORS.ahb_clocks() & 1 != 0); // DMA.
    //assert!(VECTORS.apb2_clocks() & 1 << 12 != 0); // SPI1
}
