
use crate::dma::{DMA, Flat};
use crate::vcell::{UCell, WFE};

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

pub const MINUS: u8 = SEG_G;

pub static DIGITS: [u8; 10] = [D0, D1, D2, D3, D4, D5, D6, D7, D8, D9];

impl LCD {
    pub const fn new() -> LCD {LCD{comm: false, segments: 0}}

    pub fn tick(&mut self) {
        self.comm = !self.comm;
        update_lcd(self.segments, self.comm);
    }
}

pub fn init() {
    let dma    = unsafe {&*stm32u031::DMA1  ::ptr()};
    let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    let gpioa  = unsafe {&*stm32u031::GPIOA ::ptr()};
    let gpiob  = unsafe {&*stm32u031::GPIOB ::ptr()};
    let spi    = unsafe {&*stm32u031::SPI1  ::ptr()};

    // OE pin A11, CP=B3, DAT=B5, STR=A15, COM = B9.
    // Set all the SPI control pins to output low.
    // Set PWR=PB4 to be high.
    gpioa.BSRR.write(|w| w.BR11().set_bit().BR15().set_bit());
    gpioa.MODER.modify(|_,w| w.MODE11().B_0x1().MODE15().B_0x1());
    gpiob.BSRR.write(
        |w| w.BR3().set_bit().BS4().set_bit().BR5().set_bit().BR9().set_bit());
    gpiob.MODER.modify(|_,w|
        w.MODE3().B_0x1().MODE4().B_0x1().MODE5().B_0x1().MODE9().B_0x1());

    // Set the function (AF5) for the SPI pins CP (B3), DAT(B5).
    gpiob.AFRL.modify(|_,w| w.AFSEL3().B_0x5().AFSEL5().B_0x5());
    gpiob.MODER.modify(|_,w| w.MODE3().B_0x2().MODE5().B_0x2());

    // Set-up SPI1.
    spi.CR2.write(
        |w| w.DS().B_0xF().SSOE().set_bit()
            .RXDMAEN().set_bit().TXDMAEN().bit(USE_TX_DMA));
    let br = (super::CPU_CLK / 1000000).ilog2() as u8;
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

    let dma   = unsafe {&*stm32u031::DMA1 ::ptr()};
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let spi   = unsafe {&*stm32u031::SPI1 ::ptr()};

    // Set OE (A11) and STR (A15) low.
    gpioa.BSRR.write(|w| w.BR11().set_bit().BR15().set_bit());

    // Set COL1 (A12) to high impedance ("analog").
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
    gpioa.BSRR.write(|w|
        w.BS15().set_bit().BS12().bit((bits & 1 << 48) != 0).BR12().set_bit());

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
    let dma = unsafe {&*stm32u031::DMA1 ::ptr()};
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

impl crate::cpu::VectorTable {
    pub const fn lcd_isr(&mut self) -> &mut Self {
        use stm32u031::Interrupt::*;
        self.isr(DMA1_CHANNEL4_5_6_7, lcd_dma_isr)
    }
}

#[test]
fn check_isr() {
    use stm32u031::Interrupt::*;
    assert!(std::ptr::fn_addr_eq(
        super::VECTORS.isr[DMA1_CHANNEL4_5_6_7 as usize],
        lcd_dma_isr as fn()));
}
