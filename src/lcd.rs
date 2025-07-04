
pub static LCD: crate::vcell::UCell<LCD> = crate::vcell::UCell::new(LCD::new());

pub struct LCD {
    comm: bool,
    pub segments: u64,
}

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
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let spi   = unsafe {&*stm32u031::SPI1 ::ptr()};

    // Set all the SPI control pins to output low.
    // OE pin A11, CP=B3, DAT=B5, STR=A15, COM = B9.
    // Set PWR=PB4 to be high, delay driving it for a moment.
    gpioa.BSRR.write(|w| w.BR11().set_bit().BR15().set_bit());
    gpioa.MODER.modify(|_,w| w.MODE11().B_0x1().MODE15().B_0x1());
    gpiob.BSRR.write(
        |w| w.BR3().set_bit().BS4().set_bit().BR5().set_bit().BR9().set_bit());
    gpiob.MODER.modify(|_,w|
        w.MODE3().B_0x1().MODE4().B_0x1().MODE5().B_0x1().MODE9().B_0x1());

    // Set the SR power to output. (PB4).
    //gpiob.BSRR.write(|w| w.BS4().set_bit());
    //gpiob.MODER.modify(|_,w| w.MODE4().B_0x1());

    // Set the function (AF5) for the SPI pins CP (B3), DAT(B5).
    gpiob.AFRL.modify(|_,w| w.AFSEL3().B_0x5().AFSEL5().B_0x5());
    gpiob.MODER.modify(|_,w| w.MODE3().B_0x2().MODE5().B_0x2());

    // Set-up SPI1 ...
    // /256 clock for now.
    // MODE0 should be fine.
    // 16 bit
    spi.CR2.write(|w| w.DS().B_0xF().SSOE().set_bit());
    spi.CR1.write(
        |w| w.LSBFIRST().set_bit().SPE().set_bit().BR().B_0x7()
            .MSTR().set_bit());
}

pub fn update_lcd(bits: u64, comm: bool) {
    // Invert segment bits if comm is high.
    let bits = if comm {!bits} else {bits};

    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let spi   = unsafe {&*stm32u031::SPI1 ::ptr()};

    // Set OE (A11) and STR (A15) low.
    gpioa.BSRR.write(|w| w.BR11().set_bit().BR15().set_bit());

    // Set COL1 (A12) to high impedance ("analog").
    gpioa.MODER.modify(|_, w| w.MODE12().B_0x0());
    // Set COM (PB9) to high impedance.
    gpiob.MODER.modify(|_, w| w.MODE9().B_0x0());

    // Output bits (except for COL1) via SPI.  We have a fifo, no need to
    // be prissy.
    spi.DR.write(unsafe{|w| w.bits((bits & 0xffff) as u16)});
    while spi.SR.read().FTLVL().bits() > 1 {
    }
    spi.DR.write(unsafe{|w| w.bits((bits >> 16) as u16)});
    while spi.SR.read().FTLVL().bits() > 1 {
    }
    spi.DR.write(unsafe{|w| w.bits((bits >> 32) as u16)});

    // FIXME - bit order.
    // FIXME... poll for SPI properly.  Check all words received.
    while spi.SR.read().BSY().bit() {
    }
    //sdbgln!("WTF, lets wait a bit more");

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
