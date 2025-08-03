#![allow(non_upper_case_globals)]

use crate::cpu::WFE;

pub const WIDTH: usize = crate::LCD_WIDTH;

pub const SEG_A: u8 = 32;
pub const SEG_B: u8 = 16;
pub const SEG_C: u8 = 2;
pub const SEG_D: u8 = 4;
pub const SEG_E: u8 = 8;
pub const SEG_F: u8 = 64;
pub const SEG_G: u8 = 128;
pub const DOT: u8 = 1;

pub const D0: u8 = D8 & !MINUS;
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
pub const DC: u8 = D0 & !SEG_B & !SEG_C;
pub const DA: u8 = D8 & !SEG_D;
pub const DL: u8 = DC & !SEG_A;

pub const MINUS: u8 = SEG_G;
/// Right or only colon.
pub const COL2: Segments = 1;
/// Left colon on wide display.
pub const COL1: Segments = if WIDTH == 6 {1 << 48} else {0};

pub trait SegmentsTrait<const N: usize> {type Segments;}
impl SegmentsTrait<6> for () {type Segments = u64;}
impl SegmentsTrait<4> for () {type Segments = u32;}
pub type Segments = <() as SegmentsTrait<{WIDTH}>>::Segments;
pub type SegArray = [u8; size_of::<Segments>()];

pub static DIGITS: [u8; 16] = [
    D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, 0, 0, 0, 0, 0, 0];

/// Initialize the I/O to the LCD.
pub fn init() {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let spi   = unsafe {&*stm32u031::SPI1 ::ptr()};

    const {assert!(crate::CONFIG.apb2_clocks & 1 << 12 != 0)};

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
    if WIDTH == 6 {
        rx_word(spi);
        spi.DR.write(|w| w.bits((bits >> 16 >> 16) as u16));
    }
    // Wait for 2 words RX.
    rx_word(spi);
    rx_word(spi);

    // Set STR (A15) high.  Set col1 (A12), it's not driven yet.  (If both S and
    // R are set, then S wins.)
    let col1 = if WIDTH == 6 {bits as u64 & 1 << 48 != 0} else {comm};
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

pub fn date_time_to_segments(mut t: u32, dots: Segments, kz: bool) -> Segments {
    let mut result = [0; size_of::<Segments>()];
    for p in result.iter_mut().take(WIDTH) {
        let d = t as usize & 15;
        t >>= 4;
        *p = DIGITS[d];
    }
    if WIDTH == 6 && result[WIDTH - 1] == DIGITS[0] {
        result[WIDTH - 1] = 0;
    }
    if (kz || WIDTH == 4) && result[3] == DIGITS[0] {
        result[3] = 0;
    }
    if kz && WIDTH == 4 && result[1] == DIGITS[0] {
        result[1] = 0;
    }
    Segments::from_le_bytes(result) | dots
}

pub fn time_to_segments(t: u32, sub_state: u32) -> Segments {
    let t = if WIDTH == 6 || sub_state == 3 {t} else {t >> 8};
    let segs = date_time_to_segments(t, COL1 | COL2, false);
    if WIDTH == 6 || sub_state != 3 {
        segs
    }
    else {
        segs & 0xffff
    }
}

pub fn date_to_segments(d: u32, sub_state: u32) -> Segments {
    let mult = if WIDTH == 4 {1} else {0x10001};
    let dots = mult * 0x10000 + COL1 + COL2;
    // Mask off day-of-week.
    let d = (d & !0xe000).swap_bytes();
    let d = if WIDTH == 4 && sub_state != 1 {d >> 16} else {d >> 8};
    if WIDTH == 4 && sub_state == 1 {
        date_time_to_segments(d, dots, false) & 0x1ffff
    }
    else {
        date_time_to_segments(d, dots, true)
    }
}

pub fn temp_to_segments(temp: i32) -> Segments {
    let mut segs = SegArray::default();
    let p = decimal_to_segments(&mut segs, temp, 2);
    let mut segs = Segments::from_le_bytes(segs) | DOT as Segments * 256;
    if p < WIDTH {
        segs = segs * 256 + DEG as Segments;
    }
    if WIDTH == 6 && p <= 3 {
        segs *= 256;
    }
    segs
}

/// Works on 17 bit signed, -65535 ..= 65535.
pub fn decimal_to_segments(segs: &mut SegArray, v: i32, min: usize) -> usize {
    let mut i = segs.iter_mut();
    let mut count = 0;
    let mut quo = v.unsigned_abs() as u16;
    loop {
        let rem = quo % 10;
        quo /= 10;
        let Some(p) = i.next() else {return count};
        *p = DIGITS[rem as usize];
        count += 1;
        if quo == 0 && count >= min {
            break;
        }
    }
    if v < 0 && let Some(p) = i.next() {
        *p = MINUS;
        count += 1;
    }
    count
}

pub fn cal_to_segments(cal: i32) -> Segments {
    let mut segs = SegArray::default();
    decimal_to_segments(&mut segs, cal, 0);
    Segments::from_le_bytes(segs) | (DC as Segments) << WIDTH * 8 - 8
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

#[cfg(test)]
fn segments_to_str(s: Segments) -> String {
    let mut result = Vec::<char>::new();
    for b in &s.to_le_bytes()[0..WIDTH] {
        let b = b & !DOT;
        let c = match b {
            DEG => '°',
            MINUS => '-',
            DC => 'C',
            0 => ' ',
            _ => char::from_u32(
                48 + DIGITS.iter().position(|x| *x == b).unwrap() as u32)
                .unwrap(),
        };
        result.push(c);
    }
    for (i, b) in s.to_le_bytes()[0..WIDTH].iter().enumerate().skip(1).rev() {
        let colon = match i {
            2 => s & COL2 != 0,
            4 => s & COL1 != 0,
            _ => false};
        let sep = if b & DOT != 0 {
            if colon {'⋮'} else {'.'}
        }
        else if colon {':'} else {continue};
        result.insert(i, sep);
    }
    result.iter().rev().collect()
}

#[test]
fn temp_segment_checks() {
    let checks = [
        (   6, " 0.6°", "  0.6° "),
        (  89, " 8.9°", "  8.9° "),
        ( 123, "12.3°", " 12.3° "),
        (4567, "456.7", " 456.7°"),
        (  -6, "-0.6°", " -0.6° "),
        ( -89, "-8.9°", " -8.9° "),
        (-123, "-12.3", " -12.3°"),
        (-4567, "456.7", "-456.7°")];
    for (t, s, w) in checks {
        assert_eq!(segments_to_str(temp_to_segments(t)),
                   if WIDTH == 4 {s} else {w});
    }
}

#[test]
fn date_segment_checks() {
    let checks = [
        (0x691206, " 6⋮12", " 6⋮12⋮69"),
        (0x690616, "16⋮ 6", "16⋮ 6⋮69"),
        (0x010101, " 1⋮ 1", " 1⋮ 1⋮01"),
        (0x01f101, " 1⋮11", " 1⋮11⋮01"),
    ];
    for (d, s, w) in checks {
        assert_eq!(segments_to_str(date_to_segments(d, 0)),
                   if WIDTH == 4 {s} else {w});
    }
}

#[test]
fn time_segment_checks() {
    for h in 0..24u32 {
        for m in 0..60 {
            for s in 0.. 60 {
                let xx = |x| x % 10 + x / 10 * 16;
                let t = xx(h) * 65536 + xx(m) * 256 + xx(s);
                let s = if WIDTH == 6 {format!("{h:2}:{m:02}:{s:02}")}
                    else {format!("{h:2}:{m:02}")};
                assert_eq!(segments_to_str(time_to_segments(t, 0)), s);
            }
        }
    }
}

#[test]
fn date_adjust_date_checks() {
    if WIDTH == 6 {
        for i in 0 ..= 3 {
            assert_eq!(segments_to_str(date_to_segments(0x691204, i)),
                       " 4⋮12⋮69");
        }
        return;
    }
    assert_eq!(segments_to_str(date_to_segments(0x691204, 2)),
               " 4⋮12");
    assert_eq!(segments_to_str(date_to_segments(0x691204, 3)),
               " 4⋮12");
    assert_eq!(segments_to_str(date_to_segments(0x691204, 1)),
               "  ⋮69");
    assert_eq!(segments_to_str(date_to_segments(0x691204, 1)),
               "  ⋮69");
    assert_eq!(segments_to_str(date_to_segments(0x071204, 1)),
               "  ⋮07");
}

#[test]
fn check_decimal() {
    fn expected(v: i32, min: usize) -> String {
        let min = min.max(1);
        let mut s = format!("{:0min$}", v.unsigned_abs());
        if v < 0 {
            s.insert(0, '-');
        }
        while s.len() < WIDTH {
            s.insert(0, ' ');
        }
        if s.len() > WIDTH {
            s.replace_range(.. s.len() - WIDTH, "");
        }
        s
    }
    for i in -65535..=65535 {
        for min in 0..=2 {
            let mut segs = SegArray::default();
            let p = decimal_to_segments(&mut segs, i, min);
            assert!(p > 0);
            assert!(p == segs.len() || segs[p] == 0);
            assert!(segs[p - 1] != 0);
            let segs = Segments::from_le_bytes(segs);
            assert_eq!(segments_to_str(segs), expected(i, min));
        }
    }
}

#[test]
fn check_cal() {
    for i in -99 ..= 999 {
        assert_eq!(segments_to_str(cal_to_segments(i)),
                   format!("C{i:w$}", w = WIDTH - 1));
    }
}