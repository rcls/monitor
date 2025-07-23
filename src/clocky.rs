#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![allow(internal_features)]
#![allow(unpredictable_function_pointer_comparisons)]
#![feature(const_default)]
#![feature(const_trait_impl)]
#![feature(derive_const)]
#![feature(format_args_nl)]
#![feature(link_llvm_intrinsics)]

mod cpu;
mod debug;
#[allow(dead_code)]
mod dma;
#[allow(dead_code)]
mod i2c;
mod lcd;
mod rtc;
mod tsc;
mod utils;
mod vcell;

use lcd::Segments;

const LCD_BITS: usize = 32;

const CONFIG: cpu::Config = *cpu::Config::new(2000000)
    .lazy_debug().lazy_i2c().lcd().rtc().lazy_tsc();

const MAGIC: u32 = 0xd6ea33e;

const I2C_LINES: i2c::I2CLines = i2c::I2CLines::B6_B7;

#[repr(C)]
struct System {
    magic: u32,
    state: u32,
    sub_state: u32,
    main_state: u32,
    timer: u32,
    temp: i32,
    _unused: [u32; 4],
    crash: u32,
}

#[allow(dead_code)]
#[repr(u8)]
#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Time = 0,
    Date = 1,
    Temp = 2,
    SetTime = 3,
    SetDate = 4,
}

impl From<u32> for State {
    fn from(v: u32) -> State {
        if v <= State::SetDate as u32 {
            unsafe{core::mem::transmute(v as u8)}
        }
        else {
            State::Time
        }
    }
}

impl State {
    fn main(self) -> State {
        use State::*;
        match self {Time|Date|Temp => self, _ => Time}
    }
}

const ALARM_BITS: u8 = 5;

impl System {
    unsafe fn get() -> &'static mut Self {
        let tamp = unsafe {&*stm32u031::TAMP::ptr()};
        let p = tamp.BKPR[0].as_ptr() as *const crate::vcell::UCell<System>;
        unsafe {(&*p).as_mut()}
    }
    fn state(&self) -> State {
        self.state.into()
    }
    fn set_state(&mut self, s: State) {
        self.state = s as u32;
    }
    fn main_state(&self) -> State {
        State::from(self.main_state).main()
    }
    fn set_main_state(&mut self, s: State) {
        self.main_state = s as u32;
    }
    fn sub_state(&self) -> u8 {
        self.sub_state as u8
    }
    fn set_sub_state(&mut self, v: u8) {
        self.sub_state = v as u32;
    }
    fn blink_mask(&self) -> Segments {
        let s = self.state();
        if s != State::SetTime && s != State::SetDate {
            return 0;
        }
        let dd = lcd::D8 as Segments * 0x101;
        match self.sub_state() as usize {
            1 => dd << LCD_BITS - 16,
            2 => dd << LCD_BITS - 32,
            3 if LCD_BITS > 32 => dd,
            _ => dd | dd << 16 | dd << if LCD_BITS > 32 {32} else {0}
        }
    }
    fn start_timer(&mut self) {
        self.timer = 0;
    }
    fn tick_timer(&mut self, timeout: u32) -> bool {
        if self.timer >= timeout {
            return true;
        }
        self.timer += 1;
        false
    }
}

fn cold_start(state: &mut System) {
    dbgln!("**** CLOCKY {:#x} ****", state.crash);
    i2c::init();
    // Power down the TMP117.
    let wait = i2c::write(i2c::TMP117, &[1u8, 0, 4]);
    state.magic = MAGIC;
    rtc::ensure_options();
    rtc_setup_tick();

    state.set_state(state.main_state());
    let _ = wait.wait();
}

fn rtc_setup_tick() {
    rtc::setup_start();
    // Set the alarm A to fire (8>>bits) times / sec.
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    rtc.CR.write(|w| w.ALRAE().clear_bit());

    rtc.ALRMAR.write(|w| w.bits(!0));
    rtc.ALRMASSR.write(
        |w| w.MASKSS().bits(ALARM_BITS)
            . SS().bits((1 << ALARM_BITS) - 1));

    rtc.CR.write(
        |w| w.ALRAE().set_bit().ALRAIE().set_bit().BYPSHAD().set_bit());
    rtc::rtc_setup_end();
}

fn date_time_to_segments(t: u32, dots: Segments) -> Segments {
    let mut result = [0; size_of::<Segments>()];
    let mut t = t;
    for p in 0 .. LCD_BITS / 8 {
        let d = t as usize & 15;
        t >>= 4;
        result[p] = lcd::DIGITS[d];
    }
    if result[LCD_BITS / 8 - 1] == lcd::DIGITS[0] {
        result[LCD_BITS / 8 - 1] = 0;
    }
    Segments::from_le_bytes(result) | dots
}

fn time_to_segments(t: u32) -> Segments {
    date_time_to_segments(t, lcd::COL1 | lcd::COL2)
}

fn date_to_segments(d: u32) -> Segments {
    let dots = lcd::DOT as Segments * 0x10000;
    let dots = if LCD_BITS <= 32 {dots} else {dots * 0x10001};
    // Mask off day-of-week.
    let d = d & !0xe000;
    // Byte swap.
    let d = d.swap_bytes() >> 8;
    let d = if LCD_BITS == 32 {d >> 8} else {d};
    date_time_to_segments(d, dots)
}

fn temp_to_segments(temp: i32) -> Segments {
    const N: usize = LCD_BITS / 8;
    let mut segs = [0; size_of::<Segments>()];
    let mut p = 0;
    let mut quo = temp.unsigned_abs() as u16; // u16 is better on Cortex-M0.
    while p < N && quo != 0 || p < 2 {
        let rem = quo % 10;
        quo /= 10;
        segs[p] = lcd::DIGITS[rem as usize];
        p += 1;
    }
    if p < N && temp < 0 {
        segs[p] = lcd::MINUS;
        p += 1;
    }
    let mut segs = Segments::from_le_bytes(segs) | lcd::DOT as Segments * 256;
    if p < N {
        segs = segs * 256 + lcd::DEG as Segments;
    }
    if LCD_BITS > 32 && p <= 3 {
        segs *= 256;
    }
    segs
}

fn get_segments(sys: &System) -> Segments {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    use State::*;
    let segments = match sys.state() {
        Time|SetTime => time_to_segments(rtc.TR().read().bits()),
        Date|SetDate => date_to_segments(rtc.DR().read().bits()),
        Temp => temp_to_segments(sys.temp)
    };
    match sys.state() {
        Time|Date|Temp => return segments,
        _ => (),
    };
    if rtc.SSR.read().bits() & 0x40 == 0 {
        return segments;
    }
    let dd = !lcd::DOT as Segments * 0x101;
    match sys.sub_state() {
        1 => segments & !dd,
        2 => segments & !(dd << 16),
        3 if LCD_BITS > 32 => segments & !(dd << 32),
        _ => segments & (lcd::DOT as Segments * 0x10101010101u64 as Segments)
    }
}

fn main() -> ! {
    let pwr = unsafe {&*stm32u031::PWR::ptr()};
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    let state = unsafe {System::get()};

    // Enable IO clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit().GPIOBEN().set_bit());

    cpu::init1();
    cpu::init2();

    let rcc_csr = rcc.CSR.read().bits();

    let sr1 = pwr.SR1.read();

    // If all of the reset reason, the magic number, and the woke-from-standby
    // flag agree, treat this as a normal restart.  Else do a cold-start.
    if rcc_csr & 0xfe000000 != 0 || state.magic != MAGIC
        || !sr1.SBF().bit() {
        cold_start(state);
    }

    if false {
        tsc::init();
    }

    let sr = rtc.MISR.read();
    rtc.SCR.write(|w| w.bits(sr.bits()));

    let segments = get_segments(state);
    lcd::init();

    let ssr = rtc.SSR.read().bits();
    lcd::update_lcd(segments, ssr & 1 << ALARM_BITS != 0);

    if ssr > 0xff {
        let time1 = rtc.TR.read().bits() as usize;
        let date = rtc.DR.read().bits();
        let time = rtc.TR.read().bits() as usize;
        let date = if time == time1 {date} else {rtc.DR.read().bits()};
        dbgln!("{:02x} {:02x} {:02x} ({:x}) {:02x} {:02x} {:02x}",
               date >> 16 & 0xff, date >> 8 & 0x1f, date & 0xff,
               date >> 13 & 7,
               time >> 16 & 0xff, time >> 8 & 0xff, time & 0xff);
    }
//    rtc.SCR.write(|w| w.bits(sr.bits()));

    rtc::standby(true);
}

#[cfg(test)]
fn segments_to_str(s: Segments) -> String {
    let mut result = Vec::<char>::new();
    let N = LCD_BITS / 8 as usize;
    use lcd::DOT;
    for b in &s.to_le_bytes()[0..N] {
        let b = b & !DOT;
        let c = match b {
            lcd::DEG => '°',
            lcd::MINUS => '-',
            0 => ' ',
            _ => char::from_u32(
                48 + lcd::DIGITS.iter().position(|x| *x == b).unwrap() as u32)
                .unwrap(),
        };
        result.push(c);
    }
    for (i, b) in s.to_le_bytes()[0..N].iter().enumerate().skip(1).rev() {
        let colon = match i {
            2 => s & lcd::COL1 != 0,
            4 => s & lcd::COL2 != 0,
            _ => false};
        let sep = if b & lcd::DOT != 0 {
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
                   if LCD_BITS == 32 {s} else {w});
    }
}

#[test]
fn date_segment_checks() {
    let checks = [
        (0x691206, " 6.12", " 6.12.69"),
        (0x690616, "16.06", "16.06.69"),
    ];
    for (d, s, w) in checks {
        assert_eq!(segments_to_str(date_to_segments(d)),
                   if LCD_BITS == 32 {s} else {w});
    }
}
