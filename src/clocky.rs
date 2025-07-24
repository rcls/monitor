use crate::*;

use lcd::Segments;

pub const CONFIG: cpu::Config = {
    let mut cfg = cpu::Config::new(2000000);
    cfg.pullup |= 1 << 0x2d;
    cfg.standby_pu |= 1 << 0x2d; // PC13.
    cfg.fll = false;
    *cfg.lazy_debug().lazy_i2c().lcd().rtc().lazy_tsc()};

const MAGIC: u32 = 0xd6ea33e;

pub const I2C_LINES: i2c::I2CLines = i2c::I2CLines::B6_B7;

/// Power of two divider of the 256 RTC clock to use for the main software tick.
const ALARM_BITS: u8 = 5;

/// The overall system state, taking just 9 32-bit words.
#[repr(C)]
struct System {
    /// Magic number.
    magic: u32,
    /// State enumeration.
    state: u32,
    /// Temperature in tenths of °C in low 16 bits.  High bit indicates
    /// measurement outstanding.
    temp: u32,
    /// Sub-state - which field is active when updating date or time.
    sub_state: u32,
    /// State to return to on time-out.
    main_state: u32,
    /// Timer, counting ticks.
    timer: u32,
    _unused: [u32; 2],
    /// Stored address before crash & reboot.
    crash: u32,
}
static_assertions::const_assert!(size_of::<System>() == 9 * 4);

#[allow(dead_code)]
#[repr(u32)]
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
            unsafe{core::mem::transmute(v)}
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
    fn temp(&self) -> i32 {self.temp as i16 as i32}
    /// Also clears pending.
    fn set_temp(&mut self, temp: i16) {self.temp = temp as u16 as u32}
    fn set_temp_pending(&mut self, pending: bool) {
        if pending {
            self.temp |= 1 << 31;
        }
        else {
            self.temp &= !(1 << 31);
        }
    }
    fn is_temp_pending(&self) -> bool {self.temp & 1 << 31 != 0}
}

fn cold_start(sys: &mut System) {
    debug::banner("**** CLOCKY 0x", sys.crash, " ****");
    let state = sys.main_state();
    sys.set_state(state);


    sys.magic = MAGIC;
    rtc::ensure_options();
    rtc_setup_tick();

    let state = sys.main_state();
    sys.set_state(state);

    tmp117::init(state == State::Temp);
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

fn date_time_to_segments(mut t: u32, dots: Segments, kill3: bool) -> Segments {
    let mut result = [0; size_of::<Segments>()];
    for p in 0 .. LCD_BITS / 8 {
        let d = t as usize & 15;
        t >>= 4;
        result[p] = lcd::DIGITS[d];
    }
    if LCD_BITS == 48 && result[5] == lcd::DIGITS[0] {
        result[5] = 0;
    }
    if (kill3 || LCD_BITS == 32) && result[3] == lcd::DIGITS[0] {
        result[3] = 0;
    }
    if kill3 && LCD_BITS == 32 && result[1] == lcd::DIGITS[0] {
        result[1] = 0;
    }
    Segments::from_le_bytes(result) | dots
}

fn time_to_segments(t: u32) -> Segments {
    let t = if LCD_BITS == 32 {t >> 8} else {t};
    date_time_to_segments(t, lcd::COL1 | lcd::COL2, false)
}

fn date_to_segments(d: u32) -> Segments {
    let mult = if LCD_BITS <= 32 {1} else {0x10001};
    let dots = mult * 0x10000 + lcd::COL1 + lcd::COL2;
    // Mask off day-of-week.
    let d = (d & !0xe000).swap_bytes();
    let d = if LCD_BITS == 32 {d >> 16} else {d >> 8};
    date_time_to_segments(d, dots, true)
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
        Temp => temp_to_segments(sys.temp())
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

fn tick(sys: &mut System) {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};

    let sr = rtc.MISR.read();
    rtc.SCR.write(|w| w.bits(sr.bits()));

    let segments = get_segments(sys);
    let ssr = rtc.SSR.read().bits();

    let mut tacquire = false;
    
    // If we're displaying the temperature, trigger a conversion every second.
    // If we're not displaying it, then grab it every minute.
    if ssr >= 0xfc {
        tacquire = sys.state() == State::Temp
            || rtc.TR.read().bits() & 0xff == 0;
    }
    if sys.is_temp_pending() {
        tacquire = false;
        sys.set_temp_pending(false);
    }

    let wait = if tacquire {tmp117::acquire()} else {i2c::Wait::new()};

    lcd::init();

    lcd::update_lcd(segments, ssr & 1 << ALARM_BITS != 0);

    let _ = wait.wait();

    if false {
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
}

fn temp_alert(sys: &mut System) {
    let temp = tmp117::alert();
    sys.set_temp(temp as i16);
}

pub fn main() -> ! {
    let pwr = unsafe {&*stm32u031::PWR::ptr()};
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    let sys = unsafe {System::get()};

    // Enable IO clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit().GPIOBEN().set_bit());

    cpu::init1();
    cpu::init2();

    let rcc_csr = rcc.CSR.read().bits();

    let sr1 = pwr.SR1.read();

    // If all of the reset reason, the magic number, and the woke-from-standby
    // flag agree, treat this as a normal restart.  Else do a cold-start.
    if rcc_csr & 0xfe000000 != 0 || sys.magic != MAGIC
        || !sr1.SBF().bit() {
        cold_start(sys);
    }

    if false {
        tsc::init();
    }

    // Wake-up 2 is the TMP117 alert.
    if sr1.WUF2().bit() {
        pwr.SCR.write(|w| w.CWUF2().set_bit());
        temp_alert(sys);
    }

    if sr1.WUFI().bit() {
        tick(sys);
    }

    rtc::standby(true);
}

#[cfg(test)]
fn segments_to_str(s: Segments) -> String {
    let mut result = Vec::<char>::new();
    let n = LCD_BITS / 8 as usize;
    use lcd::DOT;
    for b in &s.to_le_bytes()[0..n] {
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
    for (i, b) in s.to_le_bytes()[0..n].iter().enumerate().skip(1).rev() {
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
        (0x691206, " 6⋮12", " 6⋮12⋮69"),
        (0x690616, "16⋮ 6", "16⋮ 6⋮69"),
        (0x010101, " 1⋮ 1", " 1⋮ 1⋮01"),
        (0x01f101, " 1⋮11", " 1⋮11⋮01"),
    ];
    for (d, s, w) in checks {
        assert_eq!(segments_to_str(date_to_segments(d)),
                   if LCD_BITS == 32 {s} else {w});
    }
}

#[test]
fn time_segment_checks() {
    let checks = [
        (0x010203, " 1:02", " 1:02:03"),
        (0x160659, "16:06", "16:06:59"),
        (0x010101, " 1:01", " 1:01:01"),
    ];
    for (d, s, w) in checks {
        assert_eq!(segments_to_str(time_to_segments(d)),
                   if LCD_BITS == 32 {s} else {w});
    }
}
