use crate::*;

use lcd::{BITS, Segments};
use tsc::pad;

pub const TOUCH_EVALUATE: bool = false;

pub const CONFIG: cpu::Config = {
    let mut cfg = cpu::Config::new(2000000);
    cfg.pullup |= 1 << 0x2d;
    cfg.standby_pu |= 1 << 0x2d; // PC13.
    cfg.fll = false;
    *cfg.lazy_debug().lazy_i2c().lcd().rtc().lazy_tsc()};

const MAGIC: u32 = 0xd6ea33e;

pub const I2C_LINES: i2c::I2CLines = i2c::I2CLines::B6_B7;

/// Power of two divider of the 256Hz RTC clock to use for the main tick.
const TICKS_PER_SEC: u32 = 8;
const ALARM_BITS: u8 = 8 - TICKS_PER_SEC.ilog2() as u8;

const TOUCH_TIME_OUT: u32 = 10 * TICKS_PER_SEC;
const REPEAT_FIRST: u32 = TICKS_PER_SEC;
const REPEAT_AGAIN: u32 = TICKS_PER_SEC / 2;

/// The overall system state, taking just 9 32-bit words.
#[repr(C)]
#[derive(Default)]
struct System {
    /// Magic number.
    magic: u32,
    /// State enumeration.
    state: u32,
    /// Temperature in tenths of °C in low 16 bits.
    temp: i32,
    /// Sub-state - which field is active when updating date or time.
    sub_state: u32,
    /// Current touch state. 0:None, 1:+, 2:-, 3:≡
    touch: u32,
    /// Timer, counting ticks.  Down counter running while touch system
    /// activated.
    timer: u32,
    scratch: u32,
    _unused: u32,
    /// Stored address before crash & reboot.
    crash: u32,
}

#[allow(dead_code)]
#[repr(u32)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd)]
enum State {
    Temp = 0,
    Time = 1,
    Date = 2,
    Cal  = 3,
}

/// Cal substates.
mod cal {
    /// Clock calibration.
    pub const CAL     : u32 = 1;
    pub const DRIVE   : u32 = 2;
    pub const CRASH_HI: u32 = 3;
    pub const CRASH_LO: u32 = 4;
    pub const MAX     : u32 = 4;
}

impl State {
    const MIN: State = State::Temp;
    const MAX: State = State::Cal;
}

impl System {
    unsafe fn get() -> &'static mut Self {
        const {assert!(size_of::<System>() == 9 * 4)};
        let tamp = unsafe {&*stm32u031::TAMP::ptr()};
        let p = tamp.BKPR[0].as_ptr() as *const crate::vcell::UCell<System>;
        let sys = unsafe {(&*p).as_mut()};
        // Validate fields.
        if sys.state > State::MAX as u32 {
            sys.state = State::Temp as u32;
        }
        sys
    }

    fn init(&mut self) {
        self.magic = MAGIC;
        self.temp = self.temp.clamp(-1999, 1999);
        self.sub_state = 0;
        self.touch = 0;
        self.timer = 0;
    }
    fn state(&self) -> State {
        // SAFTEY: We sanitize this at each start-up.
        unsafe {core::mem::transmute(self.state)}
    }
    fn blink_mask(&self) -> Segments {
        let dd = lcd::D8 as Segments * 0x101;
        let state = self.state();
        use State::*;
        if state == Cal {
            if self.sub_state <= 2 {
                return (1 << BITS - 8) - 1;
            }
            else {
                return 255 << BITS - 16;
            }
        }
        // The numbering is big endian, time display is big endian, date
        // display is little endian.  For a 4 digit display, the exception
        // is always on the right.
        let pos = if state == Date {4 - self.sub_state} else {self.sub_state};
        match pos {        // Date or time.
            1 => dd << BITS - 16,
            2 => dd << BITS - 32,
            3 => dd,
            _ => 0,
        }
    }
}

fn cold_start(sys: &mut System) {
    let pwr = unsafe {&*stm32u031::PWR::ptr()};
    // Enable RTC wakeup.  (It should already be enabled).  The TMP117 alert
    // is on PC13, wake-up 2, enable that.
    pwr.CR3.write(|w| w.APC().set_bit().EIWUL().set_bit().EWUP2().set_bit());
    // Clear the wake-up pin now as it has probably gotten set during start-up.
    // We will trigger a temperature conversion immediately, and
    pwr.SCR.write(|w| w.CWUF2().set_bit());

    // Turn on the VBATT charging.  Set the wakeup 2 polarity at the same time.
    pwr.CR4.write(|w| w.VBE().set_bit().VBRS().set_bit().WP2().set_bit());

    debug::banner("**** CLOCKY 0x", sys.crash, " ****\n");
    sys.init();

    rtc::ensure_options();
    rtc_setup_tick();

    tmp117::init();
}

fn rtc_setup_tick() {
    rtc::setup_start();

    const {assert!(TICKS_PER_SEC == 1 << 8 - ALARM_BITS)};

    // Set the alarm A to fire (8>>bits) times / sec.
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    rtc.CR.write(|w| w.ALRAE().clear_bit());

    rtc.ALRMAR.write(|w| w.bits(!0));
    rtc.ALRMASSR.write(
        |w| w.MASKSS().bits(ALARM_BITS)
            . SS().bits((1 << ALARM_BITS) - 1));

    rtc.CR.write(
        |w| w.ALRAE().set_bit().ALRAIE().set_bit().BYPSHAD().set_bit());
    rtc::relock();
}

fn cal_segments(sys: &System) -> Segments {
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    use lcd::{DA, DC, Dd, DE, DH, DL};
    match sys.sub_state {
        cal::CAL => lcd::cal_to_segments(lcd::DC, rtc::get_cal()),
        cal::DRIVE => lcd::cal_to_segments(
            Dd, rcc.BDCR.read().LSEDRV().bits().into()),
        cal::CRASH_LO => lcd::hex_to_segments(sys.crash, LCD_WIDTH - 2)
            | (DE as Segments * 256 + DL as Segments) << BITS - 16,
        cal::CRASH_HI => lcd::hex_to_segments(
            sys.crash >> LCD_WIDTH * 4 - 8, LCD_WIDTH - 2)
            | (DE as Segments * 256 + DH as Segments) << BITS - 16,
        _ => (DC as Segments * 65536 + DA as Segments * 256 + DL as Segments)
            << BITS - 24
    }
}

fn get_segments(sys: &System) -> Segments {
    if TOUCH_EVALUATE {
        let mut segs = lcd::SegArray::default();
        lcd::decimal_to_segments(&mut segs, sys.scratch as i32, 0);
        return Segments::from_le_bytes(segs);
    }
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    use State::*;
    let segments = match sys.state() {
        Time => lcd::time_to_segments(rtc.TR().read().bits(), sys.sub_state),
        Date => lcd::date_to_segments(rtc.DR().read().bits(), sys.sub_state),
        Temp => lcd::temp_to_segments(sys.temp),
        Cal  => cal_segments(sys),
    };
    if sys.sub_state == 0 || rtc.SSR.read().bits() & 0x40 != 0 {
        segments
    }
    else {
        segments & !sys.blink_mask()
    }
}

fn touch_state(sys: &mut System, pad: u32) -> bool {
    let previous = sys.touch;
    sys.touch = pad;
    if pad != previous {
        sys.timer = if pad != pad::NONE {REPEAT_FIRST} else {TOUCH_TIME_OUT};
        pad != 0
    }
    else {
        let t = sys.timer;
        sys.timer = if t > 1 {t - 1} else {match pad {
            pad::NONE => 0,
            pad::MENU => REPEAT_FIRST,
            _ => REPEAT_AGAIN,
        }};
        t == 1
    }
}

fn touch_process(sys: &mut System) {
    let (pad, val) = tsc::retrieve();
    if TOUCH_EVALUATE {
        sys.scratch = val;
        return;
    }

    if !touch_state(sys, pad) {
        return;
    }

    match pad {
        pad::NONE => sys.sub_state = 0, // Cancel any blinky stuff.
        pad::MENU => {                  // Rotate sub-state.
            let s = sys.state();
            let ss = sys.sub_state;
            let max
                = match s {State::Temp => 0, State::Cal => cal::MAX, _ => 3};
            sys.sub_state = if ss >= max {0} else {ss + 1};
        },
        _ => touch_plus_minus(sys, pad),
    }

    // Update the cal clock output.
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    if sys.state() == State::Cal && sys.sub_state != 0 {
        rcc.BDCR.modify(
            |_, w| w.LSCOSEL().set_bit().LSCOEN().set_bit().LSESYSEN().set_bit());
    }
    // FIXME - when exactly do we turn off?
    else if sys.state() != State::Cal {
        rcc.BDCR.modify(
            |_, w| w.LSCOEN().clear_bit().LSESYSEN().clear_bit());
    }
}

fn touch_plus_minus(sys: &mut System, pad: u32) {
    let s = sys.state();

    if sys.sub_state == 0 {
        // Rotate the main state.
        sys.state = if pad == pad::PLUS {
            if s >= State::MAX {State::MIN as u32} else {s as u32 + 1}
        }
        else {
            if s <= State::MIN {State::MAX as u32} else {s as u32 - 1}
        };
        return;
    }

    if s == State::Cal {
        return cal_plus_minus(sys, pad);
    }

    if s == State::Temp {
        return; // Nothing to do (should have ss==0 anyway).
    }

    // Date or Time.  The sub-state numbering is big endian 1-based, the
    // forwards/backward numbering is little endian 0-based.
    let item = if s == State::Time {3} else {6} - sys.sub_state;
    if pad == pad::PLUS {
        rtc::forwards(item);
    }
    if pad == pad::MINUS {
        rtc::backwards(item);
    }
}

fn cal_plus_minus(sys: &mut System, pad: u32) {
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    match sys.sub_state {
        cal::CAL => {
            let cal = rtc::get_cal();
            rtc::set_cal(
                if pad == pad::PLUS {cal.min(98) + 1} else {cal.max(-98) - 1});
        }
        cal::DRIVE => {
            let bdcr = rcc.BDCR.read();
            let d = bdcr.LSEDRV().bits();
            let d = if pad == pad::PLUS {d.min(2) + 1} else {d.max(2) - 1};
            rcc.BDCR.write(|w| w.bits(bdcr.bits()).LSEDRV().bits(d));
        }
        _ => (),
    }
}

fn tick(sys: &mut System) {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};

    let sr = rtc.MISR.read();
    rtc.SCR.write(|w| w.bits(sr.bits()));

    let ssr = rtc.SSR.read().bits();

    let tacquire;
    if ssr >= 0xfc {
        // If we're displaying the temperature, trigger a conversion every second.
        // If we're not displaying it, then grab it every minute.
        tacquire = sys.state() == State::Temp
            || rtc.TR.read().bits() & 0xff == 0;
    }
    else {
        tacquire = false;
    }

    let wait = if tacquire {tmp117::acquire()} else {i2c::Wait::new()};

    let do_touch = sys.timer != 0 || ssr & 0x70 == 0x70;
    if do_touch {
        tsc::init();
        tsc::acquire();
    }

    lcd::init();
    let segments = get_segments(sys);
    lcd::update_lcd(segments, ssr & 1 << ALARM_BITS != 0);

    let _ = wait.wait();

    if do_touch {
        touch_process(sys);
    }
}

pub fn main() -> ! {
    let gpioc = unsafe {&*stm32u031::GPIOC::ptr()};
    let pwr   = unsafe {&*stm32u031::PWR  ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    let sys   = unsafe {System::get()};

    // Enable IO clocks.
    rcc.IOPENR.write(
        |w| w.GPIOAEN().set_bit().GPIOBEN().set_bit().GPIOBEN().set_bit());

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

    let mut lcd_update = false;
    // Wake-up 2 / PC13 is the TMP117 alert, active low.  We test the GPIO state
    // rather than the wake-up flag, so that a lost edge on the alert pin
    // doesn't hang the temperature reading.
    if sr1.WUF2().bit() || !gpioc.IDR.read().ID13().bit() {
        pwr.SCR.write(|w| w.CWUF2().set_bit());
        sys.temp = tmp117::alert();
    }

    if sr1.WUFI().bit() {
        tick(sys);
        lcd_update = true;
    }

    rtc::standby(lcd_update);
}

#[test]
fn blink_mask_checks() {
    let checks = [
        (State::Cal , 0, 0x00ffffff, 0x00ffffffffffu64),
        (State::Time, 1, 0xfefe0000, 0xfefe00000000),
        (State::Time, 2, 0x0000fefe, 0x0000fefe0000),
        (State::Time, 3, 0x0000fefe, 0x00000000fefe),
        (State::Date, 1, 0x0000fefe, 0x00000000fefe),
        (State::Date, 2, 0x0000fefe, 0x0000fefe0000),
        (State::Date, 3, 0xfefe0000, 0xfefe00000000)
    ];

    let mut sys = System::default();
    for (s, ss, m4, m6) in checks {
        sys.state = s as u32;
        sys.sub_state = ss;
        assert_eq!(sys.blink_mask(), if LCD_WIDTH == 4 {m4} else {m6} as Segments);
    }
}
