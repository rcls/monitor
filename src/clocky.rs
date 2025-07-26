use crate::*;

use lcd::{Segments, WIDTH};
use tsc::pad;

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

static_assertions::const_assert_eq!(TICKS_PER_SEC, 1 << 8 - ALARM_BITS);

const TOUCH_TIME_OUT: u32 = 10 * TICKS_PER_SEC;
const REPEAT_FIRST: u32 = TICKS_PER_SEC;
const REPEAT_AGAIN: u32 = TICKS_PER_SEC / 2;

/// The overall system state, taking just 9 32-bit words.
#[repr(C)]
struct System {
    /// Magic number.
    magic: u32,
    /// State enumeration.
    state: u32,
    /// Temperature in tenths of °C in low 16 bits.  High bit indicates
    /// measurement outstanding.
    temp: i32,
    /// Sub-state - which field is active when updating date or time.
    sub_state: u32,
    /// Current touch state. 0:None, 1:+, 2:-, 3:≡
    touch: u32,
    /// Timer, counting ticks.  Down counter running while touch system
    /// activated.
    timer: u32,
    _unused: [u32; 2],
    /// Stored address before crash & reboot.
    crash: u32,
}
static_assertions::const_assert!(size_of::<System>() == 9 * 4);

#[allow(dead_code)]
#[repr(u32)]
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd)]
enum State {
    Time = 0,
    Date = 1,
    Temp = 2,
}

impl State {
    const MIN: State = State::Time;
    const MAX: State = State::Temp;
}

impl System {
    unsafe fn get() -> &'static mut Self {
        let tamp = unsafe {&*stm32u031::TAMP::ptr()};
        let p = tamp.BKPR[0].as_ptr() as *const crate::vcell::UCell<System>;
        let sys = unsafe {(&*p).as_mut()};
        // Validate fields.
        if sys.state > State::MAX as u32 {
            sys.state = State::Time as u32;
        }
        sys
    }
    fn reset(&mut self) {
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
    fn sub_state(&self) -> u32 {
        self.sub_state
    }
    fn set_sub_state(&mut self, v: u32) {
        self.sub_state = v;
    }
    fn blink_mask(&self) -> Segments {
        let dd = lcd::D8 as Segments * 0x101;
        match self.sub_state() {
            1 => dd << WIDTH * 8 - 16,
            2 => dd << WIDTH * 8 - 32,
            3 if WIDTH == 6 => dd,
            _ => 0,
        }
    }
    fn temp(&self) -> i32 {self.temp as i16 as i32}
    /// Also clears pending.
    fn set_temp(&mut self, temp: i16) {self.temp = temp as u16 as i32}
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
    debug::banner("**** CLOCKY 0x", sys.crash, " ****\n");
    sys.reset();

    rtc::ensure_options();
    rtc_setup_tick();

    tmp117::init();
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
    rtc::relock();
}

fn get_segments(sys: &System) -> Segments {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    use State::*;
    let segments = match sys.state() {
        Time => lcd::time_to_segments(rtc.TR().read().bits()),
        Date => lcd::date_to_segments(rtc.DR().read().bits()),
        Temp => lcd::temp_to_segments(sys.temp())
    };
    if rtc.SSR.read().bits() & 0x40 == 0 {
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
    let pad = tsc::retrieve();
    if !touch_state(sys, pad) {
        return;
    }
    if pad == pad::NONE {
        // Cancel any blinky stuff.
        sys.set_sub_state(0);
        return;
    }
    if pad == pad::MENU {
        // If sub-state is 0, then rotate state.  Else rotate sub-state.
        let ss = sys.sub_state();
        if ss != 0 {
            sys.set_sub_state(if ss < WIDTH as u32 / 2 {ss + 1} else {0});
        }
        else if sys.state < State::MAX as u32 {
            sys.state += 1;
        }
        else {
            sys.state = State::MIN as u32
        }
        return;
    }
    // + or -.
    if sys.state() != State::Time && sys.state() != State::Date {
        return;
    }
    if sys.sub_state() == 0 {
        sys.set_sub_state(1);
        return;
    }

    let item;
    if sys.state() == State::Time {
        item = 3 - sys.sub_state();
    }
    else {
        item = 5 - WIDTH as u32 / 2 + sys.sub_state();
    }

    if pad == pad::PLUS {
        rtc::forwards(item);
    }
    if pad == pad::MINUS {
        rtc::backwards(item);
    }
}

fn tick(sys: &mut System) {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};

    let sr = rtc.MISR.read();
    rtc.SCR.write(|w| w.bits(sr.bits()));

    let ssr = rtc.SSR.read().bits();

    let tacquire;
    if sys.is_temp_pending() {
        // Override tacquire if we already have one pending, don't request
        // another.
        sys.set_temp_pending(false);
        tacquire = false;
    }
    else if ssr >= 0xfc {
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

    let mut lcd_update = false;
    // Wake-up 2 is the TMP117 alert.
    if sr1.WUF2().bit() {
        pwr.SCR.write(|w| w.CWUF2().set_bit());
        temp_alert(sys);
    }

    if sr1.WUFI().bit() {
        tick(sys);
        lcd_update = true;
    }

    rtc::standby(lcd_update);
}
