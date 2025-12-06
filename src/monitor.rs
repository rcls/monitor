
use crate::*;
use cpu::WFE;
use stm_common::vcell::UCell;

/// TMP117 I2C address.
pub const TMP117: u8 = 0x92;

pub const I2C_LINES: i2c::I2CLines = i2c::I2CLines::A9_A10;

pub const CONFIG: cpu::Config = {
    let mut c = cpu::Config::new(16000000);
    c.fll = false;
    if HAVE_ACCEL {
        c.fxls8971();
    }
    *c.systick(monitor::systick_handler).adc().debug().i2c()};

struct Monitor {
    divide: u8,
    cycle_refresh: u8,
    cycle_arrow: u8,
    temp: i16,
    isense: i32,
    isense_accum: i32,
    isense_count: u32,
    frame: [oled::Line; 4],
}

static MONITOR: UCell<Monitor> = UCell::new(Monitor::new());

/// Keep stats on current measurement instead of misc info.
const ISTAT: bool = false;

pub fn systick_handler() {
    unsafe{MONITOR.as_mut().systick_handler()};
}

impl Monitor {
    const fn new() -> Monitor {
        Monitor{
            divide: 0,
            cycle_refresh: 0,
            cycle_arrow: 0,
            temp: 0,
            isense: 0,
            isense_accum: 0,
            isense_count: 0,
            frame: [[0; _]; _]
        }
    }

    fn systick_handler(&mut self) {
        // Update arrows every tick.  Do this first to get smooth motion.
        let _ = self.update_arrows();
        let divide = if self.divide == 0 {4} else {self.divide - 1};
        self.divide = divide;
        // This should happen on divide = 1.
        if adc::OUTSTANDING.read() == 0 {
            adc::OUTSTANDING.write(1);
            cpu::barrier();
            let _ = self.analog_update();
        }

        if divide == 0 {
            adc::start();
        }
        if divide == 2 {
            adc::recalibrate_start();
        }
        if divide == 4 {
            let _ = self.refresh();
        }
    }

    fn analog_update(&mut self) -> i2c::Result {
        let wait = i2c::read_reg(TMP117, 0, &mut self.temp);

        let isense_counts = adc::DMA_BUF[ISENSE_INDEX].read() as i32;
        let isense_raw = isense_counts - ISENSE_ZERO;
        self.isense = (isense_raw * ISENSE_SCALE + 32768) >> 16;

        // isense accumulation...
        self.isense_accum += isense_raw;
        self.isense_count += 1;
        if ISTAT && self.isense_count.is_power_of_two() {
            dbgln!("isense accum {} over {}", self.isense_accum,
                   self.isense_count / 2);
            let mut line = [0; 9];
            decimal::format_i32(&mut line[..9], self.isense_accum, 0);
            oled::update_text(&mut self.frame[3], &line[..9], 0, 6)?;
            decimal::format_u32(&mut line[..2],
                                (self.isense_count / 2).trailing_zeros(), 0);
            oled::update_text(&mut self.frame[3], &line[..2], 10, 6)?;
            self.isense_accum = 0;
        }

        let vsense_counts = adc::DMA_BUF[VSENSE1_INDEX].read() as u32;
        let vsense;
        let power;
        if vsense_counts < 0xff00 || VSENSE1_INDEX == VSENSE2_INDEX {
            vsense = VCONVERT1.convert(vsense_counts);
            power = PCONVERT1.convert(isense_raw, vsense_counts);
        }
        else {
            let vsense2_counts = adc::DMA_BUF[VSENSE2_INDEX].read() as u32;
            vsense = VCONVERT2.convert(vsense2_counts);
            power = PCONVERT2.convert(isense_raw, vsense2_counts);
        }

        let v3v3_counts = adc::DMA_BUF[VREF_INDEX].read() as u32;
        let v3v3 = v3v3convert(v3v3_counts);

        wait.wait()?;
        let temp_counts = i16::from_be(self.temp) as i32;
        self.update_screen(power, vsense, v3v3, temp_counts)
    }

    fn update_screen(&mut self, power: u32, vsense: u32, v3v3: u32,
                     temp_counts: i32) -> i2c::Result {
        let mut line = [0; 10];
        decimal::format_u32(&mut line[..7], power, 2);
        line[7] = oled::char_map(' ');
        line[8] = oled::char_map(' ');
        line[9] = oled::char_map('W');
        oled::update_text(&mut self.frame[0], &line[..10], 0, 0)?;

        decimal::format_u32(&mut line[..6], vsense, 3);
        line[6..8].copy_from_slice(&CHARS_MAP!(" V"));
        oled::update_text(&mut self.frame[1], &line[..8], 2, 2)?;

        let usense = self.isense.unsigned_abs();
        decimal::format_u32(&mut line[..6], usense, 3);
        line[6..8].copy_from_slice(&CHARS_MAP!(" A"));
        oled::update_text(&mut self.frame[2], &line[..8], 2, 4)?;

        if ISTAT {
            return Ok(());
        }

        let centic = (temp_counts * 100 + 50) >> 7;
        decimal::format_u32(&mut line[..4], v3v3, 0);
        line[4] = font::LETTER_m;
        oled::update_text(&mut self.frame[3], &line[..5], 0, 6)?;

        decimal::format_i32(&mut line[..6], centic, 2);
        line[6] = font::DEGREE; // °
        oled::update_text(&mut self.frame[3], &line[..7], 5, 6)
    }

    fn refresh(&mut self) -> i2c::Result {
        if self.cycle_refresh < 4 {
            oled::draw_chars(&self.frame[self.cycle_refresh as usize],
                             0, self.cycle_refresh * 2)?;
            self.cycle_refresh += 1;
        }
        else {
            oled::reset()?;
            self.cycle_refresh = 0;
        }
        Ok(())
    }

    fn update_arrows(&mut self) -> i2c::Result {
        let cycle = self.cycle_arrow + 1 & 7;
        self.cycle_arrow = cycle;

        let mut base = font::SPACE;
        let mut inc = 0;
        let mut dir = 0;
        let isense = if oled::is_flipped() {-self.isense} else {self.isense};
        if isense < 0 {
            base = font::LEFT_TOP_0;
            inc = 1;
            dir = 2;
        }
        else if isense > 0 {
            base = font::RIGHT_TOP_0;
            inc = 1;
            dir = 6;
        }
        let i0 = base + inc * cycle;
        let i1 = base + inc * ((cycle + dir) & 7);
        let d = 8 * inc;
        let t = [i0, i1];
        let b = [i0 + d, i1 + d];

        oled::update_text(&mut self.frame[1], &t, 0, 2)?;
        oled::update_text(&mut self.frame[1], &t, 10, 2)?;
        oled::update_text(&mut self.frame[2], &b, 0, 4)?;
        oled::update_text(&mut self.frame[2], &b, 10, 4)
    }
}

pub fn main() -> ! {
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    let scb = unsafe {&*cortex_m::peripheral::SCB::PTR};

    // Enable clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit());

    cpu::init1();
    debug::init();
    adc::init1();
    i2c::init();

    // Set the systick interrupt priority to a high value (other interrupts
    // pre-empt).
    // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
    // address.  We want SHPR3.
    link_assert!(&scb.shpr[1] as *const _ as usize == 0xE000ED20);
    #[cfg(not(test))]
    unsafe {
        scb.shpr[1].write(0xc0 << 24);
    }

    cpu::init2();
    if let Err(()) = oled::init() {
        debug::write_str("OLED failed\n");
    }

    if crate::HAVE_ACCEL && let Err(()) = fxls8971::accel_init() {
        debug::write_str("Accel failed\n");
    }
    adc::init2();

    // systick counts at 16MHz / 8 = 2MHz.
    unsafe {
        let syst = &*cortex_m::peripheral::SYST::PTR;
        syst.rvr.write(CONFIG.clk / 8 / 25 - 1); // 2MHz / 80000 = 25Hz
        syst.cvr.write(CONFIG.clk / 8 / 25 - 1);
        syst.csr.write(3);
    }

    loop {
        WFE();
    }
}

pub struct VConvert {
    mult: u32,
    shift: u32,
}

impl VConvert {
    // Full-scale is volts for 65536 counts.
    pub const fn new(full_scale: f64, shift: u32) -> VConvert {
        let mult: u32 = (full_scale * (1000 << shift) as f64 + 0.5) as u32;
        assert!(mult >= 32768);
        assert!(mult < 65536);
        let me = VConvert{mult, shift: shift + 16};
        // Self-check conversion of full-scale.
        let full = me.convert(65536);
        assert!((full as f64 - full_scale * 1000.).abs() <= 2.);
        me
    }
    pub const fn convert(&self, counts: u32) -> u32 {
        utils::round_shr(counts * self.mult, self.shift)
    }
}

pub struct PConvert {
    mult: u32,
    shift: u32,
}

impl PConvert {
    pub const fn new(i_full_scale: f64, v_full_scale: f64,
                     up: u32, down: u32) -> PConvert {
        #[allow(non_snake_case)]
        let cW_FS = v_full_scale * i_full_scale * 100.0;
        let mult = (cW_FS * (1 << up) as f64 / (1 << down) as f64 + 0.5) as u32;
        assert!(mult < 65536);
        assert!(mult >= 32768);
        let me = PConvert{mult, shift: 16 + up - down};
        // Self check - full scale conversion should be in the right ball-park,
        // and not trigger any overflows.
        let full = me.convert(65535, 65535) as f64;
        assert!((full - 100. * i_full_scale * v_full_scale).abs() < 10.);

        me
    }

    /// Convert current and voltage counts to power in centi Watts.  Note that
    /// this returns the absolute value.
    pub const fn convert(&self, isense_raw: i32, vsense_counts: u32) -> u32 {
        let power_counts = vsense_counts * isense_raw.unsigned_abs();
        // Power in centiWatts multiplied by 1<<shift.
        let power = (power_counts >> 16) * self.mult
            + ((power_counts & 0xffff) * self.mult >> 16);
        utils::round_shr(power, self.shift)
    }
}

fn v3v3convert(v3v3_counts: u32) -> u32 {
    const COUNTS_V3: u32 = (2.5 / 3.0 * 65536.0) as u32;
    const {assert!(65536 * 2500 / COUNTS_V3 >= 3000)};
    const {assert!(65536 * 2500 / (COUNTS_V3 + 1) < 3000)};
    if v3v3_counts > 45000 && v3v3_counts <= COUNTS_V3 {
        65536 * 2500 / v3v3_counts
    }
    else {
        3300
    }
}

#[test]
fn char_checks() {
    assert_eq!(font::SPACE, 0);
}
