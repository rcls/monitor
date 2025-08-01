
use crate::*;
use cpu::WFE;
use noise::Noise;
use static_assertions::const_assert;
use vcell::UCell;

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

struct Monitor {
    divide: u8,
    cycle_refresh: u8,
    cycle_arrow: u8,
    temp: i16,
    isense: i32,
    noise: [Noise; 4],
    frame: [oled::Line; 4],
}

static MONITOR: UCell<Monitor> = UCell::new(Monitor::new());

// Accelerometer.
//const FXLS8971: u8 = 0x30;

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
            noise: [Noise::default(); _],
            frame: [[0; _]; _]
        }
    }

    fn systick_handler(&mut self) {
        // Update arrows every tick.  Do this first to get smooth motion.
        let _ = self.update_arrows();
        let divide = if self.divide == 0 {4} else {self.divide - 1};
        self.divide = divide;
        if divide == 0 {
            adc::start();
        }
        if divide == 2 {
            adc::recalibrate_start();
        }
        if divide == 3 {
            accel_frig();
        }
        if divide == 4 {
            let _ = self.refresh();
        }
    }

    fn analog_update(&mut self) -> i2c::Result {
        i2c::read_reg(i2c::TMP117, 0, &mut self.temp).wait()?;

        // Log the ADC & TEMP values....
        let isense_counts = adc::DMA_BUF[ISENSE_INDEX].read() as i32;
        let vsense_counts = adc::DMA_BUF[VSENSE1_INDEX].read() as u32;
        let v3v3_counts   = adc::DMA_BUF[VREF_INDEX].read() as u32;
        let temp_counts   = i16::from_be(self.temp) as i32;

        let isense_raw = isense_counts - ISENSE_ZERO;
        self.isense = (isense_raw * ISENSE_SCALE + 32768) >> 16;

        let vsense;
        let power;
        if vsense_counts < 0xff00 || VSENSE1_INDEX == VSENSE2_INDEX {
            vsense = vconvert(vsense_counts);
            power = power_cW(isense_raw, vsense_counts);
        }
        else {
            let vsense2_counts = adc::DMA_BUF[VSENSE2_INDEX].read() as u32;
            vsense = vconvert2(vsense2_counts);
            power = power2_cW(isense_raw, vsense2_counts);
        }

        let v3v3 = v3v3convert(v3v3_counts);
        //let v3v3 = v3v3::v3v3_est(v3v3_counts);


        self.noise[0].update(vsense);
        self.noise[1].update(self.isense as u32);
        self.noise[2].update(v3v3);
        self.noise[3].update(temp_counts as u32);

        // sdbg!("?");
        //   VV.VVV V
        //  ±II.III A
        // n.nnn -nn.n
        if false {
            let isense =self.isense;
            dbgln!("{vsense:6}mV{isense:6}mA({isense_counts:5}){v3v3:6}mV{:7} m°C {} {} {} {}",
                temp_counts * 1000 >> 7,
                self.noise[0].decimal(), self.noise[1].decimal(),
                self.noise[2].decimal(), self.noise[3].decimal());
        }
        self.update_screen(power, vsense, v3v3, temp_counts)
    }

    fn update_screen(&mut self, power: u32, vsense: u32, v3v3: u32,
                     temp_counts: i32) -> i2c::Result {
        let mut line = [0; 10];
        decimal::format_u32(&mut line[..7], power, 2);
        line[7..].copy_from_slice(&CHARS_MAP!("  W"));
        oled::update_text(&mut self.frame[0], &line[..10], 0, 0)?;

        decimal::format_u32(&mut line[..6], vsense, 3);
        line[6..8].copy_from_slice(&CHARS_MAP!(" V"));
        oled::update_text(&mut self.frame[1], &line[..8], 2, 2)?;

        let usense = self.isense.unsigned_abs();
        decimal::format_u32(&mut line[..6], usense, 3);
        line[6..8].copy_from_slice(&CHARS_MAP!(" A"));
        oled::update_text(&mut self.frame[2], &line[..8], 2, 4)?;

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
        let mut inc  = 0;
        let mut dir  = 0;
        if self.isense > 0 {
            base = font::LEFT_TOP_0;
            inc = 1;
            dir = 2;
        }
        else if self.isense < 0 {
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
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};

    // Enable clocks.
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit());

    cpu::init1();

    gpioa.MODER.modify(|_, w| w.MODE11().B_0x1().MODE12().B_0x1()); // GPO LEDs.

    debug::init();

    dbgln!("Debug up");

    adc::init1();

    i2c::init();

    // Set the systick interrupt priority to a high value (other interrupts
    // pre-empt).
    // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
    // address.  We want SHPR3.
    assert_eq!(&scb.shpr[1] as *const _ as usize, 0xE000ED20);
    #[cfg(not(test))]
    unsafe {
        scb.shpr[1].write(0xc0 << 24);
    }

    cpu::init2();

    dbgln!("Going!");

    let _ = oled::init();

    dbgln!("Oled init!");

    adc::init2();

    if crate::HAVE_ACCEL {
        let ok = accel_init();
        dbgln!("Accel start {ok:?}");
    }

    // Turn on LSCO.
    //rcc.BDCR.modify(
    //    |_, w| w.LSCOSEL().set_bit().LSCOEN().set_bit().LSESYSEN().set_bit());

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

pub fn vconvert(counts: u32) -> u32 {
    const V_MULT: u32
        = (VSENSE1_FS * (1000 << VSENSE1_SHIFT) as f64 + 0.5) as u32;
    const_assert!(V_MULT >= 32768);
    const_assert!(V_MULT < 65536);
    (counts * V_MULT + (32768 << VSENSE1_SHIFT)) >> (16 + VSENSE1_SHIFT)
}

pub fn vconvert2(counts: u32) -> u32 {
    const V_MULT: u32
        = (VSENSE2_FS * (1000 << VSENSE2_SHIFT) as f64 + 0.5) as u32;
    const_assert!(V_MULT >= 32768);
    const_assert!(V_MULT < 65536);
    (counts * V_MULT + (32768 << VSENSE2_SHIFT)) >> (16 + VSENSE2_SHIFT)
}

fn v3v3convert(v3v3_counts: u32) -> u32 {
    const COUNTS_V3: u32 = (2.5 / 3.0 * 65536.0) as u32;
    const_assert!(65536 * 2500 / COUNTS_V3 >= 3000);
    const_assert!(65536 * 2500 / (COUNTS_V3 + 1) < 3000);
    if v3v3_counts > 45000 && v3v3_counts <= COUNTS_V3 {
        65536 * 2500 / v3v3_counts
    }
    else {
        3300
    }
}

#[allow(non_snake_case)]
// Note returns absolute value.
fn power_cW(isense_raw: i32, vsense_counts: u32) -> u32 {
    #[allow(non_upper_case_globals)]
    const cW_FS: f64 = VSENSE1_FS  // Voltage
        * ISENSE_FS                // Current
        * 100.0;                           // centi Watts
    const MULT: u32 = (cW_FS * (1 << POWER1_SHIFT) as f64) as u32;
    debug_assert!(MULT < 65536, "{cW_FS} {MULT} {VSENSE1_FS} {ISENSE_FS}");
    debug_assert!(MULT > 32768, "{cW_FS} {MULT} {VSENSE1_FS} {ISENSE_FS}");
    let power_counts = vsense_counts * isense_raw.unsigned_abs();
    // Power in centiwatts * 65536.
    let power = (power_counts >> 16) * MULT
        + ((power_counts & 0xffff) * MULT + 32768 >> 16);
    // Power in cW.
    power >> (16 + POWER1_SHIFT)
}

#[allow(non_snake_case)]
// Note returns absolute value.
fn power2_cW(isense_raw: i32, vsense_counts: u32) -> u32 {
    #[allow(non_upper_case_globals)]
    const cW_FS: f64 = VSENSE2_FS  // Voltage
        * ISENSE_FS                // Current
        * 100.0;                           // centi Watts
    const MULT: u32 = (cW_FS / (1 << POWER2_SHIFT) as f64 + 0.5) as u32;
    debug_assert!(MULT < 65536, "{cW_FS} {MULT} {VSENSE2_FS} {ISENSE_FS}");
    debug_assert!(MULT > 32768, "{cW_FS} {MULT} {VSENSE2_FS} {ISENSE_FS}");
    let power_counts = vsense_counts * isense_raw.unsigned_abs();
    // Power in centiwatts * (65536 >> POWER2_SHIFT).
    let power = (power_counts >> 16) * MULT
        + ((power_counts & 0xffff) * MULT + 32768 >> 16);
    // Power in cW.
    power >> (16 - POWER2_SHIFT)
}

#[test]
fn char_checks() {
    assert_eq!(font::SPACE, 0);
}

#[test]
fn power_check() {
    assert_eq!(power_cW(0, 0), 0);
    assert_eq!(power2_cW(0, 0), 0);
    // Check that full scale readings don't overflow and are in the right
    // ball-park.
    let p = power_cW(65535, 65535);
    assert!((p as f64 - 100. * VSENSE1_FS * ISENSE_FS).abs() < 10.);
    let p = power2_cW(65535, 65535);
    assert!((p as f64 - 100. * VSENSE2_FS * ISENSE_FS).abs() < 10.);
}
