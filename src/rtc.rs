use crate::CONFIG;

/// Make sure RTC is on, and disable RTC write protect.
pub fn setup_start() {
    let rcc = unsafe {&*stm32u031::RCC::ptr()};

    // RTC enabled and clocked with LSE.  This is only cleared by power on
    // or back-up domain reset.
    rcc.BDCR.modify(|_,w| w.RTCEN().set_bit().RTCSEL().B_0x1());

    unlock();
}

/// Disable write protect.  Note that DPB bit is set in [cpu::init1()].
pub fn unlock() {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    rtc.WPR.write(|w| w.bits(0xca));
    rtc.WPR.write(|w| w.bits(0x53));
}

/// Re-enable RTC write protect.
pub fn relock() {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    rtc.WPR.write(|w| w.bits(0));
    rtc.WPR.write(|w| w.bits(0));
}

/// Start initialization.
pub fn init_start() {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};

    const {assert!(crate::CONFIG.apb1_clocks & 1 << 10 != 0)}
    const {assert!(crate::CONFIG.low_power)}

    unlock();
    rtc.ICSR.write(|w| w.INIT().set_bit());
    // Polling...
    while !rtc.ICSR.read().INITF().bit() {}
}

pub fn init_end() {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    rtc.ICSR.write(|w| w.INIT().clear_bit());
    relock();
}

#[allow(dead_code)]
pub fn set_wakeup(div: u16) {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};

    rtc.CR.write(|w| w.WUTE().clear_bit());
    // This shouldn't take long and there appears to be nothing other than
    // spin that we can do.
    while !rtc.ICSR.read().WUTWF().bit() {
    }

    // Clock input to WUT is LSE / {2,4,8,16}.
    // 32768 / 20 = 1638.4 ≈ 16 * 102.
    rtc.WUTR.write(|w| w.WUT().bits(div));
}

/// Make sure reset is output only.
pub fn ensure_options() {
    let flash = unsafe {&*stm32u031::FLASH::ptr()};

    let options = flash.OPTR.read();
    if options.NRST_MODE().bits() == 1 {
        return;
    }
    crate::debug::banner(
        "Options = 0x", options.bits(), " ... needs to be rewritten");

    // 1.Write KEY1 = 0x4567 0123 in the FLASH key register (FLASH_KEYR)
    // 2.Write KEY2 = 0xCDEF 89AB in the FLASH key register (FLASH_KEYR).
    flash.KEYR.write(|w| w.bits(0x45670123));
    flash.KEYR.write(|w| w.bits(0xcdef89ab));
    // 1.Unlock the FLASH_CR with the LOCK clearing sequence (refer to
    // Unlocking the flash memory ).
    // 2.Write OPTKEY1 = 0x0819 2A3B of the FLASH option key register
    //  (FLASH_OPTKEYR).
    // 3.Write OPTKEY2 = 0x4C5D 6E7F of the FLASH option key register
    // (FLASH_OPTKEYR)
    flash.OPTKEYR.write(|w| w.bits(0x08192a3b));
    flash.OPTKEYR.write(|w| w.bits(0x4c5d6e7f));

    // Write the options...
    flash.OPTR.modify(|_,w| w.NRST_MODE().B_0x1());

    while flash.SR.read().BSY1().bit() {}
    flash.CR.write(|w| w.OPTSTRT().set_bit());
    while flash.SR.read().BSY1().bit() {}

    crate::debug::write_str("Options written!\n");
}

pub fn get_cal() -> i32 {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    let calr = rtc.CALR.read();
    calr.CALM().bits() as i32 - calr.CALP().bit() as i32 * 512
}

pub fn set_cal(cal: i32) {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    unlock();
    rtc.CALR.write(
        |w| w.CALP().bit(cal & 512 != 0).CALM().bits(cal as u16 & 511));
    relock();
}

fn forwards_time(item: u32, mut t: u32) -> u32 {
    if item == 0 { // Seconds.
        if t & 0xff < 0x59 {
            return t + if t & 0xf < 9 {1} else {7};
        }
        t -= 0x59;
    }
    if item <= 1 { // Minutes
        if t & 0xffff < 0x5900 {
            return t + if t & 0xfff < 0x900 {0x100} else {0x700};
        }
        t -= 0x5900;
    }
    // Hours.
    if t < 0x230000 {
        return t + if t & 0xfffff < 0x90000 {0x10000} else {0x70000};
    }
    t - 0x230000 // Day roll over!.
}

fn backwards_time(item: u32, mut t: u32) -> u32 {
    if item == 0 { // Seconds.
        if t & 0xff != 0x00 {
            return t - if t & 15 != 0 {1} else {7};
        }
        t += 0x59;
    }
    if item <= 1 { // Minutes
        if t & 0xffff >= 0x100 {
            return t - if t & 0xfff >= 0x100 {0x100} else {0x700};
        }
        t += 0x5900;
    }
    // Hours.
    if t >= 0x010000 {
        return t - if t & 0xfffff >= 0x10000 {0x10000} else {0x70000};
    }
    t + 0x230000 // Day roll over!.
}

// d should be in the RTC BCD date format, and returns a BCD byte.
#[inline(never)]
fn days_in_month(d: u32) -> u32 {
    static DAYS_PER_MONTH: [u8; 32] = [ // BCD!
        0, 0x31, 0x28, 0x31,  0x30, 0x31, 0x30, 0x31, // Jan ..= Jul
        0x31,  0x30, 0, 0, 0, 0, 0, 0,                // Aug, Sep.
        0x31, 0x30, 0x31, 0,  0, 0, 0, 0,   0, 0, 0, 0,   0, 0, 0, 0];
    let month = (d >> 8) & 31;
    let mut dim = DAYS_PER_MONTH[month as usize] as u32;
    if month == 2 {
        let year4 = ((d >> 16) & 15) + 2 * ((d >> 20) & 15);
        let leap_year = year4 & 3 == 0 && year4 != 0;
        if leap_year {
            dim += 1;
        }
    }
    dim
}

fn forwards_date(item: u32, mut d: u32) -> u32 {
    if item <= 3 { // Days.
        if d & 0xff < days_in_month(d) {
            return d + if d & 15 < 9 {1} else {7};
        }
        d = d & !0xff | 1;
    }
    if item <= 4 { // Month.
        if d & 0x1f00 < 0x1200 {
            return d + if d & 0xf00 < 0x900 {0x100} else {0x700};
        }
        d -= 0x1100;
    }
    // Year.
    if d & 0xf0000 < 0x90000 {
        d + 0x10000
    }
    else if d < 0x900000 {
        d + 0x70000
    }
    else {
        d - 0x990000 // Full wrap!
    }
}

fn back_month_or_year(item: u32, mut d: u32) -> u32 {
    if item <= 4 { // Month
        if d & 0x1fff >= 0x0200 {
            return d - if d & 0xfff >= 0x100 {0x100} else {0x700};
        }
        d += 0x1100;
    }
    // Year.
    if d >= 0x10000 {
        d - if d & 0xfffff > 0x10000 {0x10000} else {0x70000}
    }
    else {
        d + 0x990000
    }
}

fn backwards_date(item: u32, mut d: u32) -> u32 {
    if item <= 3 && d & 0xff > 1 {
        return d - if d & 15 > 0 {1} else {7};
    }

    d = back_month_or_year(item, d);

    if item <= 3 { // Redo the days.
        (d & !0xff) + days_in_month(d)
    }
    else {
        d
    }
}

pub fn forwards(item: u32) {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    init_start();
    if item < 3 {
        let t = forwards_time(item, rtc.TR.read().bits());
        rtc.TR.write(|w| w.bits(t));
    }
    if item >= 3 {
        let d = forwards_date(item, rtc.DR.read().bits());
        rtc.DR.write(|w| w.bits(d));
    }
    init_end();
}

pub fn backwards(item: u32) {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    init_start();
    if item < 3 {
        let t = backwards_time(item, rtc.TR.read().bits());
        rtc.TR.write(|w| w.bits(t));
    }
    if item >= 3 {
        let d = backwards_date(item, rtc.DR.read().bits());
        rtc.DR.write(|w| w.bits(d));
    }
    init_end();
}

pub fn standby(update_pupd: bool) -> ! {
    let pwr   = unsafe {&*stm32u031::PWR  ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let gpioc = unsafe {&*stm32u031::GPIOC::ptr()};
    let scb   = unsafe {&*cortex_m::peripheral::SCB::PTR};

    // Low power mode to use is standby...
    pwr.CR1.modify(|_,w| w.LPMS().bits(3)); // 3=Standby, 4=Shutdown.

    // Clear the reset causes & set restart MSI.
    rcc.CSR.write(
        |w| w.RMVF().set_bit().MSISRANGE().bits(crate::cpu::MSIRANGE));

    fn pupd(shift: u32, gpio: &stm32u031::gpioa::RegisterBlock,
            pucr: &stm32u031::pwr::PUCRA, pdcr: &stm32u031::pwr::PDCRA) {
        let standby_pu = (CONFIG.standby_pu >> shift) as u32 & 0xffff;
        let standby_pd = (CONFIG.standby_pd >> shift) as u32 & 0xffff;
        let keep_up    = (CONFIG.keep_pu    >> shift) as u32 & 0xffff;
        let keep_down  = (CONFIG.keep_pd    >> shift) as u32 & 0xffff;

        let keep = keep_up | keep_down != 0;
        // Not that if we haven't bought the pin out of default mode then IDR
        // doesn't reflect the pin.
        let bits = if keep {gpio.IDR().read().bits()} else {0};
        if standby_pu | keep_up != 0 {
            pucr.write(|w| w.bits(bits & keep_up | standby_pu));
        }
        if standby_pd | keep_down != 0 {
            pdcr.write(|w| w.bits(!bits & keep_down | standby_pd));
        }
    }

    if update_pupd {
        pupd( 0, gpioa, &pwr.PUCRA, &pwr.PDCRA);
        pupd(16, gpiob, &pwr.PUCRB, &pwr.PDCRB);
        pupd(32, gpioc, &pwr.PUCRC, &pwr.PDCRC);
    }

    crate::debug::flush();

    // Deep sleep.  Note that if an interrupt does WFE after this we are liable
    // to go into standby from the ISR!
    unsafe {scb.scr.write(4)};

    // It looks like, if we do have wake-up bits set, then we reset immediately,
    // instead of ignoring the WFE completely.  The event flag is probably set
    // when we get here.
    loop {
        #[cfg(target_os = "none")]
        cortex_m::asm::wfi();
        unsafe {(*cortex_m::peripheral::SCB::PTR).aircr.write(0x05fa0004)};
    }
}

impl crate::cpu::Config {
    pub const fn rtc(&mut self) -> &mut Self {
        self.low_power = true;
        self.apb1_clocks |= 1 << 10;
        self
    }
}

#[test]
fn test_days_in_month() {
    assert_eq!(days_in_month(0x120201), 0x29);
    assert_eq!(days_in_month(0x110201), 0x28);
    // I wonder which the RTC implements?
    assert_eq!(days_in_month(0x000214), 0x28);
    assert_eq!(days_in_month(0x00f130), 0x30);
}

#[cfg(test)]
fn check_date_pair(item: u32, a: u32, b: u32) {
    // Cover all values of the day-of-week field.
    for d in 0 ..= 7 {
        let a = a | d << 13;
        let b = b | d << 13;
        let aa = backwards_date(item, b);
        let bb = forwards_date(item, a);
        assert_eq!(b, bb, "{item} {a:06x} {bb:06x}/{b:06x}");
        assert_eq!(aa, a, "{item} {aa:06x}/{a:06x} {b:06x}");
    }
}

#[test]
fn test_advance_date() {
    check_date_pair(3, 0x010101, 0x010102);
    check_date_pair(3, 0x690630, 0x690701);
    check_date_pair(3, 0x690228, 0x690301);
    check_date_pair(3, 0x760228, 0x760229);
    check_date_pair(3, 0x760229, 0x760301);
    check_date_pair(3, 0x791231, 0x800101);
    check_date_pair(3, 0x991231, 0x000101);

    // Hmmmmm not sure what to do about this.
    check_date_pair(4, 0x250331, 0x250431);
    check_date_pair(4, 0x251231, 0x260131);

    check_date_pair(5, 0x680616, 0x690616);
    check_date_pair(5, 0x690616, 0x700616);

    check_date_pair(3, 0x020529, 0x020530);
    check_date_pair(3, 0x020530, 0x020531);
    check_date_pair(3, 0x020531, 0x020601);

    check_date_pair(3, 0x250930, 0x251001);
    check_date_pair(4, 0x250912, 0x251012);
}

#[test]
fn test_all_dates() {
    let mut all = Vec::new();
    fn to_bcd(x: u32) -> u32 {x / 10 * 16 + x % 10}
    for y in 0 ..= 99 {
        for m in 1 ..= 12 {
            static DIM: [u32; 13] = [
                0, 31, 28, 31, 30, 31, 30,  31, 31, 30, 31, 30, 31];
            let mut days_in_month = DIM[m as usize];
            if m == 2 && y != 0 && y % 4 == 0 {
                days_in_month = 29;
            }
            for d in 1 ..= days_in_month {
                all.push(to_bcd(y) * 65536 + to_bcd(m) * 256 + to_bcd(d));
            }
        }
    }
    for i in 1 .. all.len() {
        check_date_pair(3, all[i-1], all[i]);
    }
}

#[test]
fn test_advance_time() {
    fn seconds_to_bcd(s: u32) -> u32 {
        let su = s % 10;
        let st = s / 10 % 6;
        let mu = s / 60 % 10;
        let mt = s / 600 % 6;
        let hu = s / 3600 % 10;
        let ht = s / 36000;
        [ht, hu, mt, mu, st, su].iter().fold(0, |x, y| x * 16 + y)
    }
    fn check_pair(item: u32, a: u32, b: u32) {
        let bcda = seconds_to_bcd(a % 86400);
        let bcdb = seconds_to_bcd(b % 86400);
        assert_eq!(forwards_time(item, bcda), bcdb,
                   "{item} {bcda:06x} {bcdb:06x}");
        assert_eq!(bcda, backwards_time(item, bcdb),
                   "{item} {bcda:06x} {bcdb:06x}");
    }
    for s in 35998..86400 {
        check_pair(0, s, s+1);
        check_pair(1, s, s+60);
        check_pair(2, s, s+3600);
    }
}
