//! Apply adjustments to date/time values.

pub fn adjust_time(item: u32, t: u32, forwards: bool) -> u32 {
    if forwards {
        forwards_time(item, t)
    }
    else {
        backwards_time(item, t)
    }
}

pub fn adjust_date(item: u32, d: u32, forwards: bool) -> u32 {
    if forwards {
        forwards_date(item, d)
    }
    else {
        backwards_date(item, d)
    }
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
fn test_all_times() {
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
    for s in 0 .. 86400 {
        check_pair(0, s, s + 1);
        check_pair(1, s, s + 60);
        check_pair(2, s, s + 3600);
    }
}
