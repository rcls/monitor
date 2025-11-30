use super::{LCD_WIDTH, State, System, conf};

use crate::lcd;
use lcd::{BITS, DIGITS, Segments, WIDTH, signed_to_segments, hex_to_segments};

macro_rules!dbgln {($($tt:tt)*) => {if false {crate::dbgln!($($tt)*)}};}

impl System {
    fn blink_mask(&self) -> Segments {
        let state = self.state();
        let sub_state = self.sub_state();

        if state == State::Conf {
            return match sub_state {
                0 | conf::CAL | conf::DRIVE => (1 << BITS - 8) - 1,
                conf::TOUCH => seg6(1, 1, 1, 1, 1, 0) * lcd::DOT as Segments
                    | lcd::COL1 | lcd::COL2,
                _ => (lcd::D8 as Segments) << BITS - 8
            }
        }

        if state == State::Pres && LCD_WIDTH < 6 {
            let dot = lcd::DOT as Segments;
            return if sub_state == 0 {0} else {dot * 0x01010100};
        }

        // The numbering is big endian, time display is big endian, date
        // display is little endian.  For a 4 digit display, the exception
        // is always on the right.
        let pos = if state == State::Date {4 - sub_state} else {sub_state};
        let dd = lcd::D8 as Segments * 0x101;
        match pos {        // Date or time.
            1 => dd << BITS - 16,
            2 => dd << BITS - 32,
            3 => dd,
            _ => 0,
        }
    }
}

pub fn get_segments(sys: &System) -> Segments {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};
    use State::*;
    let segments = match sys.state() {
        Time => time_to_segments(rtc.TR().read().bits(), sys.sub_state()),
        Date => date_to_segments(rtc.DR().read().bits(), sys.sub_state()),
        Temp => temp_to_segments(sys.temp),
        Pres => pres_to_segments(sys.pressure(), sys.pressure_point()),
        Humi => humi_to_segments(sys.humidity),
        Conf => conf_segments(sys),
    };
    if sys.sub_state() == 0 || rtc.SSR.read().bits() & 0x40 != 0 {
        segments
    }
    else {
        segments & !sys.blink_mask()
    }
}

fn conf_segments(sys: &System) -> Segments {
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    use crate::lcd::{DC, Dd, DF, Di, DG, Dn, Do, DP, DOT};
    match sys.sub_state() {
        conf::CAL => prefixed_to_segments(DC, crate::rtc::get_cal()),
        conf::DRIVE => prefixed_to_segments(
            Dd, rcc.BDCR.read().LSEDRV().bits().into()),
        conf::CRASH => {
            let mut segs = [0; _];
            hex_to_segments(&mut segs, sys.crash, 0, false);
            segs[WIDTH - 1] = DP | DOT;
            Segments::from_le_bytes(segs)
        },
        conf::TOUCH => touch_debug_segments(sys),
        _ => {
            if WIDTH < 6 {seg4(DC, Do, Dn, DF)} else {seg6(DC, Do, Dn, DF, Di, DG)}
        }
    }
}

fn touch_debug_segments(sys: &System) -> Segments {
    use lcd::{Dt, Do, Du, Dc, Dh, DOT};
    if sys.scratch >= 0x80000000 {
        let mut segs = [0; _];
        hex_to_segments(&mut segs, sys.scratch, 0, false);
        return Segments::from_le_bytes(segs) | lcd::COL1 | lcd::COL2;
    }

    seg6(0, DOT, DOT, DOT, DOT, 0)
        | if WIDTH < 6 {seg4(Dt, Du, Dc, Dh)} else {seg6(0, Dt, Do, Du, Dc, Dh)}
}

fn date_time_to_segments(t: u32, dots: Segments, kz: bool) -> Segments {
    let mut result = [0; _];
    hex_to_segments(&mut result, t, WIDTH, false);
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

fn time_to_segments(t: u32, sub_state: u32) -> Segments {
    let t = if WIDTH == 6 || sub_state == 3 {t} else {t >> 8};
    let segs = date_time_to_segments(t, lcd::COL1 | lcd::COL2, false);
    if WIDTH == 6 || sub_state != 3 {
        segs
    }
    else {
        segs & 0xffff
    }
}

fn date_to_segments(d: u32, sub_state: u32) -> Segments {
    let mult = if WIDTH == 4 {1} else {0x10001};
    let dots = mult * 0x10000 + lcd::COL1 + lcd::COL2;
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

fn temp_to_segments(temp: i32) -> Segments {
    let mut segs = [0; _];
    let p = signed_to_segments(&mut segs, temp, 2);
    let mut segs = Segments::from_le_bytes(segs) | lcd::DOT as Segments * 256;
    if p < WIDTH {
        segs = segs * 256 + lcd::DEG as Segments;
    }
    if WIDTH == 6 && p <= 3 {
        segs *= 256;
    }
    segs
}

fn humi_to_segments(humidity: u32) -> Segments {
    // 99.9, on a 6 digit display add %(°o).
    let mut segs = [0; _];
    let d = signed_to_segments(&mut segs, humidity as i32, 3);
    let segs = Segments::from_le_bytes(segs) | lcd::DOT as Segments * 256;
    if d > 4 {
        segs
    }
    else if WIDTH == 6 {
        segs * 65536 + seg4(0, 0, lcd::DEG, lcd::Do)
    }
    else {
        segs * 256
    }
}

fn pres_to_segments(pressure: u32, point: u32) -> Segments {
    let mut segs = [0; _];
    // The pressure value is 64 counts per Pa.
    let round = match point {0 => 3200, 1 => 320, _ => 32};
    let pres = (pressure + round) / 64;
    let bcd = crate::utils::to_bcd(pres) >> 8 - 4 * point;
    dbgln!("{pres} {:#x} -> {bcd:#x}", crate::utils::to_bcd(pres));
    hex_to_segments(&mut segs, bcd, WIDTH, false);
    Segments::from_le_bytes(segs) | (lcd::DOT as Segments) << (8 * point + 8)
}

fn prefixed_to_segments(prefix: u8, value: i32) -> Segments {
    let mut segs = [0; _];
    signed_to_segments(&mut segs, value, 0);
    segs[WIDTH - 1] = prefix;
    Segments::from_le_bytes(segs)
}

const fn seg4(a: u8, b: u8, c: u8, d: u8) -> Segments {
    (a as Segments * 65536 + b as Segments * 256 + c as Segments) * 256
        + d as Segments
}

const fn seg6(a: u8, b: u8, c: u8, d: u8, e: u8, f: u8) -> Segments {
    seg4(c, d, e, f) | if WIDTH == 6 {seg4(0, 0, a, b) << 32} else {0}
}

#[test]
fn blink_mask_checks() {
    let checks = [
        (State::Conf, 0, 0x00ffffff, 0x00ffffffffffu64),
        (State::Time, 1, 0xfefe0000, 0xfefe00000000),
        (State::Time, 2, 0x0000fefe, 0x0000fefe0000),
        (State::Time, 3, 0x0000fefe, 0x00000000fefe),
        (State::Date, 1, 0x0000fefe, 0x00000000fefe),
        (State::Date, 2, 0x0000fefe, 0x0000fefe0000),
        (State::Date, 3, 0xfefe0000, 0xfefe00000000)
    ];

    let mut sys = System::default();
    for (s, ss, m4, m6) in checks {
        sys.states = s as u32 | ss << 16;
        assert_eq!(sys.blink_mask(),
                   if LCD_WIDTH == 4 {m4} else {m6} as lcd::Segments);
    }
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
        assert_eq!(lcd::segments_to_str(temp_to_segments(t)),
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
        assert_eq!(lcd::segments_to_str(date_to_segments(d, 0)),
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
                assert_eq!(lcd::segments_to_str(time_to_segments(t, 0)), s);
            }
        }
    }
}

#[test]
fn date_adjust_date_checks() {
    if WIDTH == 6 {
        for i in 0 ..= 3 {
            assert_eq!(lcd::segments_to_str(date_to_segments(0x691204, i)),
                       " 4⋮12⋮69");
        }
        return;
    }
    assert_eq!(lcd::segments_to_str(date_to_segments(0x691204, 2)),
               " 4⋮12");
    assert_eq!(lcd::segments_to_str(date_to_segments(0x691204, 3)),
               " 4⋮12");
    assert_eq!(lcd::segments_to_str(date_to_segments(0x691204, 1)),
               "  ⋮69");
    assert_eq!(lcd::segments_to_str(date_to_segments(0x691204, 1)),
               "  ⋮69");
    assert_eq!(lcd::segments_to_str(date_to_segments(0x071204, 1)),
               "  ⋮07");
}

#[test]
fn check_cal() {
    for i in -99 ..= 999 {
        assert_eq!(lcd::segments_to_str(prefixed_to_segments(lcd::DC, i)),
                   format!("C{i:w$}", w = WIDTH - 1));
    }
}
