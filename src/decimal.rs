
fn to_bcd(mut v: u32) -> u32 {
    if v == 0 {
        return 0;
    }
    let mut remain = 32;
    while v & 15 << 28 == 0 {
        v <<= 4;
        remain -= 4;
    }
    let pos = 0x11111111;
    let mut bcd = 0u32;
    for _ in 0..remain {
        let overflow = bcd + 3 * pos & pos * 8;
        bcd = bcd.wrapping_add(bcd + (overflow >> 1) + (overflow >> 2));
        bcd += v >> 31;
        v <<= 1;
    }
    bcd
}

// Right justified.
fn format_fixed(result: &mut [u8], v: u32, sign: u8, dp: usize) {
    // Create the digits backwards.
    let mut v = to_bcd(v);
    let len = result.len();
    for i in 0..len {
        let d;
        if i != 0 && i == dp {
            d = b'.'
        }
        else {
            d = (v as u8 & 15) + b'0';
            v >>= 4;
        }
        result[len - 1 - i] = d;
    }

    // Discard leading zeros...
    let mut lead = 0;
    while result[lead] == b'0' && result[lead+1] != b'.' {
        result[lead] = b' ';
        lead += 1;
    }
    if sign == 0 {
        return;
    }
    if lead > 0 {
        result[lead - 1] = sign;
    }
    else {
        result[0] = sign;
    }
}

pub fn format_u32(result: &mut [u8], v: u32, dp: usize) {
    format_fixed(result, v, 0, dp)
}

pub fn format_i32(result: &mut [u8], v: i32, sign: bool, dp: usize) {
    let mut schar = 0;
    let mut abs = v as u32;
    if v < 0 {
        schar = b'-';
        abs = 0u32.wrapping_sub(abs);
    }
    else if sign {
        schar = b'+';
    }
    format_fixed(result, abs, schar, dp)
}

#[cfg(test)]
fn format_fixed_str(v: u32, sign: u8, width: usize, dp: usize) -> String {
    let mut result = vec![0u8; width];
    format_fixed(&mut result[..], v, sign, dp);
    result.into_iter().map(|x| x as char).collect()
}

#[test]
fn unsigned() {
    assert_eq!(format_fixed_str(12345, 0, 6, 3), "12.345");
    assert_eq!(format_fixed_str(456789, 0, 7, 2), "4567.89");
    assert_eq!(format_fixed_str(1, 0, 7, 2), "   0.01");
    assert_eq!(format_fixed_str(1234, 0, 8, 2), "   12.34");
    assert_eq!(format_fixed_str(1234, 0, 12, 2), "       12.34");
}

#[test]
fn signed() {
    assert_eq!(format_fixed_str(12345, b'-', 6, 3), "-2.345");
    assert_eq!(format_fixed_str(12345, b'-', 7, 3), "-12.345");
    assert_eq!(format_fixed_str(12345, b'-', 8, 3), " -12.345");
    assert_eq!(format_fixed_str(12345, b'+', 9, 3), "  +12.345");
}