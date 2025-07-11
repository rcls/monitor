use crate::font::{DIGIT_0, PERIOD, SPACE};

// Right justified.
fn format_fixed(result: &mut [u8], v: u32, sign: u8, dp: usize) {
    // Create the digits backwards.
    let mut v = crate::utils::to_bcd(v);
    let len = result.len();
    for i in 0..len {
        let d;
        if i != 0 && i == dp {
            d = PERIOD
        }
        else {
            d = (v as u8 & 15) + DIGIT_0;
            v >>= 4;
        }
        result[len - 1 - i] = d;
    }

    // Discard leading zeros...
    let mut lead = 0;
    while result[lead] == DIGIT_0 && result[lead+1] != PERIOD {
        result[lead] = SPACE;
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

pub fn format_i32(result: &mut [u8], v: i32, dp: usize) {
    let schar;
    let abs;
    if v < 0 {
        schar = crate::font::MINUS;
        abs = 0u32.wrapping_sub(v as u32);
    }
    else {
        schar = 0;
        abs = v as u32;
    }
    format_fixed(result, abs, schar, dp)
}

#[cfg(test)]
fn ff_u32(v: u32, width: usize, dp: usize) -> String {
    let mut result = vec![0u8; width];
    format_u32(&mut result[..], v, dp);
    let chars: Vec<char> = crate::font::CHARS.chars().collect();
    result.into_iter().map(|x| chars[x as usize]).collect()
}

#[cfg(test)]
fn ff_i32(v: i32, width: usize, dp: usize) -> String {
    let mut result = vec![0u8; width];
    format_i32(&mut result[..], v, dp);
    let chars: Vec<char> = crate::font::CHARS.chars().collect();
    result.into_iter().map(|x| chars[x as usize]).collect()
}

#[test]
fn unsigned() {
    assert_eq!(ff_u32(12345, 6, 3) , "12.345");
    assert_eq!(ff_u32(456789, 7, 2), "4567.89");
    assert_eq!(ff_u32(1, 7, 2)     , "   0.01");
    assert_eq!(ff_u32(1234, 8, 2)  , "   12.34");
    assert_eq!(ff_u32(1234, 12, 2) , "       12.34");
}

#[test]
fn signed() {
    assert_eq!(ff_i32(45689, 6, 2), "456.89");
    assert_eq!(ff_i32(1, 7, 2)    , "   0.01");
    assert_eq!(ff_i32(-12345, 6, 3), "-2.345");
    assert_eq!(ff_i32(-12345, 7, 3), "-12.345");
    assert_eq!(ff_i32(-12345, 8, 3), " -12.345");
}
