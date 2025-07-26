use crate::i2c;
use crate::vcell::nothing;

use i2c::Result;

const SH1106: u8 = 0x78;

const LINE_LENGTH: usize = 12;
pub type Line = [u8; LINE_LENGTH];
// type LineBits = [u8; 128];

pub fn reset() -> Result {
    i2c::write(
        SH1106,
        &[0u8,
          0x30,       // Set low pump voltage.
          0xad, 0x8b, // Charge pump enable.
          0x40,       // COM0 address.
          0x80,       // Default contrast.
          0xa1,       // Horizontal flip display.
          0xa6,       // Normal pixel polarity.
          0xa8, 0x3f, // Number of COM to use -1
          0xc8,       // Common direction - vertical flip.
          0xd3, 0x00, // First common line to use.
          0xd5, 0x50, // Clocking config.
          0xd9, 0x22, // discharge & precharge periods.
          0xda, 0x12, // Common signals pad config.
          0xaf]       // Display on.
        ).wait()
}

pub fn init() -> Result {
    i2c::write(
        SH1106,
        &[0u8,
          0xaeu8,        // Display off.
          0xdb, 0x35     // VCOM deselect level.
    ]).wait()?; // Common off voltage.
    reset()?;

    for _ in 0.. 1<<21 {
        nothing();
    }
    i2c::write(SH1106, &[0u8, 0xa4]).wait()?; // No all pixels on.
    for _ in 0.. 1<<21 {
        nothing();
    }
    clear_screen()
}

pub fn update_text(old: &mut Line, new: &[u8],
                   base: usize, y: u8) -> Result {
    let mut end;

    let mut iter = new.iter().zip(base..);
    let mut i;
    let mut n;
    loop {
        let Some(x) = iter.next() else {return Ok(())};
        (n, i) = x;
        if old[i] != *n {
            break;
        }
    }
    let start = i;
    'term: loop {
        old[i] = *n;
        end = i;
        for x in &mut iter {
            (n, i) = x;
            if old[i] != *n {
                continue 'term;
            }
        }
        break;
    }
    draw_chars(&new[start - base ..= end - base], start, y)
}

pub fn draw_chars(text: &[u8], start: usize, y: u8) -> Result {
    let mut data = arrayvec::ArrayVec::<u8, 129>::new();
    let mut dwait = crate::i2c::waiter(&data);

    for r in 0 ..= 1 {
        let y = y + r as u8;
        let line = 1 << y & 0x81;
        let bs = if start == 0 {2} else {start as u8 * 10 + 6};
        let command = [0u8, 0xb0 + y, 16 + (bs >> 4), bs & 15];
        dwait.wait()?;
        let cwait = i2c::write(SH1106, &command);
        data.clear();
        data.push(0x40);
        if start == 0 {
            data.extend([255, line, line, line]);
        }
        let row = &crate::font::FONT10X16[r];
        for &b in text {
            let item;
            if (b as usize) < row.len() {
                item = &row[b as usize];
            }
            else {
                item = &row[0];
            }
            for c in item {
                data.push(c | line);
            }
            // let _ = data.try_extend_from_slice(&row[b as usize & 31]);
        }
        if start + text.len() == LINE_LENGTH {
            data.extend([line, line, line, 255]);
        }
        cwait.wait()?;
        dwait = i2c::write(SH1106, data.as_slice());
    };
    dwait.wait()
}

pub fn clear_screen() -> Result {
    for y in (0..8).step_by(2) {
        draw_chars(&[crate::font::SPACE; LINE_LENGTH], 0, y)?
    }
    Ok(())
}

#[macro_export]
macro_rules! CHARS_MAP {
    ($s:expr) => (const {{
        const S: &str = $s;
        $crate::oled::chars_map::<{const{$crate::oled::char_len(S)}}>(S)
    }})
}

pub const fn char_len(s: &str) -> usize {
    let mut iter = konst::string::chars(s);
    let mut len = 0;
    while let Some((_, i)) = iter.next() {
        iter = i;
        len += 1;
    }
    len
}

pub const fn char_map(cc: char) -> u8 {
    let mut iter = konst::string::chars(crate::font::CHARS);
    let mut index = 0;
    if cc == ' ' {
        return 0;
    }
    while let Some((c, i)) = iter.next() {
        iter = i;
        if c == cc {
            return if index != 0 {index} else {128}
        }
        index += 1;
    }
    panic!()
}

pub const fn chars_map<const N: usize>(s: &str) -> [u8; N] {
    let mut result = [0; N];
    let mut iter = konst::string::chars(s);
    let mut index = 0;
    while let Some((c, i)) = iter.next() {
        iter = i;
        result[index] = char_map(c);
        index += 1;
    }
    assert!(index == N);
    result
}
