use crate::i2c;

const SH1106: u8 = 0x78;

#[inline]
fn nothing() {
    unsafe {core::arch::asm!("", options(nomem))}
}

const LINE_LENGTH: usize = 12;
pub type Line = [u8; LINE_LENGTH];
// type LineBits = [u8; 128];

pub fn init() {
    i2c::write(
        SH1106,
        &[0u8,
          0xaeu8,     // Display off.
          0x32,       // Default charge pump voltage.
          0xad, 0x8b, // Charge pump enable.
          0x00,       // Column 0 address low nibble.
          0x10,       // Column 0 address high nibble.
          0x40,       // COM0 address.
          0x80,       // Default contrast.
          0xa1,       // Horizontal flip display.
          0xa6,       // Normal pixel polarity.
          0xa8, 0x3f, // Number of COM to use -1
          0xc8,       // Common direction - vertical flip.
          0xd3, 0x30, // First common line to use - 48/64
          0xb5, 0x50, // Clocking config.
          0xb9, 0x22, // discharge & precharge periods.
          0xba, 0x12, // Common signals pad config.
          0xdb, 0x35, // Common off voltage.
          0xaf,       // Display on.
          0xa5],      // All pixels on.
        );

    for _ in 0.. 1<<21 {
        nothing();
    }
    i2c::write(SH1106, &[0u8, 0xa4]); // No all pixels on.
    for _ in 0.. 1<<21 {
        nothing();
    }
    outer_box();
    for _ in 0.. 1<<21 {
        nothing();
    }

    if false {
        let mut frame = [[0u8; LINE_LENGTH]; 4];
        refresh_line(&mut frame[0], b"<Testing123>", 0);
        refresh_line(&mut frame[1], b"--foo  bar--", 2);
        refresh_line(&mut frame[2], b"            ", 4);
        refresh_line(&mut frame[0], b"<TestABC123>", 4);
        refresh_line(&mut frame[0], b"[+-1234shag]", 6);

        for _ in 0.. 1<<24 {
            nothing();
        }
    }
}

pub fn refresh_line(old: &mut Line, new: &Line, y: u8) {
    let start;
    'find: {
        for (i, &n) in new.iter().enumerate() {
            if old[i] != n {
                start = i;
                break 'find;
            }
        }
        return; // Nothing to do.
    }
    let mut end = LINE_LENGTH;
    while new[end - 1] == old[end - 1] {
        end -= 1;
    }
    for r in 0 ..= 1 {
        let mut data = arrayvec::ArrayVec::<u8, 129>::new();
        data.push(0x40);
        let bs = start as u8 * 10 + 6;
        let command = [0u8, 0xb0 + y + r as u8, 16 + (bs >> 4), bs & 15];
        i2c::CONTEXT.wait();
        i2c::write(SH1106, &command).defer();
        for &b in &new[start .. end] {
            use crate::font::FONT10X16;
            let _ = data.try_extend_from_slice(&FONT10X16[r][b as usize - 32]);
        }
        i2c::CONTEXT.wait();
        i2c::write(SH1106, data.as_slice()).defer();
    }
    old[start .. end].copy_from_slice(&new[start .. end]);
    i2c::CONTEXT.wait();
}

pub fn outer_box() {
    let mut row = [0x40u8; 129 + 4];
    let mut command = [0u8, 0, 2, 0x10];
    // Top row...
    row[1] = 255;
    for i in 2..128 {
        row[i] = 1;
    }
    row[128] = 255;
    command[1] = 0xb0;
    i2c::write(SH1106, &command);
    i2c::write(SH1106, &row);
    for i in 2..128 {
        row[i] = 0;
    }
    for i in 1..=6 {
        command[1] = 0xb0 + i;
        i2c::write(SH1106, &command);
        i2c::write(SH1106, &row);
    }
    for i in 2..128 {
        row[i] = 128;
    }
    command[1] = 0xb7;
    i2c::write(SH1106, &command);
    i2c::write(SH1106, &row);
}
