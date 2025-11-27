// ENS212 / ENS220 testing.

const ENS220: u8 = 0x40;
const ENS212: u8 = 0x8a;

pub fn ens220id() {
    crate::i2c::init();
    let mut ids = [0u8; 12];
    if let Ok(()) = crate::i2c::read_reg(ENS220, 0, &mut ids).wait() {
        crate::dbgln!("ENS220 {ids:x?}");
    }
    else {
        crate::dbgln!("ENS220 fail");
    }

    let mut meas = [0u8; 5];
    let _ = crate::i2c::write(ENS220, &[6u8, 0x93]).wait();
    let _ = crate::i2c::write(ENS220, &[9u8, 0x3f]).wait();
    if let Ok(()) = crate::i2c::read_reg(ENS220, 0x17, &mut meas).wait() {
        let press = meas[0] as u32 + meas[1] as u32 * 256
            + meas[2] as u32 * 65536;
        let press = (press * 10 + 32) / 64;
        let temp = meas[3] as u32 + meas[4] as u32 * 256;
        let temp = (temp * 100 + 64) / 128 - 27315;
        crate::dbgln!("ENS220 press dPa {press}, temp cC {temp}")
    }
    else {
        crate::dbgln!("ENS220 fail");
    }
}

pub fn ens212id() {
    crate::i2c::init();
    let mut ids = [0u8; 12];
    // crate::dbgln!("About to ping ENS212");
    let _ = crate::i2c::write(ENS212, &[0x22u8, 3]).wait();

    if let Ok(()) = crate::i2c::read_reg(ENS212, 0, &mut ids).wait() {
        let ids = ids.as_ref();
        crate::dbgln!("ENS212 {ids:x?}");
    }
    else {
        crate::dbgln!("ENS212 fail");
    }

    let mut meas = [0u8; 6];
    if let Ok(()) = crate::i2c::read_reg(ENS212, 0x30, &mut meas).wait() {
        let temp = meas[0] as u32 + meas[1] as u32 * 256;
        //let temp = temp ^ 0x8000;
        let temp = (temp * 100 + 32) / 64 - 27315;
        let humi = meas[3] as u32 + meas[4] as u32 * 256;
        //let humi = humi ^ 0x8000;
        let humi = (humi * 100 + 256) / 512;
        crate::dbgln!("ENS212 humi c% {humi}, temp cC {temp}, {meas:x?}");
    }
    else {
        crate::dbgln!("ENS212 data read fail");
    }
}
