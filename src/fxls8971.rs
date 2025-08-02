
use crate::i2c;

/// Accelerometer I2C address.
const FXLS8971: u8 = 0x30;

static STATE: crate::vcell::UCell<State> = Default::default();

#[derive_const(Default)]
struct State {
    accum: i32,
}

pub fn accel_init() -> crate::i2c::Result {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let exti  = unsafe {&*stm32u031::EXTI ::ptr()};

    crate::link_assert!(
        crate::cpu::VECTORS.isr[stm32u031::Interrupt::EXTI4_15 as usize]
        == accel_isr);

    // First set to standby—updating the interrupt config only appears to work
    // in standby.
    i2c::write(FXLS8971, &[0x15u8,
        0,                              // SENS_CONFIG1
        0x00,                           // SENS_CONFIG2, no decimation.
        0x99,                           // 6.25Hz.
        1,
        ]).wait()?;

    // Now set to interrupt active-high on DRDY.
    i2c::write(FXLS8971, &[0x20u8,
        0x80, // DRDY interrupt.
        0,    // Route interrupts to INT1.
        ]).wait()?;

    // Activat the device.
    i2c::write(FXLS8971, &[0x15u8,
        1,                              // SENS_CONFIG1
        0x00,                           // SENS_CONFIG2, no decimation.
        0x99,                           // 6.25Hz.
        1,
        ]).wait()?;

    // INT1 is on pin 25 = PA15.  Set up EXTI to interrupt on the rising edge
    //  of PA15.
    gpioa.MODER.modify(|_,w| w.MODE11().B_0x0().MODE15().B_0x0()); // GPI

    // Reduce the priority of the EXTI interrupt. EXTI4_15 is 7.
    const EXTI_INT: usize = stm32u031::Interrupt::EXTI4_15 as usize;
    const {assert!(EXTI_INT == 7)};
    #[cfg(not(test))]
    unsafe {
        let nvic = &*cortex_m::peripheral::NVIC::PTR;
        assert_eq!(&nvic.ipr[1] as *const _ as usize, 0xE000E400 + 4);
        nvic.ipr[1].write(0xc0 << 24);
    }

    // Enable the interrupt.
    exti.RTSR1.write(|w| w.RT15().set_bit());     // Rising edge trigger.
    // Default selection is port A anyway.
    exti.IMR1.write(|w| w.IM15().set_bit());

    unsafe {
        // Trigger the interrupt immediately, the line may already be asserted.
        let nvic = &*cortex_m::peripheral::NVIC::PTR;
        nvic.ispr[0].write(1 << EXTI_INT);
    }

    Ok(())
}

pub fn accel_isr() {
    let exti  = unsafe {&*stm32u031::EXTI ::ptr()};
    exti.RPR1.modify(|_,w| w);
    let mut buf = [0u8; 8];
    let Ok(_) = crate::i2c::read_reg(FXLS8971, 0, &mut buf[1..]).wait()
        else {return};

    // Check the status.  If no data ready then ignore it.
    if buf[1] & 0x80 == 0 {
        return;
    }

    // let x = i16::from_le_bytes(buf[2..4].try_into().unwrap()) as i32;
    let y = i16::from_le_bytes(buf[4..6].try_into().unwrap()) as i32;
    // let z = i16::from_le_bytes(buf[6..8].try_into().unwrap()) as i32;

    let state = unsafe {STATE.as_mut()};

    // Integrate and clamp at 1G = 1/2 FS = 1024.
    state.accum += y;
    let fs = 1024;
    if state.accum > fs {
        state.accum = fs;
        let _ = crate::oled::flip_screen(true);
    }
    else if state.accum < -fs {
        state.accum = -fs;
        let _ = crate::oled::flip_screen(false);
    }
}

impl crate::cpu::Config {
    pub const fn fxls8971(&mut self) -> &mut Self {
        self.isr(stm32u031::Interrupt::EXTI4_15, accel_isr)
    }
}
