#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]
#![deny(warnings)]
#![feature(format_args_nl)]
#![feature(sync_unsafe_cell)]
#![feature(str_from_raw_parts)]

mod debug;
mod vcell;

use vcell::nothing;

type I2C = stm32u031::i2c1::RegisterBlock;
const CPU_CLK: u32 = 16000000;

const TMP117: u8 = 0x92;

fn i2c_tx_byte(i2c: &I2C, val: u8) -> Option<()> {
    loop {
        let status = i2c.ISR.read();
        i2c.ICR.write(|w| w.bits(status.bits() & 0x12));
        if status.NACKF().bit() {
            return None;
        }
        if status.TXIS().bit() {
            break;
        }
    }
    i2c.TXDR.write(|w| w.TXDATA().bits(val));
    Some(())
}

fn i2c_rx_byte(i2c: &I2C) -> u8 {
    while !i2c.ISR.read().RXNE().bit() {
    }
    i2c.RXDR.read().RXDATA().bits()
}

fn i2c_wait_tc(i2c: &I2C) -> Option<()> {
    loop {
        let status = i2c.ISR.read();
        i2c.ICR.write(|w| w.bits(status.bits() & 0x50));
        if status.NACKF().bit() {
            return None;
        }
        if status.TC().bit() {
            return Some(());
        }
    }
}

fn i2c_tx_reg16(i2c: &I2C, addr: u8, reg: u8, val: u16, last: bool)
                -> Option<()> {
    i2c.CR2.write(
        |w| w.START().set_bit().AUTOEND().bit(last)
            . SADD().bits(addr as u16).NBYTES().bits(3));
    i2c_tx_byte(i2c, reg)?;
    i2c_tx_byte(i2c, (val >> 8) as u8)?;
    i2c_tx_byte(i2c, val as u8)?;
    Some(())
}

fn i2c_rx_reg16(i2c: &I2C, addr: u8, reg: u8, last: bool) -> Option<u16> {
    i2c.CR2.write(
        |w| w.START().set_bit(). SADD().bits(addr as u16).NBYTES().bits(1));
    i2c_tx_byte(i2c, reg)?;
    i2c_wait_tc(i2c);
    i2c.CR2.write(
        |w| w.RD_WRN().set_bit().START().set_bit().AUTOEND().bit(last)
            . SADD().bits(addr as u16).NBYTES().bits(2));
    let byte1 = i2c_rx_byte(i2c);
    let byte2 = i2c_rx_byte(i2c);
    Some(byte1 as u16 * 256 + byte2 as u16)
}

pub fn main() -> ! {
    let gpioa = unsafe {&*stm32u031::GPIOA ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC   ::ptr()};
    let usart = unsafe {&*stm32u031::USART2::ptr()};
    let i2c   = unsafe {&*stm32u031::I2C1  ::ptr()};
    let pwr   = unsafe {&*stm32u031::PWR   ::ptr()};
    let adc   = unsafe {&*stm32u031::ADC   ::ptr()};

    // Go to 16MHz.
    rcc.CR.write(
        |w| w.MSIRANGE().B_0x8().MSIRGSEL().set_bit().MSION().set_bit());
    rcc.IOPENR.write(|w| w.GPIOAEN().set_bit());
    rcc.APBENR1.write(|w| w.PWREN().set_bit().USART2EN().set_bit().I2C1EN().set_bit());
    rcc.APBENR2.write(|w| w.ADCEN().set_bit());

    // Enable backup domain access.
    pwr.CR1.write(|w| w.DBP().set_bit()); // Already uppercase, no change

    // Enable the LSE.  Set high drive strength.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x3()); // Already uppercase, no change

    // Pullups on I2C and USART RX pin.
    gpioa.PUPDR.write(
        |w| w.PUPD3().B_0x1().PUPD9().B_0x1().PUPD10().B_0x1());

    // Set I2C pins to open drain - FIXME - is this needed?
    gpioa.OTYPER.write(|w| w.OT9().set_bit().OT10().set_bit());

    // Set-up the UART TX.
    usart.BRR.write(|w| unsafe {w.BRR().bits(139)});
    usart.CR1.write(|w| w.FIFOEN().set_bit().TE().set_bit().UE().set_bit());

    // PA2,3, USART2, AF7.
    // PA9,10, I2C1, SCL, SDA, AF4.
    gpioa.AFRL.write(|w| w.AFSEL2().B_0x7().AFSEL3().B_0x7());
    gpioa.AFRH.write(|w| w.AFSEL9().B_0x4().AFSEL10().B_0x4());

    gpioa.MODER.modify(|_, w| w
                       .MODE2().B_0x2() // USART
                       .MODE3().B_0x2()
                       .MODE9().B_0x2() // I2C.
                       .MODE10().B_0x2()
                       .MODE11().B_0x1() // GPO.
                       .MODE12().B_0x1());

    debug::init();

    // ADC.  PA5=AIN10=ISense, PB0=AIN18=VSense, PB1=AIN19=VRef.
    // iTemp=AIN11, TSEN, iRef=AIN12, VREFEN
    // TSCal1 (30°, 3V) @ 0x1FFF 6E68 - 0x1FFF 6E69
    // TSCal2 (130°, 3V) @ 0x1FFF 6E8A - 0x1FFF 6E8B
    // VRefInt (30°C, 3V) @ 0x1FFF 6EA4 - 0x1FFF 6EA5

    // Calibrate the ADC.  First enable the voltage reg.
    adc.CR.write(|w| w.ADVREGEN().set_bit());
    // Wait for LDO ready... 20µs.
    for _ in 0..200 {
        nothing();
    }
    // Start the calibration.
    adc.CR.write(|w| w.ADCAL().set_bit().ADVREGEN().set_bit());
    // Wait for it.
    sdbgln!("A1");
    while !adc.ISR.read().EOCAL().bit() {
    }
    sdbgln!("A2");

    adc.CR.write(|w| w.ADVREGEN().set_bit().ADEN().set_bit());

    for _ in 0..1000000 {
        if adc.ISR.read().ADRDY().bit() {
            sdbgln!("ADRDY");
            break;
        }
    }
    //}
    sdbgln!("A3");

    adc.SMPR.write(|w| w.SMP1().B_0x7()); // 10µs sample gate.
    //adc.cr.write(|w| w.advregen().set_bit().aden().set_bit());
    //adc.chselr.write(
    //|w| w.chsel10().set_bit().chsel11().set_bit().chsel12().set_bit()
    //. chsel18().set_bit().chsel19().set_bit());
    //adc.chselr.write(|w| unsafe{w.bits(0xfffff)});
    // The datasheet appears to lie.  We actually get Isense on AIN9,
    // VSENSE on A17 and VREF on A18 (0 based v. 1 based?)
    adc.CHSELR.write(
        |w| w.CHSEL9().set_bit().CHSEL17().set_bit().CHSEL18().set_bit());

    // AUTOFF?
    // WAIT?
    // CONT?
    // LFTRIG?
    adc.CFGR1.write(|w| w.DISCEN().set_bit().AUTOFF().set_bit());
    adc.CFGR2.write(|w| w.OVSS().B_0x4().OVSR().B_0x7().OVSE().set_bit());

    // Configure I2C.
    // 50kb/s from 16MHz.
    #[cfg(false)]
    i2c.TIMINGR.write(unsafe{
        |w| w.PRESC ().bits(7)
            . SCLL  ().bits(19)
            . SCLH  ().bits(15)
            . SDADEL().bits(2)
            . SCLDEL().bits(4)});
    // 400kb/s from 16MHz, STM documented timings.
    #[cfg(false)]
    i2c.TIMINGR.write(unsafe{
        |w| w.PRESC ().bits(1)
            . SCLL  ().bits(9)
            . SCLH  ().bits(3)
            . SDADEL().bits(2)
            . SCLDEL().bits(3)});
    // 400kb/s from 16MHz, longer clock pulse.
    i2c.TIMINGR.write(
        |w| w.PRESC ().bits(1)
            . SCLL  ().bits(3)
            . SCLH  ().bits(9)
            . SDADEL().bits(1)
            . SCLDEL().bits(1)});

    // CR1 - only need PE?
    i2c.CR1.write(|w| w.PE().set_bit());

    // Wait for LSE and then enable the MSI FLL.
    gpioa.BSRR.write(|w| w.bits(1 << 11 | 1 << 12));
    while !rcc.BDCR.read().LSERDY().bit() {
    }
    gpioa.BSRR.write(|w| w.bits(1 << 27 | 1 << 28));
    rcc.CR.write(
        |w| w.MSIRANGE().B_0x8().MSIRGSEL().set_bit().MSION().set_bit()
            . MSIPLLEN().set_bit());
    // Reduce drive strength.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x0());

    for r in 0..= 8 {
        sdbgln!("Reg {r} = {v:?}", v = i2c_rx_reg16(i2c, TMP117, r, true));
    }

    let mut adc_idx = 0;
    loop {
        gpioa.BSRR.write(|w| w.bits(1 << 27 | 1 << 12));
        uart.TDR.write(|w| w.bits('U' as u32));

        // Trigger a temperature conversion.
        i2c_tx_reg16(i2c, TMP117, 1, 0xc, true);

        for _ in 0..2666666 {
            nothing();
        }
        gpioa.BSRR.write(|w| w.bits(1 << 28 | 1 << 11));
        uart.TDR.write(|w| w.bits('y' as u32));

        // Read the temperature conversion.
        if let Some(temp) = i2c_rx_reg16(i2c, TMP117, 0, true) {
            let cels = (temp * 5 + 32) / 64;
            sdbgln!("T {cels}");
            //sdbgln!("T {}", cels);
        }

        // Trigger a ADC conversion.  We probably don't need all these bits?
        adc.CR.write(|w| w.ADVREGEN().set_bit().ADEN().set_bit()
                     .ADSTART().set_bit());

        for _ in 0..2666666 {
            nothing();
        }
        // Read an ADC measurement.
        let adc_st = adc.ISR.read().bits();
        adc.ISR.write(|w| w.bits(12));
        if adc_st & 4 != 0 {
            adc_idx += 1;
            let val = adc.DR.read().DATA().bits() as i32;
            match adc_idx {
                // 132 mV/A
                1 => sdbgln!(
                    "Isense: {} mA ({val})",
                    ((val - 2034*16) * 25000 + 32768) / 65536),
                2 => sdbgln!(
                    "Vsense: {} mV ({val})",
                    val * (3300 * 118 / 24) / (65536 * 18 / 24)),
                // 33/(val / 4096 / 25)
                // 1v gets val/2.5
                // 4096 gets 4096 / (val/2.5)
                3 => sdbgln!("3V3: {} mV ({val})", 65536 * 2500 / val),
                _ => (),
            }
            if adc_st & 8 != 0 {
                adc_idx = 0;
            }
        }
    }
}

fn hang() {
    loop { }
}

#[repr(C)]
pub struct VectorTable {
    stack     : u32,
    reset     : fn() -> !,
    nmi       : fn(),
    hard_fault: fn(),
    reserved1 : [u32; 7],
    svcall    : fn(),
    reserved2 : [u32; 2],
    pendsv    : fn(),
    systick   : fn(),
    interrupts: [fn(); 32],
}

#[used]
#[unsafe(link_section = ".vectors")]
pub static VTORS: VectorTable = VectorTable {
    stack     : 0x20000000 + 12 * 1024,
    reset     : main,
    nmi       : hang,
    hard_fault: hang,
    reserved1 : [0; 7],
    svcall    : hang,
    reserved2 : [0; 2],
    pendsv    : hang,
    systick   : hang,
    interrupts: [hang; 32],
};
