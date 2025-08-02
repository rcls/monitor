
use crate::dma::DMA;
use crate::vcell::VCell;
use crate::dbgln;

// DMA MUX lines.
// Use DMA1 Ch1
pub const ADC_MUXIN: u32 = 5;

use crate::ADC_CHANNELS;
const NUM_CHANNELS: usize = ADC_CHANNELS.len();

pub static DMA_BUF: [VCell<u16>; NUM_CHANNELS] = [
    const {VCell::new(0)}; NUM_CHANNELS];
pub static OUTSTANDING: VCell<u8> = VCell::new(0);

const F_ADC: u8 = 1;
const F_DMA: u8 = 2;

pub fn start() {
    let adc = unsafe {&*stm32u031::ADC ::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};

    OUTSTANDING.write(F_ADC | F_DMA);

    // Set-up DMA for the ADC.
    dma.CH(0).read(DMA_BUF.as_ptr().addr(), NUM_CHANNELS, 1);

    // Trigger ADC conversions and temperature conversion (via I2C).
    adc.CR.write(|w| w.ADSTART().set_bit());
}

pub fn init1() {
    let adc    = unsafe {&*stm32u031::ADC   ::ptr()};
    let dma    = unsafe {&*stm32u031::DMA1  ::ptr()};
    let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};

    // Calibrate the ADC.  First enable the voltage reg.
    adc.CR.write(|w| w.ADVREGEN().set_bit());

    // DMAMUX mapping.  Note the zero-based v. one-based f**kup in the
    // datasheet.  By changing these to arrays we have both zero-based.
    dmamux.CCR[0].write(|w| w.bits(ADC_MUXIN));
    dma.CH(0).PAR.write(|w| w.bits(adc.DR.as_ptr().addr() as u32));

    // Wait for LDO ready... ≈20µs.
    for _ in 0..200 {
        crate::vcell::nothing();
    }
    // Start the calibration.
    adc.CR.write(|w| w.ADCAL().set_bit().ADVREGEN().set_bit());
}

pub fn init2() {
    let adc = unsafe {&*stm32u031::ADC::ptr()};

    // Wait for ADC calibration complete.
    dbgln!("Eocal wait");
    while OUTSTANDING.read() != 0 {
        crate::cpu::WFE();
    }
    dbgln!("Eocal done");
    dbgln!("ADC CR = {:#x}, ISR = {:#x}", adc.CR.read().bits(),
           adc.ISR.read().bits());

    // Enable the ADC and wait for ready.  This should be fast.
    adc.CR.write(|w| w.ADVREGEN().set_bit().ADEN().set_bit());
    while !adc.ISR.read().ADRDY().bit() {
    }

    // Oversample config.
    adc.CFGR2.write(
        |w| w.OVSS().B_0x4().OVSR().B_0x7().OVSE().set_bit()
            .LFTRIG().set_bit());
    adc.CFGR1.write(|w| w.DMAEN().set_bit());

    // Configure ADC chanels and wait for ready.  The datasheet appears to lie.
    // We actually get Isense on AIN9, VSENSE on A17 and VREF on A18 (0 based
    // v. 1 based?)
    const MASK: u32 = crate::utils::make_mask(&ADC_CHANNELS);
    adc.CHSELR.write(|w| w.bits(MASK));
    // Wait for ready.  This should be fast.
    dbgln!("Ccrdy wait");
    while !adc.ISR.read().CCRDY().bit() {
    }
    dbgln!("Ccrdy done");
    // Enable ADC EOC and OVR interrupts.
    adc.IER.write(|w| w.EOSIE().set_bit().OVRIE().set_bit());
    adc.SMPR.write(|w| w.SMP1().B_0x7()); // ≈10µs sample gate.
}

pub fn recalibrate_start() {
    let adc = unsafe {&*stm32u031::ADC::ptr()};
    // Disable & start calibrate.
    OUTSTANDING.write(F_ADC);
    adc.CR.write(|w| w.ADVREGEN().set_bit().ADCAL().set_bit());
}

pub fn adc_isr() {
    let adc = unsafe {&*stm32u031::ADC::ptr()};
    // Get EOC and OVR bits.
    let isr = adc.ISR.read();
    adc.ISR.write(|w| w.bits(isr.bits()));

    if isr.EOCAL().bit() {
        // Renable the ADC.
        adc.CR.write(|w| w.ADVREGEN().set_bit().ADEN().set_bit());
    }

    if isr.EOCAL().bit() || isr.EOS().bit() || isr.OVR().bit() {
        OUTSTANDING.write(OUTSTANDING.read() & !F_ADC);
    }
}

pub fn dma1_isr() {
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    let status = dma.ISR.read().bits();
    dma.IFCR.write(|w| w.CGIF1().set_bit());

    if status & 10 != 0 {
        dma.CH(0).CR.write(|w| w.bits(0));
    }
    if status & 10 != 0  {
        OUTSTANDING.write(OUTSTANDING.read() & !F_DMA);
    }
}

impl crate::cpu::Config {
    pub const fn adc(&mut self) -> &mut Self {
        use stm32u031::Interrupt::*;
        self.isr(ADC_COMP, adc_isr).isr(DMA1_CHANNEL1, dma1_isr)
            .clocks(1 << 0, 0, 1 << 20)
    }
}

#[test]
fn check_vtors() {
    use stm32u031::Interrupt::*;
    use crate::cpu::VECTORS;

    assert!(std::ptr::fn_addr_eq(VECTORS.isr[ADC_COMP as usize],
                                 adc_isr as fn()));
    assert!(std::ptr::fn_addr_eq(VECTORS.isr[DMA1_CHANNEL1 as usize],
                                 dma1_isr as fn()));
}
