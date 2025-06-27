
use crate::dma::DMA;
use crate::vcell::VCell;
use crate::dbgln;

// DMA MUX lines.
// Use DMA1 Ch1
pub const ADC_MUXIN: u32 = 5;

pub static DMA_BUF: [VCell<u16>; 3] = [const {VCell::new(0)}; 3];
pub static DONE: VCell<bool> = VCell::new(false);

#[inline]
fn nothing() {
    unsafe {core::arch::asm!("", options(nostack, nomem, preserves_flags))}
}

pub fn start() {
    let adc = unsafe {&*stm32u031::ADC ::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};

    DONE.write(false);

    // Set-up DMA for the ADC.
    dma.CH(0).read(DMA_BUF.as_ptr().addr(), 3, 1);

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
    dmamux.CCR(0).write(|w| unsafe {w.bits(ADC_MUXIN)});
    dma.CH(0).PAR.write(|w| unsafe {w.bits(adc.DR.as_ptr().addr() as u32)});

    // Wait for LDO ready... ≈20µs.
    for _ in 0..200 {
        nothing();
    }
    // Start the calibration.
    adc.CR.write(|w| w.ADCAL().set_bit().ADVREGEN().set_bit());
}

pub fn init2() {
    let adc = unsafe {&*stm32u031::ADC::ptr()};

    // Wait for ADC calibration complete.
    dbgln!("Eocal wait");
    while !adc.ISR.read().EOCAL().bit() {
    }
    adc.ISR.write(|w| w.EOCAL().set_bit());
    dbgln!("Eocal done");
    dbgln!("ADC CR = {:#x}, ISR = {:#x}", adc.CR.read().bits(),
           adc.ISR.read().bits());

    // Enable the ADC and wait for ready...
    dbgln!("ADC CR = {:#x}, ISR = {:#x}", adc.CR.read().bits(),
           adc.ISR.read().bits());
    adc.CR.write(|w| w.ADVREGEN().set_bit().ADEN().set_bit());
    dbgln!("ADC CR = {:#x}, ISR = {:#x}", adc.CR.read().bits(),
           adc.ISR.read().bits());
    while !adc.ISR.read().ADRDY().bit() {
    }

    // Use DMAEN, and set AUTOFF for intermittent conversions.
    adc.CFGR1.write(|w| w.AUTOFF().set_bit().DMAEN().set_bit());
    // Oversample config.
    adc.CFGR2.write(|w| w.OVSS().B_0x4().OVSR().B_0x7().OVSE().set_bit());

    // Configure ADC chanels and wait for ready.  The datasheet appears to lie.
    // We actually get Isense on AIN9, VSENSE on A17 and VREF on A18 (0 based
    // v. 1 based?)
    adc.CHSELR.write(
        |w| w.CHSEL9().set_bit().CHSEL17().set_bit().CHSEL18().set_bit());
    // Wait for ready.
    dbgln!("Ccrdy wait");
    while !adc.ISR.read().CCRDY().bit() {
    }
    dbgln!("Ccrdy done");
    // Enable ADC EOC and OVR interrupts.
    adc.IER.write(|w| w.EOSIE().set_bit().OVRIE().set_bit());
    adc.SMPR.write(|w| w.SMP1().B_0x7()); // ≈10µs sample gate.
}

pub fn adc_isr() {
    let adc = unsafe {&*stm32u031::ADC ::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    // Get EOC and OVR bits.
    let isr = adc.ISR.read().bits();
    adc.ISR.write(|w| unsafe{w.bits(isr & 0x18)});
    // sdbgln!("ADC ISR {isr:#x}");
    // FIXME - abort if OVR is set.
    if dma.CH(0).NDTR.read().NDT().bits() == 0 {
        // sdbgln!("ADC done");
        DONE.write(true);
    }
    // sdbgln!("{:#x} {:#x} {:#x} {:#x} {:#x}",
    //        dma.ISR.read().bits(), dma.CCR1.read().bits(),
    //        dma.CNDTR1.read().bits(), dma.CPAR1.read().bits(),
    //        dma.CMAR1.read().bits());
    // let dmamux = unsafe {&*stm32u031::DMAMUX::ptr()};
    // sdbgln!(" {:#x}", dmamux.C0CR.read().bits());
}

pub fn dma1_isr() {
    let adc = unsafe {&*stm32u031::ADC::ptr()};
    let dma = unsafe {&*stm32u031::DMA1::ptr()};
    let status = dma.ISR.read().bits();
    dma.IFCR.write(|w| w.CGIF1().set_bit());
    // sdbgln!("DMA1 ISR {status:#x}");
    if status & 10 != 0 {
        dma.CH(0).CR.write(|w| unsafe {w.bits(0)});
    }
    if status & 10 != 0 && adc.CR.read().ADSTART().bit() {
        // sdbgln!("ADC DONE");
        DONE.write(true);
    }
}
