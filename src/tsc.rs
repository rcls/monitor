use crate::vcell::VCell;

/// +, -, menu.  Note that the index is one less than the enumeration.
static VALUES: [VCell<u32>; 3] = [const {VCell::new(0)}; 3];
static PASS_COMPLETE: VCell<bool> = Default::default();

pub mod pad {
    pub const NONE : u32 = 0;
    pub const PLUS : u32 = 1;
    pub const MINUS: u32 = 2;
    pub const MENU : u32 = 3;
}

pub fn init() {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    let tsc = unsafe {&*stm32u031::TSC::ptr()};

    // FIXME - hardwired channels & values.
    if crate::CONFIG.is_lazy_tsc() {
        rcc.AHBENR.modify(|_,w| w.TSCEN().set_bit());
    }

    gpioa.AFRH.modify(
        |_,w| w.AFSEL8().B_0x9().AFSEL9().B_0x9().AFSEL10().B_0x9());
    gpiob.AFRL.modify(|_,w| w.AFSEL0().B_0x9());
    gpiob.AFRH.modify(|_,w| w.AFSEL11().B_0x9());

    // Set open drain mode for the sampling cap pins.
    gpioa.OTYPER.modify(|_,w| w.OT9().B_0x1());
    gpiob.OTYPER.modify(|_,w| w.OT0().B_0x1());

    // Enable AF.
    gpioa.MODER.modify(|_,w| w.MODE8().B_0x2().MODE9().B_0x2().MODE10().B_0x2());
    gpiob.MODER.modify(|_,w| w.MODE0().B_0x2().MODE11().B_0x2());

    // Prescale to 2Mhz.
    let prescaler = match crate::CONFIG.clk {
        2000000 => 0,
        16000000 => 3,
        _ => crate::utils::unreachable(),
    };
    //let prescaler = prescaler * 0 + 6;
    // TODO - timings are guesses...
    // Standard pulse range "500ns to 2µs".
    // Use 3 clock = 1.5µs on high and low pulse.
    // Spread-sprectrum on but as low as possible.
    // Max counts is set to 2048, values much below that count as a touch.
    tsc.CR.write(
        |w| w.CTPH().bits(2).CTPL().bits(2)
            . SSD().bits(0). SSE().set_bit().SSPSC().set_bit()
            . PGPSC().bits(prescaler).MCV().B_0x3().TSCE().set_bit());
    // Turning off hysteresis for the cap-sense lines seems to improve
    // sensitivity.
    tsc.IOHCR.write(
        |w| w.G7_IO2().clear_bit(). G5_IO2().clear_bit());
    // Enable end-of-acquire interrupt.
    tsc.IER.write(|w| w.EOAIE().set_bit());
}

pub fn acquire() {
    // Try + and menu on the first pass (they are physically separate), and
    // then - on the second pass.
    // FIXME - how do we select channels?

    // Minus = G7IO1
    // CS = G7IO2
    // Plus = G7IO3
    // Menu = G5IO4
    // MenuCS = G5IO2
    PASS_COMPLETE.write(false);
    let tsc = unsafe {&*stm32u031::TSC::ptr()};
    // + and cap.
    tsc.IOCCR.write(|w| w.G7_IO3().set_bit().G5_IO4().set_bit());
    tsc.IOSCR.write(|w| w.G7_IO2().set_bit().G5_IO2().set_bit());
    tsc.IOGCSR.write(|w| w.G7E().set_bit().G5E().set_bit());
    tsc.CR.modify(|_,w| w.START().set_bit());
}

fn isr() {
    let tsc = unsafe {&*stm32u031::TSC::ptr()};
    let sr = tsc.ISR.read().bits();
    tsc.ICR.write(|w| w.bits(sr));

    let ccr = tsc.IOCCR.read();
    if ccr.G7_IO3().bit() { // +
        VALUES[0].write(tsc.IOG7CR.read().bits());
    }
    if ccr.G5_IO4().bit() { // Menu
        VALUES[2].write(tsc.IOG5CR.read().bits());
    }
    if ccr.G7_IO1().bit() { // -
        // We got -, the pass is done.
        VALUES[1].write(tsc.IOG7CR.read().bits());
        PASS_COMPLETE.write(true);
    }
    else {
        // We did not get -, get it now.
        tsc.IOCCR.write(|w| w.G7_IO1().set_bit());
        tsc.IOSCR.write(|w| w.G7_IO2().set_bit());
        tsc.IOGCSR.write(|w| w.G7E().set_bit());
        tsc.CR.modify(|_,w| w.START().set_bit());
    }
}

// Return touched pad, 1=+, 2=-, 3=menu, 0=None.
pub fn retrieve() -> u32 {
    while !PASS_COMPLETE.read() {
        crate::cpu::WFE();
    }
    // Find the lowest line...
    let mut min_line = 0;
    let mut min_val = VALUES[0].read();
    for (i, val) in VALUES.iter().enumerate().skip(1) {
        let val = val.read();
        if val < min_val {
            min_val = val;
            min_line = i as u32;
        }
    }
    if min_val < 1536 {
        min_line + 1
    }
    else {
        0
    }
}

impl crate::cpu::Config {
    pub const fn lazy_tsc(&mut self) -> &mut Self {
        self.isr(stm32u031::Interrupt::TSC, isr)
    }
    #[allow(dead_code)]
    pub const fn tsc(&mut self) -> &mut Self {
        self.lazy_tsc().clocks(1 << 24, 0, 0)
    }
    pub fn is_lazy_tsc(&self) -> bool {
        self.ahb_clocks & 1 << 24 == 0
    }
}

#[test]
fn check_isr() {
    assert!(crate::cpu::VECTORS.isr[stm32u031::Interrupt::TSC as usize]
        == isr);
}
