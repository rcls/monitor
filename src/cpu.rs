use crate::{link_assert, CONFIG};

#[derive(Clone, Copy, Default)]
pub struct Config {
    pub clk: u32,
    pub vectors: VectorTable,
    pub ahb_clocks: u32,
    pub apb1_clocks: u32,
    pub apb2_clocks: u32,
    pub interrupts: u32,
    /// Pull-ups to enable, bits 0..=15 are GPIOA, 16..=31 are GPIOB etc.
    pub pullup: u64,
    /// Pull-downs to enable.
    pub pulldown: u64,
    /// GPIO bits that should be pulled up in standby.
    #[allow(dead_code)]
    pub standby_pu: u64,
    /// GPIO bits that should be pulled down in standby.
    #[allow(dead_code)]
    pub standby_pd: u64,
    /// GPIO bits that should be pulled up in standby if already hi.
    #[allow(dead_code)]
    pub keep_pu: u64,
    /// GPIO bits that should be pulled down in standby if already lo.
    #[allow(dead_code)]
    pub keep_pd: u64,
    /// GPIO bits that should have PU/PD preserved on wake-up.
    /// FIXME - we can get rid of this.
    pub wakeup_pupd: u64,
    /// Use PWR for pull-up/pull-down rather than GPIOs.
    pub pupd_use_pwr: bool,
    /// Enable the MSI FLL.
    pub fll: bool,
}

#[used]
#[unsafe(link_section = ".vectors")]
pub static VECTORS: VectorTable = CONFIG.vectors;

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

pub const MSIRANGE: u8 = const {
    match CONFIG.clk {
        16000000 => 8,
        2000000 => 5,
        _ => panic!("CPU_CLK not implemented")
    }
};

// TODO - cut the Geordian knot and leave PUPD set-up to after init.
// Or runtime parameters.
// Or set in init1, release standby-only in init2?
fn pupd_via_pwr() {
    let pwr = unsafe {&*stm32u031::PWR::ptr()};
    #[inline]
    fn setup(shift: u32, pucr: &stm32u031::pwr::PUCRA, pdcr: &stm32u031::pwr::PDCRA) {
        let pullup   = (CONFIG.pullup      >> shift & 0xffff) as u32;
        let pulldown = (CONFIG.pulldown    >> shift & 0xffff) as u32;
        let wakeup   = (CONFIG.wakeup_pupd >> shift & 0xffff) as u32;
        //let sb = CONFIG.standby_io  >> shift & 0xffff;
        if pullup != 0 || wakeup != 0 {
            let pp = if wakeup != 0 {pucr.read().bits() & wakeup} else {0};
            pucr.write(|w| w.bits(pullup | pp));
        }
        if pulldown != 0 || wakeup != 0 {
            let pp = if wakeup != 0 {pdcr.read().bits() & wakeup} else {0};
            pdcr.write(|w| w.bits(pulldown | pp));
        }
    }
    setup( 0, &pwr.PUCRA, &pwr.PDCRA);
    setup(16, &pwr.PUCRB, &pwr.PDCRB);
    setup(32, &pwr.PUCRC, &pwr.PDCRC);

    pwr.CR3.modify(|_,w| w.APC().set_bit());
}

fn pupd_via_gpio() {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let gpioc = unsafe {&*stm32u031::GPIOC::ptr()};

    // Apply pull up / pull down.
    // Note that the preserves are not processed.
    if CONFIG.pupd(0) != 0x24000000 {
        gpioa.PUPDR.write(|w| w.bits(CONFIG.pupd(0)));
    }
    if CONFIG.pupd(1) != 0 {
        gpiob.PUPDR.write(|w| w.bits(CONFIG.pupd(1)));
    }
    if CONFIG.pupd(2) != 0 {
        gpioc.PUPDR.write(|w| w.bits(CONFIG.pupd(2)));
    }
}

pub fn init1() {
    let scb   = unsafe {&*cortex_m::peripheral::SCB::PTR};
    let pwr   = unsafe {&*stm32u031::PWR  ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};

    rcc.AHBENR .write(|w| w.bits(CONFIG.ahb_clocks));
    rcc.APBENR1.write(|w| w.bits(CONFIG.apb1_clocks));
    rcc.APBENR2.write(|w| w.bits(CONFIG.apb2_clocks));

    // Use MSI at appropriate frequency.
    rcc.CR.write(
        |w| w.MSIRANGE().bits(MSIRANGE).MSIRGSEL().set_bit().MSION().set_bit());

    // Enable backup domain access & maybe set lower-power run.
    let low_power = CONFIG.clk <= 2000000;
    pwr.CR1.write(|w| w.LPR().bit(low_power).DBP().set_bit());

    if CONFIG.pupd_use_pwr {
        pupd_via_pwr();
    }
    else {
        pupd_via_gpio();
    }

    // Clear the BSS.
    if !cfg!(test) {
        crate::vcell::barrier();
        // The rustc memset is hideous.
        let mut p = (&raw mut __bss_start) as *mut u32;
        loop {
            unsafe {*p = 0};
            p = p.wrapping_add(1);
            if p as *mut u8 >= &raw mut __bss_end {
                break;
            }
        }
        crate::vcell::barrier();
    }

    // We use sev-on-pending to avoid trivial interrupt handlers.
    unsafe {scb.scr.write(16)};

    // Enable the LSE.  Set high drive strength for crystal start-up.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x3());
}

pub fn init2() {
    let rcc  = unsafe {&*stm32u031::RCC::ptr()};
    let scb  = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    let nvic = unsafe {&*cortex_m::peripheral::NVIC::PTR};

    if CONFIG.vectors.systick != bugger {
        // Set the systick interrupt priority to a high value (other
        // interrupts pre-empt).
        // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
        // address.  We want SHPR3.
        link_assert!(core::ptr::from_ref(&scb.shpr[1]) as usize == 0xE000ED20);
        #[cfg(not(test))]
        unsafe {
            scb.shpr[1].write(0xc0 << 24);
        }
    }

    // Enable the LSE interrupt, wait for it and then enable the MSI FLL.
    rcc.CIER.write(|w| w.LSERDYIE().set_bit());

    // Wait for LSE.
    while !rcc.BDCR.read().LSERDY().bit() {
        WFE();
    }
    rcc.CIER.write(|w| w.LSIRDYIE().clear_bit());
    rcc.CICR.write(|w| w.bits(!0));

    if CONFIG.fll {
        rcc.CR.write(
            |w| w.MSIRANGE().bits(MSIRANGE).MSIRGSEL().set_bit()
                . MSION().set_bit().MSIPLLEN().set_bit());
    }

    // Reduce drive strength.  (Lowest drive strength appears unreliable).
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x1());

    // Enable interrupts.  reserved1 has been abused to track what to enable.
    unsafe {
        nvic.iser[0].write(CONFIG.interrupts);
    }
}

impl Config {
    pub const fn new(clk: u32) -> Config {
        Config{
            clk, vectors: VectorTable::default(),
            // Always enable PWR clock.
            ahb_clocks: 0, apb1_clocks: 1 << 28, apb2_clocks: 0,
            interrupts: 0,
            pullup: 1 << 13, pulldown: 1 << 14,
            standby_pu: 0, standby_pd: 0, keep_pu: 0, keep_pd: 0,
            wakeup_pupd: 0,
            fll: true, pupd_use_pwr: false}
    }
    pub const fn isr(&mut self,
                     isr: stm32u031::Interrupt, handler: fn()) -> &mut Self {
        self.vectors.isr[isr as usize] = handler;
        self.interrupts |= 1 << isr as u32;
        self
    }
    #[allow(dead_code)]
    pub const fn systick(&mut self, handler: fn()) -> &mut Self {
        self.vectors.systick = handler;
        self
    }
    pub const fn clocks(&mut self, hb: u32, pb1: u32, pb2: u32) -> &mut Self {
        self.ahb_clocks |= hb;
        self.apb1_clocks |= pb1;
        self.apb2_clocks |= pb2;
        self
    }
    pub const fn pupd(&self, port: u32) -> u32 {
        let up   = (self.pullup   >> port * 16) as u16;
        let down = (self.pulldown >> port * 16) as u16;
        crate::utils::spread16(up) + crate::utils::spread16(down) * 2
    }
}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct VectorTable {
    pub stack     : u32,
    pub reset     : fn() -> !,
    pub nmi       : fn(),
    pub hard_fault: fn(),
    pub reserved1 : [u32; 7],
    pub svcall    : fn(),
    pub reserved2 : [u32; 2],
    pub pendsv    : fn(),
    pub systick   : fn(),
    pub isr       : [fn(); 32],
}

impl const Default for VectorTable {
    fn default() -> Self {
        VectorTable{
            stack     : 0x20000000 + 12 * 1024,
            reset     : crate::main,
            nmi       : bugger,
            hard_fault: bugger,
            reserved1 : [0; 7],
            svcall    : bugger,
            reserved2 : [0; 2],
            pendsv    : bugger,
            systick   : bugger,
            isr       : [bugger; 32]}
    }
}

fn bugger() {
    panic!("Unexpected interrupt");
}

#[inline(always)]
#[allow(non_snake_case)]
pub fn WFE() {
    if cfg!(target_arch = "arm") {
        unsafe {
            core::arch::asm!("wfe", options(nomem, preserves_flags, nostack))};
    }
    else {
        panic!("wfe!");
    }
}

#[test]
fn default_pupd() {
    let config = Config::new(16000000);
    assert_eq!(config.pupd(0), 0x24000000);
    assert_eq!(config.pupd(1), 0);
    assert_eq!(config.pupd(2), 0);
}
