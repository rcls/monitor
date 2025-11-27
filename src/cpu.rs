use crate::{link_assert, CONFIG};

#[derive(Clone, Copy)]
#[derive_const(Default)]
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
    /// Use PWR for pull-up/pull-down rather than GPIOs.  This is required for
    /// the standby processing to work.
    pub low_power: bool,
    /// Enable the MSI FLL.
    pub fll: bool,
    /// Turn off debug...
    pub no_debug: bool,
}

#[used]
#[unsafe(link_section = ".vectors")]
pub static VECTORS: VectorTable = CONFIG.vectors;

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
    #[cfg(target_os = "none")]
    static end_of_ram: u8;
}

#[cfg(not(target_os = "none"))]
#[allow(non_upper_case_globals)]
static end_of_ram: u8 = 0;

pub const MSIRANGE: u8 = const {
    match CONFIG.clk {
        16000000 => 8,
        2000000 => 5,
        _ => panic!("CPU_CLK not implemented")
    }
};

/// Use the PWR configuration for PU/PD instead of the GPIO.  This enables
/// consistent PU/PD handling between runtime and standby.
fn pupd_via_pwr() {
    let pwr = unsafe {&*stm32u031::PWR::ptr()};
    #[inline]
    fn setup(shift: u32, pucr: &stm32u031::pwr::PUCRA, pdcr: &stm32u031::pwr::PDCRA) {
        let xtract = |x| (x >> shift & 0xffff) as u32;
        let pullup   = xtract(CONFIG.pullup);
        let keepup   = xtract(CONFIG.keep_pu | CONFIG.standby_pu);
        let pulldown = xtract(CONFIG.pulldown);
        let keepdown = xtract(CONFIG.keep_pd | CONFIG.standby_pd);

        // Always write all the PUCR, even if the flags don't indicate it is
        // needed.  This should keep us in a consistent state even over software
        // upgrades etc.
        let pp = if keepup == 0 {0} else {pucr.read().bits() & keepup};
        pucr.write(|w| w.bits(pullup | pp));

        let pp = if keepdown == 0 {0} else {pdcr.read().bits() & keepdown};
        pdcr.write(|w| w.bits(pulldown | pp));
    }
    setup( 0, &pwr.PUCRA, &pwr.PDCRA);
    setup(16, &pwr.PUCRB, &pwr.PDCRB);
    setup(32, &pwr.PUCRC, &pwr.PDCRC);

    // Special case, set the NRST pull-up.  Is this needed?
    pwr.PUCRF.write(|w| w.PU2().set_bit());

    // Apply the PU/PD config.
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
    let scb  = unsafe {&*cortex_m::peripheral::SCB::PTR};
    let nvic = unsafe {&*cortex_m::peripheral::NVIC::PTR};
    let pwr  = unsafe {&*stm32u031::PWR::ptr()};
    let rcc  = unsafe {&*stm32u031::RCC::ptr()};

    rcc.AHBENR .write(|w| w.bits(CONFIG.ahb_clocks));
    rcc.APBENR1.write(|w| w.bits(CONFIG.apb1_clocks));
    rcc.APBENR2.write(|w| w.bits(CONFIG.apb2_clocks));

    // Use MSI at appropriate frequency.
    rcc.CR.write(
        |w| w.MSIRANGE().bits(MSIRANGE).MSIRGSEL().set_bit().MSION().set_bit());

    // Enable backup domain access & maybe set lower-power run.
    let low_power = CONFIG.clk <= 2000000;
    pwr.CR1.write(|w| w.LPR().bit(low_power).DBP().set_bit());

    if CONFIG.low_power {
        pupd_via_pwr();
    }
    else {
        pupd_via_gpio();
    }

    // Enable the LSE.  Set high drive strength for crystal start-up.
    let bdcr = rcc.BDCR.read();
    if !bdcr.LSEON().bit() || !bdcr.LSERDY().bit() {
        rcc.BDCR.write(
            |w| w.bits(bdcr.bits()).LSEON().set_bit().LSEDRV().B_0x3());
    }

    // Clear the BSS.
    if !cfg!(test) {
        barrier();
        // The rustc memset is hideous.
        let mut p = (&raw mut __bss_start) as *mut u32;
        loop {
            unsafe {*p = 0};
            p = p.wrapping_add(1);
            if p as *mut u8 >= &raw mut __bss_end {
                break;
            }
        }
        barrier();
    }

    // We use sev-on-pend to avoid trivial interrupt handlers.
    unsafe {scb.scr.write(16)};

    if CONFIG.vectors.systick != bugger {
        // Set the systick interrupt priority to a high value (other
        // interrupts pre-empt).
        // The Cortex-M crate doesn't use the ARM indexes, so be careful about
        // the address.  We want SHPR3.
        link_assert!(core::ptr::from_ref(&scb.shpr[1]) as usize == 0xE000ED20);
        #[cfg(not(test))]
        unsafe {
            scb.shpr[1].write(0xc0 << 24);
        }
    }

    // Enable interrupts.
    unsafe {
        nvic.iser[0].write(CONFIG.interrupts);
    }
}

pub fn init2() {
    let rcc = unsafe {&*stm32u031::RCC::ptr()};

    // Enable the LSE interrupt, wait for it and then enable the MSI FLL.
    rcc.CIER.write(|w| w.LSERDYIE().set_bit());

    // Wait for LSE.  FIXME - this hangs if the interrupt is asserted for some
    // other reason.
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
}

impl Config {
    pub const fn new(clk: u32) -> Config {
        Config {
            clk, vectors: VectorTable::default(),
            // Always enable PWR clock.
            ahb_clocks: 0x100,
            apb1_clocks: 1 << 28,
            pullup: 1 << 13, pulldown: 1 << 14,
            fll: true, .. Config::default()
        }
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
    /// GPIO PUPD register value.
    pub const fn pupd(&self, port: u32) -> u32 {
        let up   = (self.pullup   >> port * 16) as u16;
        let down = (self.pulldown >> port * 16) as u16;
        crate::utils::spread16(up) + crate::utils::spread16(down) * 2
    }

    /// Clear any standby pull up/down in the config.
    #[allow(dead_code)]
    #[inline(always)]
    pub fn clear_pupd(&self) {
        // This is only relevant when the main program is using PWR for PUPD.
        if !crate::CONFIG.low_power {
            return;
        }
        let pwr = unsafe {&*stm32u031::PWR::ptr()};
        fn do1(me: &Config, shift: u32, pucr: &stm32u031::pwr::PUCRA,
               pdcr: &stm32u031::pwr::PDCRA) {
            let xtract = |b| (b >> shift & 0xffff) as u32;
            let clear_up   = xtract(me.standby_pu | me.keep_pu);
            let clear_down = xtract(me.standby_pd | me.keep_pd);
            if clear_up != 0 {
                pucr.modify(|r,w| w.bits(r.bits() & !clear_up));
            }
            if clear_down != 0 {
                pdcr.modify(|r,w| w.bits(r.bits() & !clear_down));
            }
        }
        do1(self,  0, &pwr.PUCRA, &pwr.PDCRA);
        do1(self, 16, &pwr.PUCRB, &pwr.PDCRB);
        do1(self, 32, &pwr.PUCRC, &pwr.PDCRC);
    }

}

#[derive(Clone, Copy)]
#[repr(C)]
pub struct VectorTable {
    pub stack     : *const u8,
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

/// !@#$!@$#
unsafe impl Sync for VectorTable {}

impl const Default for VectorTable {
    fn default() -> Self {
        VectorTable{
            stack     : &raw const end_of_ram,
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

unsafe extern "C" {
    #[link_name = "llvm.frameaddress"]
    fn frameaddress(level: i32) -> *const u8;
}

fn bugger() {
    let fp = unsafe {frameaddress(0)};
    // The exception PC is at +0x18, but then LLVM pushes an additional 8
    // bytes to form the frame.
    let pcp = fp.wrapping_add(0x20);
    let pc = unsafe {*(pcp as *const u32)};
    if crate::CONFIG.low_power {
        let tamp = unsafe {&*stm32u031::TAMP::ptr()};
        tamp.BKPR[8].write(|w| w.bits(pc));
    }
    else {
        crate::debug::banner("Crash @ 0x", pc, "\n");
        crate::debug::flush();
    }
    reboot();
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

pub fn reboot() -> ! {
    loop {
        unsafe {(*cortex_m::peripheral::SCB::PTR).aircr.write(0x05fa0004)};
    }
}

#[inline(always)]
pub fn barrier() {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
}

#[inline(always)]
#[allow(unused)]
pub fn nothing() {
    unsafe {core::arch::asm!("", options(nomem))}
}

pub mod interrupt {
    // We don't use disabling interrupts to transfer ownership, so no need for
    // the enable to be unsafe.
    #[cfg(target_arch = "arm")]
    #[allow(unused)]
    pub fn enable_all() {unsafe{cortex_m::interrupt::enable()}}
    #[cfg(target_arch = "arm")]
    #[allow(unused)]
    pub fn disable_all() {cortex_m::interrupt::disable()}
    #[cfg(not(target_arch = "arm"))]
    #[allow(unused)]
    pub fn enable_all() { }
    #[cfg(not(target_arch = "arm"))]
    #[allow(unused)]
    pub fn disable_all() { }
}

#[test]
fn default_pupd() {
    let config = Config::new(16000000);
    assert_eq!(config.pupd(0), 0x24000000);
    assert_eq!(config.pupd(1), 0);
    assert_eq!(config.pupd(2), 0);
}
