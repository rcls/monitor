
#[derive(Clone, Copy)]
pub struct CpuConfig {
    pub clk: u32,
    pub vectors: VectorTable,
    pub ahb_clocks: u32,
    pub apb1_clocks: u32,
    pub apb2_clocks: u32,
    pub interrupts: u32,
    //pub pull_up: u64,
    //pub pull_down: u64,
    //pub standby_pull_up: u64,
    //pub standby_preserve: u64,
}

#[used]
#[unsafe(link_section = ".vectors")]
pub static VECTORS: VectorTable = crate::CONFIG.vectors;

unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

const MSIRANGE: u8 = const {
    match crate::CONFIG.clk {
        16000000 => 8,
        2000000 => 5,
        _ => panic!("CPU_CLK not implemented")
    }
};

pub fn init1() {
    let scb = unsafe {&*cortex_m::peripheral::SCB::PTR};
    let pwr = unsafe {&*stm32u031::PWR::ptr()};
    let rcc = unsafe {&*stm32u031::RCC::ptr()};

    // We can infer some of what clocks to enable from the interrupt mask
    // hidden in VECTORS.reserved1.  This is a bit inexact!
    rcc.AHBENR .write(|w| w.bits(crate::CONFIG.ahb_clocks));
    rcc.APBENR1.write(|w| w.bits(crate::CONFIG.apb1_clocks));
    rcc.APBENR2.write(|w| w.bits(crate::CONFIG.apb2_clocks));

    // Use MSI at appropriate frequency.
    rcc.CR.write(
        |w| w.MSIRANGE().bits(MSIRANGE).MSIRGSEL().set_bit().MSION().set_bit());

    // Enable backup domain access & maybe set lower-power run.
    let low_power = crate::CONFIG.clk <= 2000000;
    pwr.CR1.write(|w| w.LPR().bit(low_power).DBP().set_bit());

    // Clear the BSS.
    if !cfg!(test) {
        // The rustc memset is hideous.
        let mut p = (&raw mut __bss_start) as *mut u32;
        loop {
            unsafe {*p = 0};
            p = p.wrapping_add(1);
            if p as *mut u8 >= &raw mut __bss_end {
                break;
            }
        }
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

    // Set the systick and pendsv interrupt priority to a high value (other
    // interrupts pre-empt).
    // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
    // address.  We want SHPR3.
    assert_eq!(core::ptr::from_ref(&scb.shpr[1]) as usize, 0xE000ED20);
    #[cfg(not(test))]
    unsafe {
        scb.shpr[1].write(0xc0 << 24);
    }

    // Enable the LSE interrupt, wait for it and then enable the MSI FLL.
    rcc.CIER.write(|w| w.LSERDYIE().set_bit());
    //unsafe {nvic.iser[0].write(1u32 << stm32u031::Interrupt::RCC_CRS as u32)};

    // Wait for LSE and then enable the MSI FLL.
    while !rcc.BDCR.read().LSERDY().bit() {
        WFE();
    }
    rcc.CIER.write(|w| w.LSIRDYIE().clear_bit());
    rcc.CICR.write(|w| w.bits(!0));

    rcc.CR.write(
        |w| w.MSIRANGE().bits(MSIRANGE).MSIRGSEL().set_bit().MSION().set_bit()
            . MSIPLLEN().set_bit());

    // Reduce drive strength.  (Lowest drive strength appears unreliable).
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x1());

    // Enable interrupts.  reserved1 has been abused to track what to enable.
    unsafe {
        nvic.iser[0].write(crate::CONFIG.interrupts);
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

impl CpuConfig {
    pub const fn new(clk: u32) -> CpuConfig {
        CpuConfig{
            clk, vectors: VectorTable::new(),
            ahb_clocks: 0, apb1_clocks: 1 << 28, apb2_clocks: 0,
            interrupts: 0}
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
}

impl VectorTable {
    pub const fn new() -> VectorTable {
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
