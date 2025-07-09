
unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

const MSIRANGE: u8 = const {
    match super::CPU_CLK {
        16000000 => 8,
        2000000 => 5,
        _ => panic!("CPU_CLK not implemented")
    }
};

pub fn init1() {
    let pwr = unsafe {&*stm32u031::PWR::ptr()};
    let rcc = unsafe {&*stm32u031::RCC::ptr()};

    // We can infer some of what clocks to enable from the interrupt mask
    // hidden in VECTORS.reserved1.  This is a bit inexact!
    rcc.AHBENR .write(|w| w.bits(const {crate::VECTORS.ahb_clocks()}));
    rcc.APBENR1.write(|w| w.bits(const {crate::VECTORS.apb1_clocks()}));
    rcc.APBENR2.write(|w| w.bits(const {crate::VECTORS.apb2_clocks()}));

    // Use MSI at appropriate frequency.
    rcc.CR.write(
        |w| w.MSIRANGE().bits(MSIRANGE).MSIRGSEL().set_bit().MSION().set_bit());

    if !cfg!(test) {
        let bss_start = &raw mut __bss_start;
        let bss_end   = &raw mut __bss_end;
        let bss_size = bss_end.addr() - bss_start.addr();
        unsafe {
            core::ptr::write_bytes(&raw mut __bss_start, 0u8, bss_size);
        }
    }

    // Enable backup domain access & maybe set lower-power run.
    let low_power = super::CPU_CLK <= 2000000;
    pwr.CR1.write(|w| w.LPR().bit(low_power).DBP().set_bit());

    // Enable the LSE.  Set high drive strength for crystal start-up.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x3());


}

pub fn init2() {
    let rcc = unsafe {&*stm32u031::RCC  ::ptr()};
    let scb = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    let nvic= unsafe {&*cortex_m::peripheral::NVIC::PTR};

    // Set the systick and pendsv interrupt priority to a high value (other
    // interrupts pre-empt).
    // The Cortex-M crate doesn't use the ARM indexes, so be careful about the
    // address.  We want SHPR3.
    assert_eq!(&scb.shpr[1] as *const _ as usize, 0xE000ED20);
    #[cfg(not(test))]
    unsafe {
        scb.shpr[1].write(0xc0 << 24);
    }

    // Wait for LSE and then enable the MSI FLL.
    while !rcc.BDCR.read().LSERDY().bit() {
    }
    rcc.CR.write(
        |w| w.MSIRANGE().bits(MSIRANGE).MSIRGSEL().set_bit().MSION().set_bit()
            . MSIPLLEN().set_bit());
    // Reduce drive strength.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x0());

    // Enable interrupts.  reserved1 has been abused to track what to enable.
    unsafe {
        nvic.iser[0].write(crate::VECTORS.reserved1[0]);
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

impl VectorTable {
    pub const fn new() -> VectorTable {
        VectorTable{
            stack     : 0x20000000 + 12 * 1024,
            reset     : super::main,
            nmi       : bugger,
            hard_fault: bugger,
            reserved1 : [0; 7],
            svcall    : bugger,
            reserved2 : [0; 2],
            pendsv    : bugger,
            systick   : bugger,
            isr       : [bugger; 32]}
    }
    #[allow(dead_code)]
    pub const fn systick(&mut self, handler: fn()) -> &mut Self {
        self.systick = handler;
        self
    }
    pub const fn isr(&mut self,
                     isr: stm32u031::Interrupt, handler: fn()) -> &mut Self {
        self.isr[isr as usize] = handler;
        // We abuse reserved1 to track what ISRs are used!
        self.reserved1[isr as usize / 32] |= 1 << isr as u32 % 32;
        self
    }
    pub const fn used(&self, isr: stm32u031::Interrupt) -> bool {
        self.reserved1[0] & (1 << isr as u32) != 0
    }
    pub const fn used_bit(&self, isr: stm32u031::Interrupt, b: u32) -> u32 {
        if self.used(isr) {1 << b} else {0}
    }
    pub const fn ahb_clocks(&self) -> u32 {
        // TSC -> 24
        use stm32u031::Interrupt::*;
        self.used_bit(TSC, 24) | self.used_bit(DMA1_CHANNEL1, 0)
        | self.used_bit(DMA1_CHANNEL2_3, 0)
        | self.used_bit(DMA1_CHANNEL4_5_6_7, 0)
    }
    pub const fn apb1_clocks(&self) -> u32 {
        use stm32u031::Interrupt::*;
        // Always PWR.
        1 << 28 | self.used_bit(USART3_LPUART1, 20) | self.used_bit(I2C1, 21)
    }
    pub const fn apb2_clocks(&self) -> u32 {
        use stm32u031::Interrupt::*;
        self.used_bit(ADC_COMP, 20) | self.used_bit(SPI1, 12)
    }
}

fn bugger() {
    panic!("Unexpected interrupt");
}

#[test]
fn check_vectors() {
    assert!(crate::VECTORS.reset == crate::main);
}
