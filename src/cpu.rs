
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

#[inline(always)]
pub fn init1() {
    //let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let pwr   = unsafe {&*stm32u031::PWR  ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    //let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    //let nvic  = unsafe {&*cortex_m::peripheral::NVIC::PTR};

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
                     number: stm32u031::Interrupt, handler: fn()) -> &mut Self {
        self.isr[number as usize] = handler;
        self
    }
}

fn bugger() {
    panic!("Unexpected interrupt");
}

#[test]
fn check_vectors() {
    assert!(crate::VECTORS.reset == crate::main);
}
