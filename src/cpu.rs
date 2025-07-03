
unsafe extern "C" {
    static mut __bss_start: u8;
    static mut __bss_end: u8;
}

#[inline(always)]
pub fn init1() {
    //let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let pwr   = unsafe {&*stm32u031::PWR  ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    //let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    //let nvic  = unsafe {&*cortex_m::peripheral::NVIC::PTR};

    // Go to 16MHz.
    rcc.CR.write(
        |w| w.MSIRANGE().B_0x8().MSIRGSEL().set_bit().MSION().set_bit());

    if !cfg!(test) {
        let bss_start = &raw mut __bss_start;
        let bss_end   = &raw mut __bss_end;
        let bss_size = bss_end.addr() - bss_start.addr();
        unsafe {
            core::ptr::write_bytes(&raw mut __bss_start, 0u8, bss_size);
        }
    }

    // Enable backup domain access.
    pwr.CR1.write(|w| w.DBP().set_bit());

    // Enable the LSE.  Set high drive strength for crystal start-up.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x3());
}

pub fn init2() {
    //let pwr   = unsafe {&*stm32u031::PWR  ::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC  ::ptr()};
    let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};
    //let nvic  = unsafe {&*cortex_m::peripheral::NVIC::PTR};

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
        |w| w.MSIRANGE().B_0x8().MSIRGSEL().set_bit().MSION().set_bit()
            . MSIPLLEN().set_bit());
    // Reduce drive strength.
    rcc.BDCR.write(|w| w.LSEON().set_bit().LSEDRV().B_0x0());
}