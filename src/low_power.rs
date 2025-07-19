use crate::CONFIG;

pub fn rtc_enable() {
    let rcc = unsafe {&*stm32u031::RCC  ::ptr()};

    // APB clock to RTC, clock RTC with LSE.
    rcc.APBENR1.modify(|_, w| w.PWREN().set_bit().RTCAPBEN().set_bit());

    // RTC enabled and clocked with LSE.
    rcc.BDCR.modify(|_,w| w.RTCEN().set_bit().RTCSEL().B_0x1());
}

#[allow(non_snake_case)]
pub fn rtc_setup_20Hz() {
    let rtc = unsafe {&*stm32u031::RTC::ptr()};

    // To wake up from Stop mode with an RTC alarm event, it is necessary to:
    // •Configure the EXTI Line 18 to be sensitive to rising edge
    // •Configure the RTC to generate the RTC alarm
    // To wake up from Standby mode, there is no need to configure the EXTI Line 18.
    //
    // To wake up from Stop mode with an RTC wake-up event, it is necessary to:
    // •Configure the EXTI Line 20 to be sensitive to rising edge
    // •Configure the RTC to generate the RTC alarm
    //
    // .... bit that doesn't match the docs, its EXTI line 28.
    //exti.RTSR1.write(|w| w.RT28().set_bit());
    // None of these needed from standby or shutdown?

    rtc.WPR.write(|w| w.bits(0xca));
    rtc.WPR.write(|w| w.bits(0x53));

    rtc.CR.write(|w| w.WUTE().clear_bit());
    // This shouldn't take long and there appears to be nothing other than
    // spin that we can do.
    while !rtc.ICSR.read().WUTWF().bit() {
    }

    // Clock input to WUT is LSE / {2,4,8,16}.
    // 32768 / 20 = 1638.4 ≈ 16 * 102.
    rtc.WUTR.write(|w| w.WUT().bits(101));

    rtc.CR.write(|w| w.WUTE().set_bit().WUTIE().set_bit().WUCKSEL().B_0x0());

    // Reprotect.
    rtc.WPR.write(|w| w.bits(0));
    rtc.WPR.write(|w| w.bits(0));
}

/// Make sure reset is output only.
pub fn ensure_options() {
    let flash = unsafe {&*stm32u031::FLASH::ptr()};

    let options = flash.OPTR.read();
    if options.NRST_MODE().bits() == 1 {
        return;
    }
    crate::dbgln!("Options = {:#x} ... needs to be rewritten", options.bits());

    // 1.Write KEY1 = 0x4567 0123 in the FLASH key register (FLASH_KEYR)
    // 2.Write KEY2 = 0xCDEF 89AB in the FLASH key register (FLASH_KEYR).
    flash.KEYR.write(|w| w.bits(0x45670123));
    flash.KEYR.write(|w| w.bits(0xcdef89ab));
    // 1.Unlock the FLASH_CR with the LOCK clearing sequence (refer to
    // Unlocking the flash memory ).
    // 2.Write OPTKEY1 = 0x0819 2A3B of the FLASH option key register
    //  (FLASH_OPTKEYR).
    // 3.Write OPTKEY2 = 0x4C5D 6E7F of the FLASH option key register
    // (FLASH_OPTKEYR)
    flash.OPTKEYR.write(|w| w.bits(0x08192a3b));
    flash.OPTKEYR.write(|w| w.bits(0x4c5d6e7f));

    // Write the options...
    flash.OPTR.modify(|_,w| w.NRST_MODE().B_0x1());

    while flash.SR.read().BSY1().bit() {}
    flash.CR.write(|w| w.OPTSTRT().set_bit());
    while flash.SR.read().BSY1().bit() {}
    crate::dbgln!("Options written!");
}

pub fn standby() -> ! {
    let pwr   = unsafe {&*stm32u031::PWR::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC::ptr()};
    let rtc   = unsafe {&*stm32u031::RTC::ptr()};
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let gpiob = unsafe {&*stm32u031::GPIOB::ptr()};
    let gpioc = unsafe {&*stm32u031::GPIOC::ptr()};
    let scb   = unsafe {&*cortex_m::peripheral::SCB ::PTR};

    // Low power mode to use is standby...
    pwr.CR1.modify(|_,w| w.LPMS().bits(3)); // 3=Standby, 4=Shutdown.

    // Clear the reset causes & set restart MSI.
    rcc.CSR.write(|w| w.RMVF().set_bit().MSISRANGE().bits(crate::cpu::MSIRANGE));

    fn pupd(shift: u32, gpio: &stm32u031::gpioa::RegisterBlock,
            pucr: &stm32u031::pwr::PUCRA, pdcr: &stm32u031::pwr::PDCRA) {
        let standby_pu = (CONFIG.standby_pu >> shift) as u32 & 0xffff;
        let standby_pd = (CONFIG.standby_pd >> shift) as u32 & 0xffff;
        let keep_up    = (CONFIG.keep_pu    >> shift) as u32 & 0xffff;
        let keep_down  = (CONFIG.keep_pd    >> shift) as u32 & 0xffff;

        let keep = keep_up | keep_down != 0;
        let bits = if keep {gpio.IDR().read().bits()} else {0};
        if standby_pu | keep_up != 0 {
            pucr.write(|w| w.bits(bits & keep_up | standby_pu));
        }
        if standby_pd | keep_down != 0 {
            pdcr.write(|w| w.bits(!bits & keep_down | standby_pd));
        }
    }

    pupd( 0, gpioa, &pwr.PUCRA, &pwr.PDCRA);
    pupd(16, gpiob, &pwr.PUCRB, &pwr.PDCRB);
    pupd(32, gpioc, &pwr.PUCRC, &pwr.PDCRC);

    if !crate::CONFIG.pupd_use_pwr {
        // Enable the pullups.
        pwr.CR3.modify(|_, w| w.APC().set_bit());
    }

    crate::debug::flush();

    // Clear the RTC wake-up flags.  This seems to need to be some time after
    // the wake-up!  Presumably a clock-tick of the RTC domain.
    // FIXME - do this individually!
    rtc.SCR.write(|w| w.bits(!0));

    // Deep sleep.  Note that if an interrupt does WFE after this we are liable
    // to go into standby from the ISR!
    unsafe {scb.scr.write(4)};

    // It looks like, if we do have wake-up bits set, then we reset immediately,
    // instead of ignoring the WFE completely.  The event flag is probably set
    // when we get here.
    loop {
        #[cfg(target_os = "none")]
        cortex_m::asm::wfi();
        unsafe {(*cortex_m::peripheral::SCB::PTR).aircr.write(0x05fa0004)};
    }
}
