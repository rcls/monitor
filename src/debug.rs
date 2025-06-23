#![allow(unused_macros)]

pub struct Debug;

#[inline(never)]
fn debug_str(s: &str) {
    let uart = unsafe {&*stm32u031::USART2::ptr()};
    for &b in s.as_bytes() {
        // This is a bit manky...
        while !uart.ISR.read().TXFNF().bit() {
        }
        uart.TDR.write(|w| unsafe { w.bits(b as u32) });
    }
}

impl core::fmt::Write for Debug {
    #[inline(always)]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        debug_str(s);
        Ok(())
    }
}

#[macro_export]
macro_rules! dbg {
    ($($tt:tt)*) => ({
        let _ = core::fmt::Write::write_fmt(&mut $crate::debug::Debug,
                                            format_args!($($tt)*));});
}

#[macro_export]
macro_rules! dbgln {
    () => ({
        let _ = core::fmt::Write::write_str(&mut $crate::debug::Debug, "\n")});
    ($($tt:tt)*) => ({
        let _ = core::fmt::Write::write_fmt(&mut $crate::debug::Debug,
                                            format_args_nl!($($tt)*));});
}

#[cfg(not(test))]
#[panic_handler]
fn ph(info: &core::panic::PanicInfo) -> ! {
    dbg!("{info}");
    // Let the TX FIFO drain.
    let usart = unsafe { &*stm32u031::USART2::ptr() };
    while !usart.ISR().read().TXFE().bit() {
    }
    loop {
        unsafe {(*cortex_m::peripheral::SCB::PTR).aircr.write(0x05fa0004)};
    }
}
