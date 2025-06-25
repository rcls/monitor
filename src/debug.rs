
use crate::vcell::{UCell, VCell};

pub struct DebugS;
pub struct DebugAMarker;

static DEBUG: UCell<DebugA> = UCell::new(DebugA::new());

pub struct DebugA {
    w: VCell<u8>,
    r: VCell<u8>,
    buf: [UCell<u8>; 256],
}

pub fn debug_isr() {
    unsafe {DEBUG.as_mut()}.isr();
}

#[inline(always)]
fn barrier() {
    unsafe {core::arch::asm!("")}
}

impl DebugA {
    const fn new() -> DebugA {
        DebugA {
            w: VCell::new(0), r: VCell::new(0),
            buf: [const {UCell::new(0)}; 256]
        }
    }
    fn write_str(&self, s: &str) {
        let mut w = self.w.read();
        for &b in s.as_bytes() {
            while self.r.read().wrapping_sub(w) == 1 {
                self.enable(w);
                cortex_m::asm::wfe();
            }
            // The ISR won't access the array element in question.
            unsafe {*self.buf[w as usize].as_mut() = b};
            w = w.wrapping_add(1);
        }
        self.enable(w);
    }
    fn enable(&self, w: u8) {
        barrier();
        self.w.write(w);
        let usart  = unsafe {&*stm32u031::USART2::ptr()};
        // Use the FIFO empty interrupt.  Normally we should be fast enough
        // to refill before the last byte finishes.
        usart.CR1.write(
            |w| w.FIFOEN().set_bit().TE().set_bit().UE().set_bit()
                . TXFEIE().set_bit());
    }
    fn isr(&mut self) {
        // Assume no spurious wake-ups!
        let usart  = unsafe {&*stm32u031::USART2::ptr()};
        const FIFO_SIZE: usize = 8;
        let mut r = *self.r.as_mut() as usize;
        let w = *self.w.as_mut() as usize;
        let mut done = 0;
        while r != w && done < FIFO_SIZE {
            usart.TDR.write(
                |w| unsafe{w.bits(*self.buf[r].as_ref() as u32)});
            r = (r + 1) & 0xff;
            done += 1;
        }
        if r == w {
            usart.CR1.write(
                |w| w.FIFOEN().set_bit().TE().set_bit().UE().set_bit());
        }
        *self.r.as_mut() = r as u8;
        usart.ICR.write(|w| w.TXFECF().set_bit());
    }
}

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

#[inline(never)]
fn debug_char(s: char) {
    let uart = unsafe {&*stm32u031::USART2::ptr()};
    // This is manky and doesn't cope with UTF-8.
    while !uart.ISR.read().TXFNF().bit() {
    }
    uart.TDR.write(|w| unsafe { w.bits(s as u32) });
}

#[inline(never)]
fn async_debug_str(s: &str) {
    let debug = DEBUG.as_ref();
    debug.write_str(s);
}

#[inline(never)]
fn async_debug_char(c: char) {
    // In practice we only get called with ASCII. And async_debug_str doesn't
    // complain if the character is not valid.
    let cc = [c as u8];
    async_debug_str(unsafe {str::from_utf8_unchecked(&cc)});
}

impl core::fmt::Write for DebugS {
    #[inline(always)]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        debug_str(s);
        Ok(())
    }
    #[inline(always)]
    fn write_char(&mut self, c: char) -> core::fmt::Result {
        debug_char(c);
        Ok(())
    }
}

impl core::fmt::Write for DebugAMarker {
    #[inline(always)]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        async_debug_str(s);
        Ok(())
    }
    #[inline(always)]
    fn write_char(&mut self, c: char) -> core::fmt::Result {
        async_debug_char(c);
        Ok(())
    }
}

#[macro_export]
macro_rules! sdbg {
    ($($tt:tt)*) => (
        let _ = core::fmt::Write::write_fmt(
            &mut $crate::debug::DebugS, format_args!($($tt)*)););
}

#[macro_export]
macro_rules! dbgln {
    () => ({let _ = core::fmt::Write::write_str(
        &mut $crate::debug::DebugAMarker, "\n");});
    ($($tt:tt)*) => ({let _ = core::fmt::Write::write_fmt(
        &mut $crate::debug::DebugAMarker, format_args_nl!($($tt)*));});
}

#[macro_export]
macro_rules! sdbgln {
    () => ({let _ = core::fmt::Write::write_str(
        &mut $crate::debug::DebugS, "\n");});
    ($($tt:tt)*) => ({let _ = core::fmt::Write::write_fmt(
        &mut $crate::debug::DebugS, format_args_nl!($($tt)*));});
}

pub fn init() {
    // Set-up the UART TX.  TODO - we should enable RX at some point.  The dbg*
    // macros will work after this.
    let usart  = unsafe {&*stm32u031::USART2::ptr()};
    usart.BRR.write(|w| unsafe {w.BRR().bits(139)});
    usart.CR1.write(
        |w| w.FIFOEN().set_bit().TE().set_bit().UE().set_bit());

    if false {
        debug_isr();
        // dbg!("Hello world!");
        dbgln!();
        dbgln!("Hello world!");
        // sdbg!("Hello world!");
        sdbgln!();
        sdbgln!("Hello world!")
    }
}

#[cfg(not(test))]
#[panic_handler]
fn ph(info: &core::panic::PanicInfo) -> ! {
    sdbg!("{info}");
    // Let the TX FIFO drain.
    let usart = unsafe { &*stm32u031::USART2::ptr() };
    while !usart.ISR().read().TXFE().bit() {
    }
    loop {
        unsafe {(*cortex_m::peripheral::SCB::PTR).aircr.write(0x05fa0004)};
    }
}
