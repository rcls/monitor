
use crate::cpu::WFE;
use crate::vcell::{UCell, VCell, barrier};

use core::fmt::Write;
use static_assertions::const_assert;

pub use stm32u031::LPUART1 as UART;
pub struct DebugS;
pub struct DebugAMarker;

#[allow(unused_imports)]
pub use stm32u031::Interrupt::USART3_LPUART1 as UART_ISR;

pub static DEBUG: UCell<DebugA> = UCell::new(DebugA::new());

pub struct DebugA {
    w: VCell<u8>,
    r: VCell<u8>,
    buf: [UCell<u8>; 256],
}

pub fn debug_isr() {
    unsafe {DEBUG.as_mut()}.isr();
}

impl DebugA {
    const fn new() -> DebugA {
        DebugA {
            w: VCell::new(0), r: VCell::new(0),
            buf: [const {UCell::new(0)}; 256]
        }
    }
    fn write_bytes(&self, s: &[u8]) -> core::fmt::Result {
        let mut w = self.w.read();
        for &b in s {
            while self.r.read().wrapping_sub(w) == 1 {
                self.enable(w);
                WFE();
            }
            // The ISR won't access the array element in question.
            unsafe {*self.buf[w as usize].as_mut() = b};
            w = w.wrapping_add(1);
        }
        self.enable(w);
        Ok(())
    }
    fn enable(&self, w: u8) {
        barrier();
        self.w.write(w);
        let uart  = unsafe {&*UART::ptr()};
        // Use the FIFO empty interrupt.  Normally we should be fast enough
        // to refill before the last byte finishes.
        uart.CR1.write(
            |w| w.FIFOEN().set_bit().TE().set_bit().UE().set_bit()
                . TXFEIE().set_bit());
    }
    fn isr(&mut self) {
        // Assume no spurious wake-ups!
        let uart  = unsafe {&*UART::ptr()};
        let sr = uart.ISR.read();
        if sr.TC().bit() {
            uart.CR1.modify(|_,w| w.TCIE().clear_bit());
        }
        if !sr.TXFE().bit() {
            return;
        }

        const FIFO_SIZE: usize = 8;
        let mut r = *self.r.as_mut() as usize;
        let w = *self.w.as_mut() as usize;
        let mut done = 0;
        while r != w && done < FIFO_SIZE {
            uart.TDR.write(|w| w.bits(*self.buf[r].as_ref() as u32));
            r = (r + 1) & 0xff;
            done += 1;
        }
        *self.r.as_mut() = r as u8;
        if r == w {
            uart.CR1.modify(|_,w| w.TXFEIE().clear_bit());
        }
    }
}

#[allow(dead_code)]
pub fn drain() {
    let uart  = unsafe {&*UART::ptr()};
    // Enable the TC interrupt.
    uart.CR1().modify(|_,w| w.TCIE().set_bit());
    // Wait for the TC bit.
    while !uart.ISR().read().TC().bit() {
        WFE();
    }
}

fn sdebug_bytes(s: &[u8]) -> core::fmt::Result {
    let uart = unsafe {&*UART::ptr()};
    for &b in s {
        // This is a bit manky...
        while !uart.ISR.read().TXFNF().bit() {
        }
        uart.TDR.write(|w| w.bits(b as u32));
    }
    Ok(())
}

impl Write for DebugS {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        sdebug_bytes(s.as_bytes())
    }
    fn write_char(&mut self, c: char) -> core::fmt::Result {
        let cc = [c as u8];
        sdebug_bytes(&cc)
    }
}

impl Write for DebugAMarker {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        DEBUG.write_bytes(s.as_bytes())
    }
    fn write_char(&mut self, c: char) -> core::fmt::Result {
        let cc = [c as u8];
        DEBUG.write_bytes(&cc)
    }
}

#[macro_export]
macro_rules! dbg {
    ($($tt:tt)*) => (
        let _ = core::fmt::Write::write_fmt(
            &mut $crate::debug::DebugAMarker, format_args!($($tt)*)););
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
    let gpioa = unsafe {&*stm32u031::GPIOA ::ptr()};
    let uart  = unsafe {&*UART::ptr()};

    // Configure UART lines.  sdbg! should now work.
    gpioa.AFRL.modify(|_, w| w.AFSEL2().B_0x8().AFSEL3().B_0x8());
    gpioa.MODER.modify(|_, w| w.MODE2().B_0x2().MODE3().B_0x2());

    // Pullups on the UART RX pin.
    gpioa.PUPDR.modify(|_, w| w.PUPD3().B_0x1());

    // Set-up the UART TX.  TODO - we should enable RX at some point.  The dbg*
    // macros will work after this.

    //const BRR: u32 = (super::CPU_CLK * 2 / 115200 + 1) / 2;
    //assert!(BRR >= 16 && BRR < 65536);
    const PB: (u32, u32) = prescale(115200);
    const PRESC: u32 = PB.0;
    const BRR: u32 = PB.1;
    const_assert!(BRR >= 0x300);
    const_assert!(BRR < 1 << 20);
    uart.BRR.write(|w| w.bits(BRR));
    uart.PRESC.write(|w| w.bits(PRESC));
    uart.CR1.write(
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
    sdbgln!("{info}");
    // Let the TX FIFO drain.
    let uart = unsafe { &*UART::ptr() };
    while !uart.ISR().read().TXFE().bit() {
        let isr = uart.ISR().read();
        if isr.TXFE().bit() && isr.TC().bit() {
            break;
        }
    }
    loop {
        unsafe {(*cortex_m::peripheral::SCB::PTR).aircr.write(0x05fa0004)};
    }
}

// Returns prescaler (prescaler index, BRR value).
const fn prescale(bit_rate: u32) -> (u32, u32) {
    const fn calc(bit_rate: u32, presc: usize) -> u32 {
        let divs = [1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256];
        let div = divs[presc];
        ((512 * super::CPU_CLK as u64 / bit_rate as u64) as u32 + div)
            / 2 / div
    }
    const fn ok(bit_rate: u32, presc: usize) -> bool {
        let brr = calc(bit_rate, presc);
        brr >= 0x300 && brr < 1<<20
    }
    let mut presc = 11;
    while !ok(bit_rate, presc) {
        presc -= 1;
    }
    (presc as u32, calc(bit_rate, presc))
}

impl crate::cpu::VectorTable {
    pub const fn debug_isr(&mut self) -> &mut Self {
        self.isr(UART_ISR, debug_isr)
    }
}

#[test]
fn check_vtors() {
    use super::VECTORS;

    assert!(std::ptr::fn_addr_eq(VECTORS.isr[UART_ISR as usize],
                                 debug_isr as fn()));
    assert!(VECTORS.apb1_clocks() & (1 << 20) != 0, "{:#x} {:#x}",
            VECTORS.reserved1[0], VECTORS.apb1_clocks());
}
