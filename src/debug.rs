
use crate::cpu::WFE;
use crate::vcell::{UCell, VCell, barrier};

use core::fmt::Write;

pub use stm32u031::LPUART1 as UART;

pub struct DebugMarker;

pub use stm32u031::Interrupt::USART3_LPUART1 as UART_ISR;

/// State for debug logging.  We mark this as no-init and initialize the cells
/// ourselves, to avoid putting the buffer into BSS.
#[unsafe(link_section = ".noinit")]
pub static DEBUG: Debug = Debug::new();

pub struct Debug {
    w: VCell<u8>,
    r: VCell<u8>,
    buf: [UCell<u8>; 256],
}

pub fn debug_isr() {
    if !crate::CONFIG.no_debug {
        DEBUG.isr();
    }
}

impl Debug {
    const fn new() -> Debug {
        Debug {
            w: VCell::new(0), r: VCell::new(0),
            buf: [const {UCell::new(0)}; 256]
        }
    }
    fn write_bytes(&self, s: &[u8]) {
        check_vtors();
        lazy_init();
        let mut w = self.w.read();
        for &b in s {
            while self.r.read().wrapping_sub(w) == 1 {
                self.enable(w);
                self.push();
            }
            // The ISR won't access the array element in question.
            unsafe {*self.buf[w as usize].as_mut() = b};
            w = w.wrapping_add(1);
        }
        self.enable(w);
    }
    fn push(&self) {
        WFE();
        // If the interrupt is pending, call the ISR ourselves.  Read the bit
        // twice in case there is a race condition where we read pending on an
        // enabled interrupt.
        let nvic = unsafe {&*cortex_m::peripheral::NVIC::PTR};
        let bit: u32 = 1 << stm32u031::Interrupt::USART3_LPUART1 as u32;
        if nvic.icpr[0].read() & bit == 0 {
            return;
        }
        // It might take a couple of goes for the pending state to clear, so
        // loop.
        while nvic.icpr[0].read() & bit != 0 {
            unsafe {nvic.icpr[0].write(bit)};
            debug_isr();
        }
    }
    fn enable(&self, w: u8) {
        barrier();
        self.w.write(w);

        let uart = unsafe {&*UART::ptr()};
        // Use the FIFO empty interrupt.  Normally we should be fast enough
        // to refill before the last byte finishes.
        uart.CR1.write(
            |w| w.FIFOEN().set_bit().TE().set_bit().UE().set_bit()
                . TXFEIE().set_bit());
    }
    fn isr(&self) {
        let uart = unsafe {&*UART::ptr()};
        let sr = uart.ISR.read();
        if sr.TC().bit() {
            uart.CR1.modify(|_,w| w.TCIE().clear_bit());
        }
        if !sr.TXFE().bit() {
            return;
        }

        const FIFO_SIZE: usize = 8;
        let mut r = self.r.read() as usize;
        let w = self.w.read() as usize;
        let mut done = 0;
        while r != w && done < FIFO_SIZE {
            uart.TDR.write(|w| w.bits(*self.buf[r].as_ref() as u32));
            r = (r + 1) & 0xff;
            done += 1;
        }
        self.r.write(r as u8);
        if r == w {
            uart.CR1.modify(|_,w| w.TXFEIE().clear_bit());
        }
    }
}

pub fn flush() {
    check_vtors();
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    if crate::CONFIG.no_debug ||
        crate::CONFIG.is_lazy_debug() && !rcc.APBENR1.read().LPUART1EN().bit() {
        return;                        // Not initialized, nothing to do.
    }

    let uart  = unsafe {&*UART::ptr()};
    // Enable the TC interrupt.
    uart.CR1().modify(|_,w| w.TCIE().set_bit());
    // Wait for the TC bit.
    loop {
        let isr = uart.ISR().read();
        if DEBUG.r.read() == DEBUG.w.read()
            && isr.TC().bit() && isr.TXFE().bit() {
            break;
        }
        DEBUG.push();
    }
}

/// The code gen for formats is so awful that we do this!
pub fn banner(s: &str, mut v: u32, t: &str) {
    write_str(s);
    let mut hex = [0; 8];
    for p in hex.iter_mut().rev() {
        let d = v as u8 & 15;
        v >>= 4;
        *p = d + b'0';
        if *p > b'9' {
            *p += b'a' - 10 - b'0';
        };
    }
    DEBUG.write_bytes(&hex);
    write_str(t);
}

pub fn write_str(s: &str) {
    DEBUG.write_bytes(s.as_bytes());
}

impl Write for DebugMarker {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        write_str(s);
        Ok(())
    }
    fn write_char(&mut self, c: char) -> core::fmt::Result {
        let cc = [c as u8];
        DEBUG.write_bytes(&cc);
        Ok(())
    }
}

#[macro_export]
macro_rules! dbg {
    ($($tt:tt)*) => {
        if !$crate::CONFIG.no_debug {
            let _ = core::fmt::Write::write_fmt(
                &mut $crate::debug::DebugMarker, format_args!($($tt)*));
        }
    }
}

#[macro_export]
macro_rules! dbgln {
    () => {if !$crate::CONFIG.no_debug {
        let _ = core::fmt::Write::write_str(
            &mut $crate::debug::DebugMarker, "\n");
        }};
    ($($tt:tt)*) => {if !$crate::CONFIG.no_debug {
        let _ = core::fmt::Write::write_fmt(
            &mut $crate::debug::DebugMarker, format_args_nl!($($tt)*));
        }};
}

pub fn lazy_init() {
    // Check for lazy enablement...
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    if crate::CONFIG.is_lazy_debug()
        && !rcc.APBENR1.read().LPUART1EN().bit() {
        init();
    }
}

pub fn init() {
    check_vtors();
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC::ptr()};
    let uart  = unsafe {&*UART::ptr()};

    if crate::CONFIG.is_lazy_debug() {
        // Lazy initialization.
        rcc.APBENR1.modify(|_, w| w.LPUART1EN().set_bit());
    }
    DEBUG.w.write(0);
    DEBUG.r.write(0);

    // Configure UART lines.  sdbg! should now work.
    gpioa.AFRL.modify(|_, w| w.AFSEL2().B_0x8().AFSEL3().B_0x8());
    gpioa.MODER.modify(|_, w| w.MODE2().B_0x2().MODE3().B_0x2());

    // Set-up the UART TX.  TODO - we should enable RX at some point.  The dbg*
    // macros will work after this.

    //const BRR: u32 = (super::CPU_CLK * 2 / 115200 + 1) / 2;
    //assert!(BRR >= 16 && BRR < 65536);
    const PB: (u32, u32) = prescale(115200);
    const PRESC: u32 = PB.0;
    const BRR: u32 = PB.1;
    const {assert!(BRR >= 0x300)}
    const {assert!(BRR < 1 << 20)}
    uart.BRR.write(|w| w.bits(BRR));
    uart.PRESC.write(|w| w.bits(PRESC));
    uart.CR1.write(
        |w| w.FIFOEN().set_bit().TE().set_bit().UE().set_bit());

    if false {
        dbg!("");
        dbgln!("");
    }
}

#[cfg(target_os = "none")]
#[panic_handler]
fn ph(info: &core::panic::PanicInfo) -> ! {
    dbgln!("{info}");
    flush();
    crate::cpu::reboot();
}

/// Returns prescaler (prescaler index, BRR value).
const fn prescale(bit_rate: u32) -> (u32, u32) {
    const fn calc(bit_rate: u32, presc: usize) -> u32 {
        let divs = [1, 2, 4, 6, 8, 10, 12, 16, 32, 64, 128, 256];
        let div = divs[presc];
        ((512 * crate::CONFIG.clk as u64 / bit_rate as u64) as u32 + div)
            / 2 / div
    }
    const fn ok(bit_rate: u32, presc: usize) -> bool {
        let brr = calc(bit_rate, presc);
        brr >= 0x300 && brr < 1<<20 && (brr >= 0x800 || presc == 0)
    }
    let mut presc = 11;
    while !ok(bit_rate, presc) {
        presc -= 1;
    }
    (presc as u32, calc(bit_rate, presc))
}

#[allow(dead_code)]
impl crate::cpu::Config {
    pub const fn debug(&mut self) -> &mut Self {
        self.lazy_debug().clocks(0, 1 << 20, 0)
    }
    pub const fn lazy_debug(&mut self) -> &mut Self {
        // self.pullup |= 1 << 3; // Pull-up on RX pin.
        self.isr(UART_ISR, debug_isr)
    }
    pub const fn no_debug(&mut self) -> &mut Self {
        self.no_debug = true;
        self
    }
    pub const fn is_lazy_debug(&self) -> bool {
        self.apb1_clocks & 1 << 20 == 0
    }
}

#[inline(always)]
#[cfg_attr(test, test)]
fn check_vtors() {
    use crate::link_assert;
    if !crate::CONFIG.no_debug {
        link_assert!(crate::cpu::VECTORS.isr[UART_ISR as usize] == debug_isr);
    }
    else {
        link_assert!(crate::cpu::VECTORS.isr[UART_ISR as usize] != debug_isr);
    }
}
