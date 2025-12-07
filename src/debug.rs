
use stm_common::{debug, link_assert};
use debug::Debug;

use stm32u031::Interrupt::USART3_LPUART1 as INTERRUPT;
use stm32u031::LPUART1 as UART;

#[derive_const(Default)]
pub struct DebugMeta;

impl debug::Meta for DebugMeta {
    const ENABLE: bool = !crate::CONFIG.no_debug;

    fn debug() -> &'static Debug<Self> {&DEBUG}

    fn uart(&self) -> &'static debug::UART {unsafe {&*stm32u031::LPUART1::PTR}}

    fn lazy_init(&self) {
        if crate::CONFIG.is_lazy_debug() {
            init();
        }
    }

    fn is_init(&self) -> bool {
        let rcc = unsafe {&*stm32u031::RCC::ptr()};
        ENABLE && rcc.APBENR1.read().LPUART1EN().bit()
    }

    fn interrupt(&self) -> u32 {INTERRUPT as u32}
}

/// State for debug logging.  We mark this as no-init and initialize the cells
/// ourselves, to avoid putting the buffer into BSS.
#[unsafe(link_section = ".noinit")]
pub static DEBUG: Debug<DebugMeta> = Debug::default();

pub const ENABLE: bool = !crate::CONFIG.no_debug;

pub fn flush() {
    debug::flush::<DebugMeta>();
}

pub fn write_str(s: &str) {
    debug::write_str::<DebugMeta>(s);
}

pub fn debug_isr() {
    DEBUG.isr();
}

pub fn init() {
    let gpioa = unsafe {&*stm32u031::GPIOA::ptr()};
    let rcc   = unsafe {&*stm32u031::RCC::ptr()};
    let uart  = unsafe {&*UART::ptr()};

    if !ENABLE {
        return;
    }

    check_vtors();
    // Lazy initialization.
    let apbenr1 = rcc.APBENR1.read();
    if crate::CONFIG.is_lazy_debug() && apbenr1.LPUART1EN().bit() {
        return;
    }
    rcc.APBENR1.write(|w| w.bits(apbenr1.bits()).LPUART1EN().set_bit());

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
}

#[cfg(target_os = "none")]
#[panic_handler]
fn ph(info: &core::panic::PanicInfo) -> ! {
    stm_common::dbgln!("{info}");
    stm_common::debug::flush::<DebugMeta>();
    stm_common::utils::reboot();
}

/// The code gen for formats is so awful that we do this!
pub fn banner(s: &str, mut v: u32, t: &str) {
    if crate::CONFIG.no_debug {
        return;
    }
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
    // Use the highest prescaler that passes the check.
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
        self.isr(INTERRUPT, debug_isr)
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
    if !crate::CONFIG.no_debug {
        link_assert!(crate::cpu::VECTORS.isr[INTERRUPT as usize] == debug_isr);
    }
}
