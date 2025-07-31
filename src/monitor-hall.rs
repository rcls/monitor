#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![allow(internal_features)]
#![allow(unpredictable_function_pointer_comparisons)]
#![feature(const_default)]
#![feature(const_trait_impl)]
#![feature(derive_const)]
#![feature(format_args_nl)]
#![feature(link_llvm_intrinsics)]

mod adc;
mod cpu;
mod decimal;
mod dma;
mod debug;
mod font;
mod i2c;
mod monitor;
mod noise;
mod oled;
mod usqrt;
mod utils;
mod vcell;

use monitor::main;

const I2C_LINES: i2c::I2CLines = i2c::I2CLines::A9_A10;

const CONFIG: cpu::Config = {
    let mut c = cpu::Config::new(16000000);
    c.fll = false;
    *c.systick(monitor::systick_handler).adc().debug().i2c()};

const ADC_CHANNELS: [u32; 3] = [9, 17, 18];

const ISENSE_INDEX: usize = 0; // Pin 11
const VSENSE1_INDEX: usize = 1; // Pin 14
const VREF_INDEX: usize = 2;   // Pin 15

const ISENSE_ZERO: i32 = 32646;
// Current in A that gives 3.3V.
const ISENSE_FS: f64 = 25.;
const ISENSE_SCALE: i32 = (ISENSE_FS as f64 * 1000. + 0.5) as i32;

// Full scale on vsense in V.
const VSENSE1_FS: f64 = 118.0 / 18.0 * 3.3;
const VSENSE1_SHIFT: u32 = 1;

const VSENSE2_INDEX: usize = VSENSE1_INDEX;
const VSENSE2_FS: f64 = VSENSE1_FS;
const VSENSE2_SHIFT: u32 = 1;

const POWER1_SHIFT: u32 = 0;
const POWER2_SHIFT: u32 = 0;

#[test]
fn test_vconvert() {
    const FS: f64 = 118.0 / 18.0 * 3.3 * 1000.0;
    assert!(20000. <= FS && FS <= 25000., "{FS}");
    assert_eq!(monitor::vconvert(0), 0);
    let high = monitor::vconvert(0xffff);
    assert!((high as f64 - FS * 65535./65536.).abs() <= 1.);
}
