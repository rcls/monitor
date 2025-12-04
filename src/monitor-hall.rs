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
mod fxls8971; // Not actually on board.
mod i2c;
mod monitor;
mod oled;
mod utils;
#[path = "../stm-common/vcell.rs"]
mod vcell;

use monitor::*;

const ADC_CHANNELS: [u32; 3] = [9, 17, 18];

const ISENSE_INDEX: usize = 0; // Pin 11
const VSENSE1_INDEX: usize = 1; // Pin 14
const VREF_INDEX: usize = 2;   // Pin 15

const ISENSE_ZERO: i32 = 32646;
// Current in A that gives 3.3V.
const ISENSE_FS: f64 = 25.;
const ISENSE_SCALE: i32 = (ISENSE_FS * 1000. + 0.5) as i32;

// Full scale on vsense in V.
const VSENSE_FS: f64 = 118.0 / 18.0 * 3.3;

const VCONVERT1: monitor::VConvert = monitor::VConvert::new(VSENSE_FS, 1);
const VCONVERT2: monitor::VConvert = VCONVERT1;

const VSENSE2_INDEX: usize = VSENSE1_INDEX;

const PCONVERT1: monitor::PConvert = monitor::PConvert::new(
    ISENSE_FS, VSENSE_FS, 0, 0);
const PCONVERT2: monitor::PConvert = PCONVERT1;

const HAVE_ACCEL: bool = false;

#[test]
fn test_vconvert() {
    const FS: f64 = 118.0 / 18.0 * 3.3 * 1000.0;
    assert!(20000. <= FS && FS <= 25000., "{FS}");
    assert_eq!(VCONVERT1.convert(0), 0);
    let high = VCONVERT1.convert(0xffff);
    assert!((high as f64 - FS * 65535./65536.).abs() <= 1.);
}
