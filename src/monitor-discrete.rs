#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![allow(internal_features)]
#![allow(unpredictable_function_pointer_comparisons)]
#![feature(const_default, const_trait_impl, derive_const)]
#![feature(format_args_nl)]
#![feature(link_llvm_intrinsics)]

mod adc;
mod cpu;
mod decimal;
mod dma;
mod debug;
mod font;
mod fxls8971;
mod i2c;
mod monitor;
mod oled;
mod utils;

use monitor::{
    CONFIG, DEBUG_ENABLE, I2C_LINES, PConvert, VConvert, debug_fmt, main};

const ADC_CHANNELS: [u32; 4] = [8, 10, 14, 18];

const ISENSE_INDEX: usize = 3;
const VSENSE1_INDEX: usize = 1;
const VSENSE2_INDEX: usize = 2;
const VREF_INDEX: usize = 0;

const ISENSE_ZERO: i32 = 32784;
/// A to V transfer of isense circuit.
const ISENSE_GAIN: f64 = 0.001 * 101. * 100. / 39.;
/// Current in A to give 3.3V.
const ISENSE_FS: f64 = 3.3 / ISENSE_GAIN;
const ISENSE_SCALE: i32 = (ISENSE_FS * 1000.) as i32;

const VCONVERT1: VConvert = VConvert::new(VSENSE1_FS, 2);
const VCONVERT2: VConvert = VConvert::new(VSENSE2_FS, 0);

// Increase the multiplier to keep the precision up.
const PCONVERT1: PConvert = PConvert::new(ISENSE_FS, VSENSE1_FS, 2, 0);
// Decreate the multiplier to keep the precision up.
const PCONVERT2: PConvert = PConvert::new(ISENSE_FS, VSENSE2_FS, 0, 1);

// Input voltage in V for full scale on VSENSE1.
const VSENSE1_FS: f64 = (220. + 100.) / 100. * 3.3;

// Full scale voltage on VSENSE2.
const VSENSE2_FS: f64 = (330. + 22.) / 22. * 3.3;

const HAVE_ACCEL: bool = true;
