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

//const ADC_CHANNELS: [u32; 3] = [9, 17, 18];
//const ADC_CHANNELS: [u32; 19] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,18];
const ADC_CHANNELS: [u32; 4] = [8, 10, 14, 18];

const ISENSE_INDEX: usize = 3;
const VSENSE1_INDEX: usize = 1;
const VSENSE2_INDEX: usize = 2;
const VREF_INDEX: usize = 0;

const ISENSE_ZERO: i32 = 32743;
/// A to V transfer of isense circuit.
const ISENSE_GAIN: f64 = 0.001 * 101. * 100. / 39.;
/// Current in A to give 3.3V.
const ISENSE_FS: f64 = 3.3 / ISENSE_GAIN;
const ISENSE_SCALE: i32 = (ISENSE_FS * 1000.) as i32;

// Input voltage in V for full scale on VSENSE1.
const VSENSE1_FS: f64 = (220. + 100.) / 100. * 3.3;
const VSENSE1_SHIFT: u32 = 2;

const VSENSE2_FS: f64 = (330. + 22.) / 22. * 3.3;
const VSENSE2_SHIFT: u32 = 0;

// Power1 calculation loses precision unless we increase the multiplier.
const POWER1_SHIFT: u32 = 2;

// Power2 calculation can overflow unless we reduce the multiplier.
const POWER2_SHIFT: u32 = 1;

