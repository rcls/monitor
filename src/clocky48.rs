#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]
#![allow(internal_features)]
#![allow(unpredictable_function_pointer_comparisons)]
#![feature(const_default)]
#![feature(const_trait_impl)]
#![feature(derive_const)]
#![feature(format_args_nl)]
#![feature(link_llvm_intrinsics)]

mod clocky;
mod cpu;
mod debug;
mod dma;
mod ens;
mod i2c;
mod lcd;
mod rtc;
mod tmp117;
mod tsc;
mod utils;
mod vcell;

const LCD_WIDTH: usize = 6;

use clocky::*;
