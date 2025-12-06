pub use stm_common::dma::{DMA_Channel, Flat};

pub type Dma = stm32u031::dma1::RegisterBlock;

pub fn dma() -> &'static Dma {unsafe {&*stm32u031::DMA1::ptr()}}
