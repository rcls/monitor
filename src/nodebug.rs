
#[macro_export]
macro_rules!dbgln {
   ($($tt:tt)*) => {if false {format_args_nl!($($tt)*);}};
}

#[macro_export]
macro_rules!sdbgln {
   ($($tt:tt)*) => {if false {format_args_nl!($($tt)*);}};
}

pub fn init() {}

impl crate::cpu::VectorTable {
    pub const fn debug_isr(&mut self) -> &mut Self {self}
}

#[allow(dead_code)]
pub fn drain() {}
