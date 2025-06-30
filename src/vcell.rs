
use core::cell::SyncUnsafeCell;

/// Interrupt safe volatile cell, has read/write for scalar types.
#[repr(transparent)]
pub struct VCell<T>(SyncUnsafeCell<T>);
/// A basic cell for storage.  Shared access is safe, mutable access is
/// unsafe.
#[repr(transparent)]
pub struct UCell<T>(SyncUnsafeCell<T>);

impl<T: Sync> VCell<T> {
    #[inline]
    pub const fn new(v: T) -> Self {Self(SyncUnsafeCell::new(v))}
    #[inline]
    pub fn as_ptr(&self) -> *mut T {self.0.get()}
    #[inline]
    pub fn as_mut(&mut self) -> &mut T {self.0.get_mut()}
}

impl<T: Sync> UCell<T> {
    #[inline]
    pub const fn new(v: T) -> Self {Self(SyncUnsafeCell::new(v))}
    #[inline]
    pub fn as_ref(&self) -> &T {unsafe{&*(self.0.get() as *const T)}}
    #[inline]
    /// We are naughty and use this in interrupts, using barriers.
    pub unsafe fn as_mut(&self) -> &mut T {unsafe{&mut *self.0.get()}}
}

#[inline(always)]
pub fn barrier() {
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
}

#[inline(always)]
#[allow(non_snake_case)]
pub fn WFE() {
    if cfg!(target_arch = "arm") {
        unsafe {
            core::arch::asm!("wfe", options(nomem, preserves_flags, nostack))};
    }
    else {
        panic!("wfe!");
    }
}


#[allow(dead_code)]
pub mod interrupt {
    // We don't use disabling interrupts to transfer ownership, so no need for
    // the enable to be unsafe.
    #[cfg(target_arch = "arm")]
    pub fn enable() {unsafe{cortex_m::interrupt::enable()}}
    #[cfg(target_arch = "arm")]
    pub fn disable() {cortex_m::interrupt::disable()}
    #[cfg(not(target_arch = "arm"))]
    pub fn enable() { }
    #[cfg(not(target_arch = "arm"))]
    pub fn disable() { }
}

impl<T: Sync> core::ops::Deref for UCell<T> {
    type Target = T;
    fn deref(&self) -> &T {self.as_ref()}
}

macro_rules! VCellImpl {
    ($($t:ty),*) => {$(
        #[allow(dead_code)]
        impl VCell<$t> {
            #[inline(always)]
            pub fn read(&self) -> $t {
                unsafe {core::ptr::read_volatile(self.as_ptr())}
            }
            #[inline(always)]
            pub fn write(&self, v: $t) {
                unsafe {core::ptr::write_volatile(self.as_ptr(), v)};
            }
        }
    )*};
}

VCellImpl!(bool, i8, u8, i16, u16, i32, u32, isize, usize, char);
