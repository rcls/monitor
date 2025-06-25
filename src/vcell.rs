
use core::cell::SyncUnsafeCell;

/// Interrupt safe volatile cell, has read/write for scalar types.
#[repr(transparent)]
pub struct VCell<T>(SyncUnsafeCell<T>);
/// A basic cell for storage.  Shared access is safe, mutable access is
/// unsafe.
#[repr(transparent)]
pub struct UCell<T>(SyncUnsafeCell<T>);

impl<T> VCell<T> {
    #[inline]
    pub const fn new(v: T) -> Self {Self(SyncUnsafeCell::new(v))}

    #[inline]
    pub fn as_ptr(&self) -> *mut T {self.0.get()}
    #[inline]
    pub fn as_mut(&mut self) -> &mut T {self.0.get_mut()}
}

impl<T> From<T> for VCell<T> {
    fn from(v: T) -> VCell<T> {VCell::new(v)}
}

impl<T> UCell<T> {
    #[inline]
    pub const fn new(v: T) -> Self {Self(SyncUnsafeCell::new(v))}
    #[inline]
    pub fn as_ref(&self) -> &T {unsafe{&*(self.0.get() as *const T)}}
    #[inline]
    /// We are naughty and use this in interrupts, using barriers.
    pub unsafe fn as_mut(&self) -> &mut T {unsafe{&mut *self.0.get()}}
}

macro_rules! VCellImpl {
    ($t:ty) => {
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
    }
}

VCellImpl!(bool);
VCellImpl!(i16);
VCellImpl!(u8);
VCellImpl!(u16);
