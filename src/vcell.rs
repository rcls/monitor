
use core::cell::SyncUnsafeCell;

/// Interrupt safe volatile cell, has read/write for scalar types.
#[repr(transparent)]
pub struct VCell<T>(SyncUnsafeCell<T>);
/// A basic cell for storage.  Shared access is safe, mutable access is
/// unsafe.
#[repr(transparent)]
pub struct UCell<T>(SyncUnsafeCell<T>);

impl<T> VCell<T> {
    #[inline(always)]
    pub const fn new(v: T) -> Self {Self(SyncUnsafeCell::new(v))}

    #[inline(always)]
    pub fn as_ptr(&self) -> *mut T {self.0.get()}
    #[inline(always)]
    pub fn as_mut(&mut self) -> &mut T {self.0.get_mut()}
}

impl<T> From<T> for VCell<T> {
    fn from(v: T) -> VCell<T> {VCell::new(v)}
}

impl<T> UCell<T> {
    #[inline(always)]
    pub const fn new(v: T) -> Self {Self(SyncUnsafeCell::new(v))}
    #[inline(always)]
    pub fn as_ref(&self) -> &T {unsafe{&*(self.0.get() as *const T)}}
    #[inline(always)]
    /// Caller must ensure no other references to content exist.
    /// We are naughty and use this in interrupts, using barriers.
    pub unsafe fn as_mut(&self) -> &mut T {unsafe{&mut *self.0.get()}}
}

pub trait VCellAccess<T> {
    fn read(&self) -> T;
    fn write(&self, v: T);
}

macro_rules! VCellImpl {
    ($t:ty) => {
        impl VCellAccess<$t> for VCell<$t> {
            #[inline(always)]
            fn read(&self) -> $t {
                unsafe {core::ptr::read_volatile(self.as_ptr())}
            }
            #[inline(always)]
            fn write(&self, v: $t) {
                unsafe {core::ptr::write_volatile(self.as_ptr(), v)};
            }
        }
    }
}

VCellImpl!(bool);
VCellImpl!(i16);
VCellImpl!(u8);
VCellImpl!(u16);
