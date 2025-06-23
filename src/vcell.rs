
use core::cell::SyncUnsafeCell;

#[repr(transparent)]
pub struct VCell<T>(SyncUnsafeCell<T>);

impl<T> VCell<T> {
    #[inline(always)]
    pub const fn new(v: T) -> Self {Self(SyncUnsafeCell::new(v))}

    #[inline(always)]
    pub fn read(&self) -> T {
        unsafe {core::ptr::read_volatile(self.as_ptr())}
    }

    #[inline(always)]
    pub fn write(&self, v: T) {
        unsafe {
            core::ptr::write_volatile(self.as_ptr(), v);
        }
    }

    #[inline(always)]
    pub fn as_ptr(&self) -> *mut T {
        self.0.get()
    }
}

impl<T> From<T> for VCell<T> {
    fn from(v: T) -> VCell<T> {VCell::new(v)}
}
