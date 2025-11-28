
use crate::{Completer, System, i2c};
use i2c::Result;

const ENS212: u8 = 0x8a;

macro_rules!dbgln {($($tt:tt)*) => {if false {crate::dbgln!($($tt)*)}};}

pub fn start() -> Completer {
    dbgln!("ENS212 start");
    i2c::init();
    // TODO - let other processing happen in the shadow.
    Completer(i2c::write(ENS212, &[0x22u8, 3]), start_done)
}

fn start_done(sys: &mut System, ok: Result) {
    dbgln!("ENS212 start done {ok:?}");
    if ok.is_err() {
        sys.humidity = !0;
    }
}

static HUMIDITY: crate::vcell::UCell<u16> = Default::default();

pub fn get() -> Completer {
    dbgln!("ENS212 get");
    i2c::init();
    Completer(
        i2c::read_reg(ENS212, 0x33, unsafe {HUMIDITY.as_mut()}),
        get_done)
}

fn get_done(sys: &mut System, ok: Result) {
    dbgln!("ENS212 get done {ok:?} {}", HUMIDITY.as_ref());
    if ok.is_ok() {
        sys.humidity = ((*HUMIDITY.as_ref() as u32 * 10 + 256) / 512).min(999);
    } else {
        sys.humidity = !0
    };
}
