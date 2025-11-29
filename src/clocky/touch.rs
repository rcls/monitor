use super::{LCD_WIDTH, State, System, TICKS_PER_SEC, TOUCH_EVALUATE, cal};

use crate::tsc;
use tsc::pad;

const TOUCH_TIME_OUT: u32 = 10 * TICKS_PER_SEC;
const REPEAT_FIRST: u32 = TICKS_PER_SEC;
const REPEAT_AGAIN: u32 = TICKS_PER_SEC / 2;

pub fn process(sys: &mut System) {
    let (pad, val) = tsc::retrieve();
    if TOUCH_EVALUATE {
        sys.scratch = val;
        return;
    }

    if !state(sys, pad) {
        return;
    }

    match pad {
        pad::NONE => sys.set_sub_state(0), // Cancel any blinky stuff.
        pad::MENU => {                  // Rotate sub-state.
            let s = sys.state();
            let ss = sys.sub_state();
            let max = match s {
                State::Date | State::Time => 3,
                State::Cal => cal::MAX,
                State::Pres => if LCD_WIDTH == 6 {0} else {1},
                _ => 0};
            sys.set_sub_state(if ss >= max {0} else {ss + 1});
        },
        _ => plus_minus(sys, pad == pad::PLUS),
    }

    // Update the cal clock output.
    let rcc = unsafe {&*stm32u031::RCC::ptr()};
    if sys.state() == State::Cal && sys.sub_state() != 0 {
        rcc.BDCR.modify(
            |_, w| w.LSCOSEL().set_bit().LSCOEN().set_bit().LSESYSEN().set_bit());
    }
    else if sys.state() != State::Cal {
        // FIXME - what happens if we take a real system reset with the LSCO
        // active?
        rcc.BDCR.modify(|_, w| w.LSCOEN().clear_bit().LSESYSEN().clear_bit());
    }
}

/// Decide what to do with the current touch pad state.  Update timers, and
/// return true if this is an event we should process (down, up, auto-repeat).
fn state(sys: &mut System, pad: u32) -> bool {
    let previous = sys.touch();
    let timer;
    let result;
    if pad != previous {
        timer = if pad != pad::NONE {REPEAT_FIRST} else {TOUCH_TIME_OUT};
        result = pad != 0;
    }
    else {
        let t = sys.touch_timer();
        timer = if t > 1 {t - 1} else {match pad {
            pad::NONE => 0,
            pad::MENU => REPEAT_FIRST,
            _ => REPEAT_AGAIN,
        }};
        result = t == 1;
    }
    sys.set_touch(pad, timer);
    return result;
}

fn plus_minus(sys: &mut System, is_plus: bool) {
    let state = sys.state();

    if sys.sub_state() == 0 {
        // Rotate the main state.
        let mut state = state;
        loop {
            state = if is_plus {state.next()} else {state.prev()};
            if state == State::Pres && !sys.have_pressure()
                || state == State::Humi && !sys.have_humidity() {
            }
            else {
                break;
            }
        }
        sys.set_state(state);
        return;
    }

    match state {
        State::Cal => super::cal_plus_minus(sys, is_plus),
        State::Pres => if LCD_WIDTH < 6 {
            let point = sys.pressure_point();
            let point = if is_plus {
                if point + LCD_WIDTH >= 6 {0} else {point + 1}
            }
            else {
                if point == 0 {6 - LCD_WIDTH} else {point - 1}
            };
            sys.set_pressure_point(point);
        }
        // Date or Time.  The numberings are different to the sub-state
        // numbering.
        State::Time => super::rtc_adjust(3 - sys.sub_state(), is_plus),
        State::Date => super::rtc_adjust(6 - sys.sub_state(), is_plus),
        _ => (),
    }
}
