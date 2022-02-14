use core::cell::RefCell;
use kernel::hil;
use kernel::hil::time::ConvertTicks;

#[derive(Clone, Copy)]
enum Order {
    Unstarted,
    Shining(usize),
    Finished(usize),
}

pub trait Callable {
    fn next(&mut self);
}


pub struct Periodic<'a, A: hil::time::Alarm<'a>, F> {
    alarm: &'a A,
    f: RefCell<F>,
}

impl<'a, A: hil::time::Alarm<'a>, F> Periodic<'a, A, F> {
    pub fn new(alarm: &'a A, f: F) -> Self {
        Self {
            alarm,
            f: RefCell::new(f),
        }
    }

    pub fn arm(&self) {
        let delay = self.alarm.ticks_from_ms(1000);
        self.alarm.set_alarm(self.alarm.now(), delay);
    }
}

impl<'a, A: hil::time::Alarm<'a>, F: Callable> hil::time::AlarmClient for Periodic<'a, A, F> {
    fn alarm(&self) {
        self.arm();
        let mut f = self.f.borrow_mut();
        f.next();
    }
}
