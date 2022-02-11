use core::cell::Cell;
use kernel::debug;
use kernel::hil;
use kernel::hil::led;
use kernel::hil::time::ConvertTicks;

#[derive(Clone, Copy)]
enum Order {
    Unstarted,
    Shining(usize),
    Finished(usize),
}


pub struct Search<'a, A: hil::time::Alarm<'a>, L> {
    alarm: &'a A,
    leds: L,
    pos: Cell<Order>,
}

impl<'a, A: hil::time::Alarm<'a>, L> Search<'a, A, L> {
    pub fn new(alarm: &'a A, leds: L) -> Self {
        Self {
            alarm,
            leds,
            pos: Cell::new(Order::Unstarted),
        }
    }

    pub fn start(&self) {
        let delay = self.alarm.ticks_from_ms(1500);
        self.alarm.set_alarm(self.alarm.now(), delay);
    }
}

use capsules::led::LedDriver;

impl<'a, A: hil::time::Alarm<'a>, L: led::Led, const NUM: usize> hil::time::AlarmClient for Search<'a, A, &'a LedDriver<'a, L, NUM>> {
    fn alarm(&self) {
        self.start();
        use Order::*;
        self.pos.set(match self.pos.get() {
            Unstarted => Shining(0),
            Shining(p) => Finished(p),
            Finished(p) => Shining((p + 1) % NUM),
        });

        match self.pos.get() {
            Shining(p) => {
                debug!("trying {}", p);
                self.leds.leds[p].on();
            },
            Finished(p) => {
                debug!("ok");
                self.leds.leds[p].off();
            },
            _ => {},
        };
    }
}
