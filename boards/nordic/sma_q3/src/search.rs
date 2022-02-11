use kernel::debug;
use kernel::hil;

use kernel::hil::time::ConvertTicks;


pub struct Search<'a, A: hil::time::Alarm<'a>> {
    alarm: &'a A
}

impl<'a, A: hil::time::Alarm<'a>> Search<'a, A> {
    pub fn new(alarm: &'a A) -> Self {
        Self {
            alarm: alarm
        }
    }

    pub fn start(&self) {
        let delay = self.alarm.ticks_from_ms(500);
        self.alarm.set_alarm(self.alarm.now(), delay);
    }
}

impl<'a, A: hil::time::Alarm<'a>> hil::time::AlarmClient for Search<'a, A> {
    fn alarm(&self) {
        self.start();
        debug!("Hello world");
    }
}
