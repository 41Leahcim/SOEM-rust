use std::time::{Duration, SystemTime};

pub struct OsalTimer {
    pub stop_time: SystemTime,
}

impl OsalTimer {
    pub fn new(timeout: Duration) -> Self {
        Self {
            stop_time: SystemTime::now() + timeout,
        }
    }

    pub fn start(&mut self, timeout: Duration) {
        *self = Self::new(timeout);
    }

    pub fn is_expired(&self) -> bool {
        self.stop_time.elapsed().is_ok()
    }
}
