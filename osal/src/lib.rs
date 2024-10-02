use std::time::{Duration, SystemTime};

pub struct OsalTimer {
    pub stop_time: SystemTime,
}

impl OsalTimer {
    pub fn start(&mut self, timeout: Duration) {
        self.stop_time = SystemTime::now() + timeout;
    }

    pub fn is_expired(&self) -> bool {
        self.stop_time.elapsed().is_ok()
    }
}
