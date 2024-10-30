//! Module for debugging and time measurement.

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

    pub fn restart(&mut self, timeout: Duration) {
        *self = Self::new(timeout);
    }

    pub fn is_expired(&self) -> bool {
        self.stop_time.elapsed().is_ok()
    }
}

#[macro_export]
macro_rules! ec_print {
    ($($arg:tt)*) => {{
        if cfg!(debug_assertions) {
            print!($($arg)*);
        }
    }};
}

#[macro_export]
macro_rules! ec_println {
    () => {
        if cfg!(debug_assertions) {
            println!();
        }
    };
    ($($arg:tt)*) => {{
        if cfg!(debug_assertions) {
            println!($($arg)*);
        }
    }};
}
