//! Module for debugging and time measurement.

use std::time::{Duration, Instant};

pub struct OsalTimer {
    stop_time: Instant,
}

impl OsalTimer {
    pub fn new(timeout: Duration) -> Self {
        Self {
            stop_time: Instant::now() + timeout,
        }
    }

    pub fn restart(&mut self, timeout: Duration) {
        *self = Self::new(timeout);
    }

    pub fn is_expired(&self) -> bool {
        Instant::now() >= self.stop_time
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
