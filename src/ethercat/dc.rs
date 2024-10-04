use std::{task::Context, time::Duration};

#[cfg(feature = "ec_ver1")]
pub mod ec_ver1 {
    use std::time::Duration;

    pub fn configdc() -> bool {
        todo!()
    }

    pub fn dcsync0(slave: u16, active: bool, cycle_time: Duration, cycle_shift: i32) {
        todo!()
    }

    pub fn dsync01(
        slave: u16,
        active: bool,
        cycle_time0: Duration,
        cycle_time1: Duration,
        cycle_shift: i32,
    ) {
        todo!()
    }
}

pub fn config_dc(context: &mut Context) -> bool {
    todo!()
}

pub fn dsync0(
    context: &mut Context,
    slave: u16,
    active: bool,
    cycle_time: Duration,
    cycle_shift: i32,
) {
    todo!()
}

pub fn dsync01(
    context: &mut Context,
    slave: u16,
    active: bool,
    cycle_time0: Duration,
    cycle_time1: Duration,
    cycle_shift: i32,
) {
    todo!()
}
