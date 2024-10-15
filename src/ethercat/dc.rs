use std::{task::Context, time::Duration};

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
