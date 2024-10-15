use std::{any::Any, time::Duration};

use super::main::Context;

pub fn foe_define_hook(context: &mut Context, hook: &mut [Box<dyn Any>]) -> i32 {
    todo!()
}

pub fn foe_read(
    context: &mut Context,
    slave: u16,
    file_name: &str,
    password: u32,
    size: &mut usize,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn foe_write(
    context: &mut Context,
    slave: u16,
    file_name: &str,
    password: u32,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}
