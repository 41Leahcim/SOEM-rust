use std::{any::Any, time::Duration};

use super::main::Context;

pub const NODE_OFFSET: u16 = 0x1000;
pub const TEMP_NODE: u16 = 0xFFFF;

pub fn config_init(context: &mut Context, usetable: u8) -> i32 {
    todo!()
}

pub fn config_map_group(context: &mut Context, io_map: &mut [Box<dyn Any>], group: u8) -> i32 {
    todo!()
}

pub fn config_overlap_map_group(
    context: &mut Context,
    io_map: &mut [Box<dyn Any>],
    group: u8,
) -> i32 {
    todo!()
}

pub fn config_map_group_aligned(
    context: &mut Context,
    io_map: &mut [Box<dyn Any>],
    group: u8,
) -> i32 {
    todo!()
}

pub fn recover_slave(context: &mut Context, slave: u16, timeout: Duration) -> i32 {
    todo!()
}

pub fn reconfig_slave(context: &mut Context, slave: u16, timeout: Duration) -> i32 {
    todo!()
}
