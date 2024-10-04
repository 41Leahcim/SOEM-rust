use std::{any::Any, time::Duration};

use super::main::EcxContext;

#[cfg(feature = "ec_ver1")]
pub mod ec_ver1 {
    use std::{any::Any, time::Duration};

    pub fn foe_define_hook(hook: &mut [Box<dyn Any>]) -> i32 {
        todo!()
    }

    pub fn foe_read(
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
        slave: u16,
        file_name: &str,
        password: u32,
        pointer: &mut [Box<dyn Any>],
        timeout: Duration,
    ) -> i32 {
        todo!()
    }
}

pub fn foe_define_hook(context: &mut EcxContext, hook: &mut [Box<dyn Any>]) -> i32 {
    todo!()
}

pub fn foe_read(
    context: &mut EcxContext,
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
    context: &mut EcxContext,
    slave: u16,
    file_name: &str,
    password: u32,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}
