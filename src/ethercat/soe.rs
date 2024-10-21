//! Servo over EtherCAT (SoE) module

use std::{any::Any, time::Duration};

use super::main::Context;

pub const DATASTATE_BIT: u8 = 0x01;
pub const NAME_BIT: u8 = 0x02;
pub const ATTRIBUTE_BIT: u8 = 0x04;
pub const UNIT_BIT: u8 = 0x08;
pub const MIN_BIT: u8 = 0x10;
pub const MAX_BIT: u8 = 0x20;
pub const VALUE_BIT: u8 = 0x40;
pub const DEFAULT_BIT: u8 = 0x80;

pub const MAX_NAME_LENGTH: usize = 60;
pub const MAX_MAPPING_LENGTH: usize = 64;

pub const IDN_MDT_CONFIG: u8 = 24;
pub const IDN_AT_CONFIG: u8 = 16;

/// SoE name
pub struct SoEName {
    /// Current length in bytes of list
    pub current_length: u16,

    /// Maximum length in bytes of list
    pub max_length: u16,

    pub name: heapless::String<MAX_NAME_LENGTH>,
}

/// SoE list
pub enum SoEListValue {
    Byte([u8; 8]),
    Word([u16; 4]),
    Dword([u32; 2]),
    Lword(u64),
}

pub struct SoEList {
    /// Current length in bytes of list
    pub current_length: u16,

    /// Maximum length in bytes of list
    pub max_length: u16,

    pub data: SoEListValue,
}

/// SoE IDN mapping
pub struct SoeMapping {
    /// Current length in bytes of list
    current_length: u16,

    /// Maximum length in bytes of list
    max_length: u16,

    idn: [u16; MAX_MAPPING_LENGTH],
}

pub const LENGTH1: u8 = 0;
pub const LENGTH2: u8 = 1;
pub const LENGTH4: u8 = 2;
pub const LENGTH8: u8 = 3;

pub enum SoeType {
    Binary,
    Uint,
    Int,
    Hex,
    String,
    Idn,
    Float,
    Parameter,
}

pub struct SoeAttribute {
    /// Evaluation factor for display purposes
    pub eval_factor: [u32; 16],

    /// Length of IDN element(s)
    pub length: [u32; 2],

    /// IDN is list
    pub list: u32,

    /// IDN is command
    pub command: u32,

    /// datatype
    pub datatype: [u32; 3],
    reserved1: u32,

    /// Decimals to display in float datatype
    pub decimals: [u32; 4],

    /// Write protected pre-op
    pub wppreop: u32,

    /// Write protected in safe-op
    pub wpsafeop: u32,

    /// Write protected in op
    pub wpop: u32,
    reserved2: u32,
}

pub fn soe_read(
    context: &mut Context,
    slave: u16,
    drive_no: u8,
    elementflags: u8,
    idn: u16,
    size: &mut usize,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn soe_write(
    context: &mut Context,
    slave: u16,
    drive_no: u8,
    elementflags: u8,
    idn: u16,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn read_id_nmap(context: &mut Context, slave: u16, osize: &mut u32, isize: &mut u32) -> i32 {
    todo!()
}
