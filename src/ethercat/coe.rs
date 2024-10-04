use std::{any::Any, time::Duration};

use heapless::String as HeaplessString;

use super::main::MAX_NAME_LENGTH;
use super::{main::EcxContext, r#type::Datatype};

/// Max entries in object description list
pub const MAX_OBJECT_DESCRIPTION_LIST_SIZE: usize = 1024;

/// Max entries in Object Entries list
pub const MAX_OBJECT_ENTRY_LIST_SIZE: usize = 256;

/// Storage for object description list
pub struct ObjectDescriptionList {
    /// Slave number
    pub slave: u16,

    /// Number of entries in list
    pub entries: u16,

    /// Array of indexes
    pub index: [u16; MAX_OBJECT_DESCRIPTION_LIST_SIZE],

    /// Array of datatypes, see EtherCAT specification
    pub data_type: [Datatype; MAX_OBJECT_DESCRIPTION_LIST_SIZE],

    /// Array of object codes, see EtherCAT specification
    pub object_code: [u16; MAX_OBJECT_DESCRIPTION_LIST_SIZE],

    /// Number of subindexes for each index
    pub max_sub: [u8; MAX_OBJECT_DESCRIPTION_LIST_SIZE],

    pub name: [HeaplessString<MAX_NAME_LENGTH>; MAX_OBJECT_DESCRIPTION_LIST_SIZE],
}

/// Storage for object list entry information
pub struct ObjectEntryList {
    /// Number of entries in list
    pub entries: u16,

    /// Array of value info, see EtherCAT specification
    pub value_info: [u8; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of value data types, see EtherCAT specification
    pub data_type: [u16; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of bit lengths, see EtherCAT specification
    pub bit_length: [u16; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of object access bits, see EtherCAT specification
    pub object_access: [u16; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Textual description of each index
    pub name: [HeaplessString<MAX_NAME_LENGTH>; MAX_OBJECT_ENTRY_LIST_SIZE],
}

#[cfg(feature = "ec_ver1")]
pub mod ec_ver1 {
    use std::{any::Any, time::Duration};

    use super::{ObjectDescriptionList, ObjectEntryList};

    pub fn sdo_error(slave: u16, index: u16, sub_index: u8, abort_code: i32) {
        todo!()
    }

    pub fn sdo_read(
        slave: u16,
        index: u16,
        sub_index: u8,
        ca: bool,
        pointer_length: &mut usize,
        pointer: &mut [Box<dyn Any>],
        timeout: Duration,
    ) -> i32 {
        todo!()
    }

    pub fn sdo_write(
        slave: u16,
        index: u16,
        sub_index: u8,
        ca: bool,
        pointer: &[Box<dyn Any>],
        timeout: Duration,
    ) {
        todo!()
    }

    pub fn rx_pdo(slave: u16, rx_pdo_number: u16, pointer: &[Box<dyn Any>]) -> i32 {
        todo!()
    }

    pub fn tx_pdo(
        slave: u16,
        tx_pdo_number: u16,
        length: &mut usize,
        pointer: &[Box<dyn Any>],
        timeout: Duration,
    ) -> i32 {
        todo!()
    }

    pub fn read_pdo_map(slave: u16, osize: &mut [u32], isize: &mut [u32]) -> i32 {
        todo!()
    }

    pub fn read_pdo_map_ca(slave: u16, thread_n: i32, osize: &mut [u32], isize: &mut [u32]) -> i32 {
        todo!()
    }

    pub fn read_od_list(slave: u16, od_list: &mut ObjectDescriptionList) -> i32 {
        todo!()
    }

    pub fn read_od_description(item: u16, od_list: &mut ObjectDescriptionList) -> i32 {
        todo!()
    }

    pub fn read_oe_single(
        item: u16,
        sub_index: u8,
        od_list: &mut ObjectDescriptionList,
        oe_list: &mut ObjectEntryList,
    ) -> i32 {
        todo!()
    }

    pub fn read_oe(
        item: u16,
        od_list: ObjectDescriptionList,
        oe_list: &mut ObjectEntryList,
    ) -> i32 {
        todo!()
    }
}

pub fn sdo_error(context: &mut EcxContext, slave: u16, index: u16, sub_index: u8, abort_code: i32) {
    todo!()
}

pub fn sdo_read(
    context: &mut EcxContext,
    slave: u16,
    index: u16,
    sub_index: u8,
    ca: bool,
    size: &mut usize,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn sdo_write(
    context: &mut EcxContext,
    slave: u16,
    index: u16,
    sub_index: u8,
    ca: bool,
    pointer: &[Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn rx_pdo(
    context: &mut EcxContext,
    slave: u16,
    rx_pdo_number: u16,
    pointer: &[Box<dyn Any>],
) -> i32 {
    todo!()
}

pub fn tx_pdo(
    context: &mut EcxContext,
    slave: u16,
    tx_pdo_number: u16,
    size: &mut usize,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn read_pdo_map(context: &mut EcxContext, slave: u16, osize: &mut u32, isize: &mut u32) -> i32 {
    todo!()
}

pub fn read_pdo_map_ca(
    context: &mut EcxContext,
    slave: u16,
    thread_n: i32,
    osize: &mut u32,
    isize: &mut u32,
) -> i32 {
    todo!()
}

pub fn read_od_list(
    context: &mut EcxContext,
    slave: u16,
    od_list: &mut ObjectDescriptionList,
) -> i32 {
    todo!()
}

pub fn read_od_description(
    context: &mut EcxContext,
    item: u16,
    od_list: &mut ObjectDescriptionList,
) -> i32 {
    todo!()
}

pub fn read_oe_single(
    context: &mut EcxContext,
    item: u16,
    sub_index: u8,
    od_list: &mut ObjectDescriptionList,
    oe_list: &mut ObjectEntryList,
) -> i32 {
    todo!()
}

pub fn read_oe(
    context: &mut EcxContext,
    item: u16,
    od_list: &mut ObjectDescriptionList,
    oe_list: &mut ObjectEntryList,
) -> i32 {
    todo!()
}
