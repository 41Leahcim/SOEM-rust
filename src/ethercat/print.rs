//! Module to convert EtherCAT errors to readable messages.
//!
//! SDO abort messages and AL status codes are used to relay slave errors to
//! the user application. This module converts the binary codes to readable text.
//! For the defined error codes see the EtherCAT protocol documentation.

use super::{main::Context, r#type::Error};

pub fn sdo_error_to_string(sdo_error_code: u32) -> String {
    todo!()
}

pub fn al_status_code_to_string(al_status_code: u16) -> String {
    todo!()
}

pub fn soee_error_to_string(error_code: u16) -> String {
    todo!()
}

pub fn mailbox_error_to_string(error_code: u16) -> String {
    todo!()
}

pub fn ecx_error_to_string(ec: Error) -> String {
    todo!()
}

pub fn elist_to_string(context: &Context) -> String {
    todo!()
}
