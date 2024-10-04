use super::{main::EcxContext, r#type::Error};

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

pub fn elist_to_string(context: &EcxContext) -> String {
    todo!()
}

#[cfg(feature = "ec_ver1")]
pub fn ec_elist_to_string() -> String {
    todo!()
}
