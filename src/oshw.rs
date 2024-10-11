use std::{ffi::CStr, str::FromStr};

use libc::if_nameindex;

use crate::ethercat::main::EcAdapter;

pub mod nicdrv;

/// Host to network byte order (i.e. to big endian)
///
/// Not that Ethercat uses little endian byte order, except for the Ethernet
/// header which is big endian as usual
pub fn htons(host_short: u16) -> u16 {
    host_short.to_be()
}

/// Network (i.e. big endian) to host byte order.
///
/// Note that Ethercat uses little endian byte order, except for the Ethernet
/// header which is big endian as usual
pub fn ntohs(network_short: u16) -> u16 {
    network_short.to_le()
}

/// Create list over available network adapters.
///
/// # Returns
/// First element in linked list of adapters
pub fn find_adaters() -> Vec<EcAdapter> {
    // Iterate all devices and create a local copy holding the name and description
    let ids = unsafe { if_nameindex() };
    (0..)
        .filter_map(|i| unsafe { ids.add(i).as_mut() })
        .take_while(|id| id.if_index != 0)
        .map(|id| {
            // Fetch description and name, in Linux they are the same.
            let name =
                heapless::String::from_str(unsafe { CStr::from_ptr(id.if_name) }.to_str().unwrap())
                    .unwrap();
            let desc = name.clone();
            EcAdapter { name, desc }
        })
        .collect()
}
