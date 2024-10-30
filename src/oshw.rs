//! Parent module for low level operations.

use std::{ffi::CStr, str::FromStr};

use libc::if_nameindex;
use num_traits::PrimInt;

use crate::ethercat::main::Adapter;

pub mod nicdrv;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Network<Int: PrimInt>(Int);

impl<Int: PrimInt> Network<Int> {
    pub const fn from_raw(value: Int) -> Self {
        Self(value)
    }

    pub const fn into_inner(self) -> Int {
        self.0
    }
}

/// Host to network byte order (i.e. to big endian)
///
/// Not that Ethercat uses little endian byte order, except for the Ethernet
/// header which is big endian as usual
pub fn host_to_network<Int: PrimInt>(host_short: Int) -> Network<Int> {
    Network(host_short.to_be())
}

/// Network (i.e. big endian) to host byte order.
///
/// Note that Ethercat uses little endian byte order, except for the Ethernet
/// header which is big endian as usual
pub fn network_to_host<Int: PrimInt>(network_short: Network<Int>) -> Int {
    if cfg!(target_endian = "little") {
        network_short.0.to_le()
    } else {
        network_short.0
    }
}

/// Create list over available network adapters.
///
/// # Panics
/// Panics if the name of an interface is too long
///
/// # Returns
/// First element in linked list of adapters
pub fn find_adaters() -> Vec<Adapter> {
    // Iterate all devices and create a local copy holding the name and description
    let ids = unsafe { if_nameindex() };
    (0..usize::MAX)
        .filter_map(|i| unsafe { ids.add(i).as_mut() })
        .take_while(|id| id.if_index != 0)
        .map(|id| {
            // Fetch description and name, in Linux they are the same.
            let name =
                heapless::String::from_str(unsafe { CStr::from_ptr(id.if_name) }.to_str().unwrap())
                    .unwrap();
            let desc = name.clone();
            Adapter { name, desc }
        })
        .collect()
}
