//! Parent module for low level operations.

use num_traits::PrimInt;

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
