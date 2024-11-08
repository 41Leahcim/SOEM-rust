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

    /// Host to network byte order (i.e. to big endian)
    ///
    /// Not that Ethercat uses little endian byte order, except for the Ethernet
    /// header which is big endian as usual
    pub fn from_host(value: Int) -> Self {
        Self(value.to_be())
    }

    /// Network (i.e. big endian) to host byte order.
    ///
    /// Note that Ethercat uses little endian byte order, except for the Ethernet
    /// header which is big endian as usual
    pub fn to_host(self) -> Int {
        if cfg!(target_endian = "little") {
            self.0.to_le()
        } else {
            self.0
        }
    }
}
