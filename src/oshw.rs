//! Parent module for low level operations.

use num_traits::PrimInt;

pub mod nicdrv;

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
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

macro_rules! network_bytes {
    ($data_type: ident) => {
        impl Network<$data_type> {
            pub const fn to_bytes(self) -> [u8; size_of::<$data_type>()] {
                self.0.to_ne_bytes()
            }

            pub const fn from_bytes(bytes: [u8; size_of::<$data_type>()]) -> Self {
                Self::from_raw($data_type::from_ne_bytes(bytes))
            }
        }
    };
}

network_bytes!(u16);
network_bytes!(u32);
network_bytes!(i32);
network_bytes!(i64);
network_bytes!(u64);

#[cfg(test)]
mod tests {
    use crate::oshw::Network;

    #[test]
    fn from_host_to_host() {
        assert_eq!(Network::<u16>::from_host(0x1234).to_host(), 0x1234);
    }
}
