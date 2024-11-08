//! Parent module for all EtherCAT modules

use std::io::{self, Read};

pub mod base;
pub mod coe;
pub mod config;
pub mod dc;
pub mod eoe;
pub mod foe;
pub mod main;
pub mod soe;
pub mod r#type;

trait ReadFrom<R: Read> {
    type Err;

    /// # Errors
    /// Returns an error if:
    /// - The reader didn't contain enough bytes
    /// - An unexpected byte was read (invalid value)
    fn read_from(reader: &mut R) -> Result<Self, Self::Err>
    where
        Self: Sized;

    /// # Errors
    /// Returns an error on failure to read the requested number of bytes
    fn read_bytes<const LENGTH: usize>(reader: &mut R) -> io::Result<[u8; LENGTH]> {
        let mut bytes = [0; LENGTH];
        reader.read_exact(&mut bytes)?;
        Ok(bytes)
    }

    /// # Errors
    /// Returns an error on failure to read a byte of data
    fn read_byte(reader: &mut R) -> io::Result<u8> {
        Ok(Self::read_bytes::<1>(reader)?[0])
    }
}
