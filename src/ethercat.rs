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

pub trait ReadFrom<R: Read>: Sized {
    type Err;

    /// # Errors
    /// Returns an error if:
    /// - The reader didn't contain enough bytes
    /// - An unexpected byte was read (invalid value)
    fn read_from(reader: &mut R) -> Result<Self, Self::Err>
    where
        Self: Sized;
}

impl<const SIZE: usize, R: Read> ReadFrom<R> for [u8; SIZE] {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let mut data = [0; SIZE];
        reader.read_exact(&mut data)?;
        Ok(data)
    }
}

impl<R: Read> ReadFrom<R> for u8 {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        <[Self; 1]>::read_from(reader).map(|data| data[0])
    }
}

trait WriteTo<W> {
    fn write_to(&self, writer: &mut W) -> io::Result<()>;
}
