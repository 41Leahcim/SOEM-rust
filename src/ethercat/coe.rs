//! CAT over EtherCAT module.
//!
//! SDO read/write and SDO service functions.

use core::str;
use std::io;
use std::io::Read;
use std::io::Write;
use std::num::TryFromIntError;
use std::str::Utf8Error;
use std::time::Duration;

use heapless::String as HeaplessString;

use crate::ethercat::main::SyncManagerType;
use crate::ethercat::main::{MAX_SM, SYNC_MANAGER_ENABLE_MASK};
use crate::ethercat::r#type::{
    low_byte, mailbox_header_set_count, COEMailboxType, CanopenOverEthercatSdoCommand, MailboxType,
    SDO_PDO_ASSIGNMENT, SDO_SMCOMMTYPE, TIMEOUT_TX_MAILBOX,
};

use super::main::PdoAssign;
use super::main::PdoDescription;
use super::main::{
    Context, MailboxHeader, MailboxIn, MailboxOut, MainError, PacketError,
    SyncManagerCommunicationType, MAX_NAME_LENGTH,
};
use super::r#type::{
    AbortError, COEObjectDescriptionCommand, Datatype, Ethercat, InvalidCommand, InvalidDataType,
    TIMEOUT_RX_MAILBOX,
};
use super::ReadFrom;
use super::WriteTo;

#[derive(Debug)]
#[must_use]
pub enum CoEError {
    InvalidSDOSize,
    InvalidOpcode(InvalidCommand),
    Mailbox(MainError),
    Packet(PacketError),
    Abort(AbortError),
    TryFromInt(TryFromIntError),
    InvalidDataType(InvalidDataType),
    Utf8(Utf8Error),
    Io(io::Error),
    NotEnoughSyncManagers,
}

impl From<InvalidCommand> for CoEError {
    fn from(value: InvalidCommand) -> Self {
        Self::InvalidOpcode(value)
    }
}

impl From<io::Error> for CoEError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<MainError> for CoEError {
    fn from(value: MainError) -> Self {
        Self::Mailbox(value)
    }
}

impl From<PacketError> for CoEError {
    fn from(value: PacketError) -> Self {
        Self::Packet(value)
    }
}

impl From<AbortError> for CoEError {
    fn from(value: AbortError) -> Self {
        Self::Abort(value)
    }
}

impl From<TryFromIntError> for CoEError {
    fn from(value: TryFromIntError) -> Self {
        Self::TryFromInt(value)
    }
}

impl From<InvalidDataType> for CoEError {
    fn from(value: InvalidDataType) -> Self {
        Self::InvalidDataType(value)
    }
}

impl From<Utf8Error> for CoEError {
    fn from(value: Utf8Error) -> Self {
        Self::Utf8(value)
    }
}

/// Variants for easy data access
#[derive(Debug)]
enum ServiceData {
    Byte([u8; 0x200]),
    Word([u16; 0x100]),
    Long([u32; 0x80]),
}

impl Default for ServiceData {
    fn default() -> Self {
        Self::Byte([0; 0x200])
    }
}

impl ServiceData {
    pub const fn size() -> usize {
        size_of::<[u8; 512]>()
    }

    pub fn as_bytes(&self) -> &[u8; 512] {
        match self {
            Self::Byte(bytes) => bytes,
            Self::Word(words) => bytemuck::cast_ref(words),
            Self::Long(longs) => bytemuck::cast_ref(longs),
        }
    }

    pub fn as_words(&self) -> &[u16; 256] {
        match self {
            Self::Byte(bytes) => bytemuck::cast_ref(bytes),
            Self::Word(words) => words,
            Self::Long(longs) => bytemuck::cast_ref(longs),
        }
    }

    pub fn as_longs(&self) -> &[u32; 128] {
        match self {
            Self::Byte(bytes) => bytemuck::cast_ref(bytes),
            Self::Word(words) => bytemuck::cast_ref(words),
            Self::Long(longs) => longs,
        }
    }

    pub fn as_bytes_mut(&mut self) -> &mut [u8; 512] {
        match self {
            Self::Byte(bytes) => bytes,
            Self::Word(words) => bytemuck::cast_mut(words),
            Self::Long(longs) => bytemuck::cast_mut(longs),
        }
    }

    pub fn as_words_mut(&mut self) -> &mut [u16; 256] {
        match self {
            Self::Byte(bytes) => bytemuck::cast_mut(bytes),
            Self::Word(words) => words,
            Self::Long(longs) => bytemuck::cast_mut(longs),
        }
    }

    pub fn as_longs_mut(&mut self) -> &mut [u32; 128] {
        match self {
            Self::Byte(bytes) => bytemuck::cast_mut(bytes),
            Self::Word(words) => bytemuck::cast_mut(words),
            Self::Long(longs) => longs,
        }
    }

    pub fn set_word_compile_time_checked<const INDEX: usize>(&mut self, value: Ethercat<u16>) {
        self.as_words_mut()[INDEX] = value.into_inner();
    }

    pub fn set_long_compile_time_checked<const INDEX: usize>(&mut self, value: Ethercat<u32>) {
        self.as_longs_mut()[INDEX] = value.into_inner();
    }

    pub fn get_word_compile_time_checked<const INDEX: usize>(&self) -> Ethercat<u16> {
        Ethercat::from_raw(self.as_words()[INDEX])
    }

    pub fn get_long_compile_time_checked<const INDEX: usize>(&self) -> Ethercat<u32> {
        Ethercat::from_raw(self.as_longs()[INDEX])
    }
}

/// Service Data Object structure, not to be confused with `ServiceDataObjectService`

#[derive(Debug, Default)]
pub struct ServiceDataObject {
    mailbox_header: MailboxHeader,
    can_open: Ethercat<u16>,
    command: u8,
    index: Ethercat<u16>,
    sub_index: u8,
    data: ServiceData,
}

impl<R: Read> ReadFrom<R> for ServiceDataObject {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let mailbox_header = MailboxHeader::read_from(reader).unwrap();
        let can_open = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let command = u8::read_from(reader)?;
        let index = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let sub_index = u8::read_from(reader)?;
        let data = ServiceData::Byte(<[u8; 512]>::read_from(reader)?);
        Ok(Self {
            mailbox_header,
            can_open,
            command,
            index,
            sub_index,
            data,
        })
    }
}

impl<W: Write> WriteTo<W> for ServiceDataObject {
    fn write_to(&self, writer: &mut W) -> io::Result<()> {
        self.mailbox_header.write_to(writer)?;
        writer.write_all(&self.can_open.to_bytes())?;
        writer.write_all(&[self.command])?;
        writer.write_all(&self.index.to_bytes())?;
        writer.write_all(&[self.sub_index])?;
        writer.write_all(self.data.as_bytes())?;
        Ok(())
    }
}

impl ServiceDataObject {
    fn read_from_index<R: Read>(&mut self, reader: &mut R) -> io::Result<()> {
        self.index = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        self.sub_index = u8::read_from(reader)?;
        self.data = ServiceData::Byte(<[u8; 512]>::read_from(reader)?);
        Ok(())
    }

    fn to_bytes(&self) -> [u8; size_of::<MailboxHeader>() + 6 + 512] {
        let mut bytes = [0; size_of::<MailboxHeader>() + 6 + 512];
        self.write_to(&mut bytes.as_mut_slice()).unwrap();
        bytes
    }

    const fn mailbox_header_offset() -> usize {
        0
    }

    const fn can_open_offset() -> usize {
        Self::mailbox_header_offset() + size_of::<MailboxHeader>()
    }

    const fn command_offset() -> usize {
        Self::can_open_offset() + size_of::<u16>()
    }

    const fn index_offset() -> usize {
        Self::command_offset() + size_of::<u8>()
    }

    fn segmented_read<'parameters>(
        context: &mut Context,
        slave: u16,
        frame_data_size: usize,
        index: u16,
        sub_index: u8,
        parameter_buffer: &'parameters mut [u8],
        timeout: Duration,
    ) -> Result<&'parameters mut [u8], CoEError> {
        // Increment the buffer pointer
        let mut hp = frame_data_size;
        let mut written = frame_data_size;
        let mut segments_left = true;
        let mut toggle = 0;
        let mut mailbox_out = MailboxOut::default();
        while segments_left {
            let sdo = Self {
                mailbox_header: MailboxHeader::new(
                    Ethercat::from_host(0xA),
                    Ethercat::default(),
                    0,
                    u8::from(MailboxType::CanopenOverEthercat)
                        + mailbox_header_set_count(
                            context.get_slave_mut(slave).mailbox.next_count(),
                        ),
                ),
                can_open: Ethercat::from_host(u16::from(COEMailboxType::SdoRequest) << 12),
                // Segment upload request
                command: u8::from(CanopenOverEthercatSdoCommand::SegUpReq) + toggle,
                index: Ethercat::from_host(index),
                sub_index,
                data: ServiceData::Long([0; 128]),
            };

            // Send segmented upload request to slave
            sdo.write_to(&mut mailbox_out)?;
            if mailbox_out.send(context, slave, timeout).is_ok() {
                toggle ^= 0x10;
                continue;
            }

            // Read slave response
            let mut mailbox_in = MailboxIn::default();
            if mailbox_in.receive(context, slave, timeout).is_err() {
                toggle ^= 0x10;
                continue;
            }
            let a_sdo = Self::read_from(&mut mailbox_in)?;

            // Slave response should be CANopen over EtherCAT, SDO response
            if a_sdo.mailbox_header.mailbox_type & 0xF != u8::from(MailboxType::CanopenOverEthercat)
                || (a_sdo.can_open.to_host() >> 12) as u8 != u8::from(COEMailboxType::SdoResponse)
                || a_sdo.command & 0xE0 != 0
            {
                // Unexpected frame returned from slave
                return Err(handle_invalid_slave_response(
                    &a_sdo, context, slave, index, sub_index,
                ));
            }
            // Calculate mailbox transfer size
            let mut frame_data_size = a_sdo.mailbox_header.length.to_host() - 3;

            // Check if this is the last segment
            if a_sdo.command & 1 > 0 {
                // Last segment
                segments_left = false;
                if frame_data_size == 7 {
                    // Subtract unused bytes from frame
                    frame_data_size -= u16::from((a_sdo.command & 0xE) >> 1);
                }

                // Copy to parameter buffer
                let index_offset = Self::index_offset();
                parameter_buffer[hp..].copy_from_slice(
                    &a_sdo.to_bytes()[index_offset..index_offset + usize::from(frame_data_size)],
                );
            } else {
                // Segments follow

                // Copy to parameter buffer
                let index_offset = Self::index_offset();
                parameter_buffer[hp..].copy_from_slice(
                    &a_sdo.to_bytes()[index_offset..index_offset + usize::from(frame_data_size)],
                );

                // Increment buffer pointer
                hp += usize::from(frame_data_size);
            }

            // Update parameter size
            written += usize::from(frame_data_size);

            toggle ^= 0x10;
        }
        Ok(&mut parameter_buffer[..written])
    }

    /// CANopen over EtherCAT Service Data Object read (blocking). Single subindex or complete access.
    ///
    /// Only a "normal" upload request is issued. If the requested parameter <= 4 bytes,
    /// an "expedited" response is returned. Otherwise, a "normal" response. If a "normal"
    /// response is larger than the mailbox size, the response is segmented. The function
    /// will combine all segments and copy them to the parameter buffer.
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `slave`: Slave number
    /// - `index`: Index to read
    /// - `subindex`: Subindex to read, must be 0 or 1 if CA is used
    /// - `ca`: False for single subindex, true for complete access, all subindexes read
    /// - `parameter_buffer`: Reference to parameter buffer
    /// - `timeout`: Timeout duration, standard is `TIMEOUT_RX_MAILBOX`
    ///
    /// # Errors
    /// Returns an error if:
    /// - A message couldn't be send/received.
    /// - A response couldn't be parsed into an object.
    ///
    /// # Returns
    /// Bytes read into `parameter_buffer` or error
    pub fn read(
        context: &mut Context,
        slave: u16,
        index: u16,
        sub_index: u8,
        complete_access: bool,
        mut parameter_buffer: &mut [u8],
        timeout: Duration,
    ) -> Result<usize, CoEError> {
        let mut mailbox_in = MailboxIn::default();

        // Empty slave out mailbox, if something is in it. Timeout set to 0.
        let _ = mailbox_in.receive(context, slave, Duration::default());
        let sdo = Self {
            mailbox_header: MailboxHeader::new(
                Ethercat::from_host(0xa),
                Ethercat::default(),
                0,
                // Get new mailbox count value, used as session handle
                u8::from(MailboxType::CanopenOverEthercat)
                    + mailbox_header_set_count(context.get_slave_mut(slave).mailbox.next_count()),
            ),
            can_open: Ethercat::from_host(u16::from(COEMailboxType::SdoRequest) << 12),
            command: if complete_access {
                // Upload request complete access
                CanopenOverEthercatSdoCommand::UpReqCa.into()
            } else {
                // Upload request normal
                CanopenOverEthercatSdoCommand::UpReq.into()
            },
            index: Ethercat::from_host(index),
            sub_index: if complete_access {
                sub_index.min(1)
            } else {
                sub_index
            },
            data: ServiceData::Long([0; 128]),
        };

        // Send CANopen over EtherCAT request to slave
        let mut mailbox_out = MailboxOut::default();
        sdo.write_to(&mut mailbox_out)?;
        mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

        // Clean mailbox buffer
        mailbox_in.clear();

        // Read slave response
        mailbox_in.receive(context, slave, timeout)?;
        let a_sdo = Self::read_from(&mut mailbox_in)?;

        // Slave response should be CANopen over EtherCAT, a Service Data Object response, and use the correct index
        if a_sdo.mailbox_header.mailbox_type & 0xF != u8::from(MailboxType::CanopenOverEthercat)
            || (a_sdo.can_open.to_host() >> 12) as u8 != u8::from(COEMailboxType::SdoResponse)
            || a_sdo.index != sdo.index
        {
            // Other slave response
            // SDO abort frame received
            return Err(handle_invalid_slave_response(
                &a_sdo, context, slave, index, sub_index,
            ));
        }
        if a_sdo.command & 0x2 > 0 {
            let bytesize = 4 - a_sdo.command / 4 % 4;

            // Check if the parameter in the parameter buffer is big enough
            if parameter_buffer.len() >= usize::from(bytesize) {
                parameter_buffer.copy_from_slice(&a_sdo.data.as_bytes()[..usize::from(bytesize)]);
                parameter_buffer = &mut parameter_buffer[..usize::from(bytesize)];
            } else {
                // Data container too small for type
                context.packet_error(
                    slave,
                    index,
                    sub_index,
                    PacketError::DataContainerTooSmallForType,
                );
                return Err(PacketError::DataContainerTooSmallForType.into());
            }
        } else {
            // Normal frame response
            let sdo_len = a_sdo.data.get_long_compile_time_checked::<0>().to_host() as usize;

            // Check whether the parameter fits in the parameter buffer
            if sdo_len > parameter_buffer.len() {
                // Data container too small for type
                context.packet_error(
                    slave,
                    index,
                    sub_index,
                    PacketError::DataContainerTooSmallForType,
                );
                return Err(PacketError::DataContainerTooSmallForType.into());
            }

            // Calculate mailblox transfer size
            let frame_data_size = usize::from(a_sdo.mailbox_header.length.to_host() - 10);

            // Check whether the transfer is segmented
            if frame_data_size < sdo_len {
                parameter_buffer.copy_from_slice(
                    &a_sdo.data.as_bytes()[size_of::<u32>()..frame_data_size + size_of::<u32>()],
                );

                parameter_buffer = Self::segmented_read(
                    context,
                    slave,
                    frame_data_size,
                    index,
                    sub_index,
                    parameter_buffer,
                    timeout,
                )?;
            } else {
                // Non segmented transfer
                parameter_buffer.copy_from_slice(
                    &a_sdo.data.as_bytes()[size_of::<u32>()..sdo_len + size_of::<u32>()],
                );
                parameter_buffer = &mut parameter_buffer[..sdo_len];
            }
        }

        Ok(parameter_buffer.len())
    }

    fn read_bytes<const SIZE: usize>(
        context: &mut Context,
        slave: u16,
        index: u16,
        sub_index: u8,
        timeout: Duration,
    ) -> Result<[u8; SIZE], CoEError> {
        let mut data = [0; SIZE];
        Self::read(context, slave, index, sub_index, false, &mut data, timeout)?;
        Ok(data)
    }

    #[expect(clippy::too_many_arguments)]
    fn write_segments(
        context: &mut Context,
        mut parameter_buffer: &mut [u8],
        mut max_data: u16,
        complete_access: bool,
        index: u16,
        sub_index: u8,
        slave: u16,
        timeout: Duration,
    ) -> Result<(), CoEError> {
        let (frame_data_size, mut segments_left) = if parameter_buffer.len() > max_data.into() {
            (max_data, true)
        } else {
            (parameter_buffer.len() as u16, false)
        };
        let mut data = ServiceData::Long([0; 128]);
        data.set_long_compile_time_checked::<0>(Ethercat::from_host(parameter_buffer.len() as u32));

        // Copy data to mailbox
        data.as_bytes_mut()[size_of::<u32>()..]
            .copy_from_slice(&parameter_buffer[..usize::from(frame_data_size)]);
        parameter_buffer = &mut parameter_buffer[usize::from(frame_data_size)..];

        let mut sdo = Self {
            mailbox_header: MailboxHeader::new(
                Ethercat::from_host(0xA + frame_data_size),
                Ethercat::default(),
                0,
                // Get new mailbox counter, used for session handle
                u8::from(MailboxType::CanopenOverEthercat)
                    + mailbox_header_set_count(context.get_slave_mut(slave).mailbox.next_count()),
            ),
            can_open: Ethercat::from_host(u16::from(COEMailboxType::SdoRequest) << 12),
            command: if complete_access {
                // Complete access, normal Service Data Object init download transfer
                CanopenOverEthercatSdoCommand::DownInitCa
            } else {
                // Normal Service Data Object init download transfer
                CanopenOverEthercatSdoCommand::DownInit
            }
            .into(),
            index: Ethercat::from_host(index),
            sub_index: if complete_access {
                sub_index.min(1)
            } else {
                sub_index
            },
            data,
        };

        // Send mailbox SDO download request to slave
        let mut mailbox_out = MailboxOut::default();
        sdo.write_to(&mut mailbox_out)?;
        mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

        // Read slave response
        let mut mailbox_in = MailboxIn::default();
        mailbox_in.receive(context, slave, timeout)?;
        let a_sdo = Self::read_from(&mut mailbox_in)?;

        // Response should be a SDO response send over CANopen over EtherCAT, containing the correct index and subindex
        if a_sdo.mailbox_header.mailbox_type & 0xF != u8::from(MailboxType::CanopenOverEthercat)
            || (a_sdo.can_open.to_host() >> 12) as u8 != u8::from(COEMailboxType::SdoResponse)
            || a_sdo.index != sdo.index
            || a_sdo.sub_index != sdo.sub_index
        {
            return Err(handle_invalid_slave_response(
                &a_sdo, context, slave, index, sub_index,
            ));
        }
        // All ok
        max_data += 7;
        let mut toggle = 0;

        // Repeat while there are segments left
        while segments_left {
            // Last segment
            (segments_left, sdo.command) = if frame_data_size > max_data {
                sdo.command = 0;
                (true, 0)
            } else {
                (false, 1)
            };
            sdo.mailbox_header.length = if segments_left && frame_data_size >= 7 {
                Ethercat::from_host(frame_data_size + 3)
            } else {
                sdo.command = (1 + ((7 - frame_data_size) << 1)) as u8;
                Ethercat::from_host(0xA)
            };

            *sdo.mailbox_header.address_mut() = Ethercat::default();
            *sdo.mailbox_header.priority_mut() = 0;

            // Get new mailbox counter value

            sdo.mailbox_header.mailbox_type = u8::from(MailboxType::CanopenOverEthercat)
                + mailbox_header_set_count(context.get_slave_mut(slave).mailbox.next_count());

            // Service data object request
            sdo.can_open = Ethercat::from_host(u16::from(COEMailboxType::SdoRequest) << 12);

            // Add toggle to command byte
            sdo.command += toggle;

            // Copy parameter data to mailbox
            sdo.read_from_index(&mut &parameter_buffer[..usize::from(frame_data_size)])?;
            parameter_buffer = &mut parameter_buffer[usize::from(frame_data_size)..];

            // Send Service Data Object download request
            sdo.write_to(&mut mailbox_out)?;
            if mailbox_out.send(context, slave, timeout).is_err() {
                toggle ^= 0x10;
                continue;
            }

            mailbox_in.clear();

            // Read slave response
            if mailbox_in.receive(context, slave, timeout).is_err() {
                toggle ^= 0x10;
                continue;
            }
            sdo = Self::read_from(&mut mailbox_in)?;
            if a_sdo.mailbox_header.mailbox_type & 0xF != u8::from(MailboxType::CanopenOverEthercat)
                || (a_sdo.can_open.to_host() >> 12) as u8 != u8::from(COEMailboxType::SdoResponse)
                || a_sdo.command & 0xE0 != 0x20
            {
                return Err(handle_invalid_slave_response(
                    &a_sdo, context, slave, index, sub_index,
                ));
            }
            toggle ^= 0x10;
        }
        Ok(())
    }

    /// CANopen over EtherCAT Service Dta Object write, blocking. Single subindex or complete access.
    ///
    /// A "normal" download request is issued, unless we have small data,
    /// then an "expedited" transfer is used. If the parameter is larger than
    /// the mailbox size, the download is segemented. The function will split the
    /// parameter data in segments and send them to the slave one by one.
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `slave`: Slave number
    /// - `index`: Index to write
    /// - `sub_index`: Subindex to write, must be 0 or 1 if complete access is used.
    /// - `comlete_access`: False for single subindex, true for complete access, all subindexes written
    /// - `parameter_buffer`: Timeout delay, standard is `TIMEOUT_RX_MAILBOX`
    ///
    /// # Errors
    /// Returns an error if:
    /// - A message couldn't be send/received
    /// - Received data couldn't be parsed into an object
    ///
    /// # Returns
    /// Unit or error
    pub fn write(
        context: &mut Context,
        slave: u16,
        index: u16,
        sub_index: u8,
        complete_access: bool,
        parameter_buffer: &mut [u8],
        timeout: Duration,
    ) -> Result<(), CoEError> {
        // Empty slave out mailbox if something is in it. With timeout set to default.
        let mut mailbox_in = MailboxIn::default();
        let _ = mailbox_in.receive(context, slave, Duration::default());

        // Data section = mailbox size - 6 mailbox - 2 CANopen over EtherCAT - 8 service data object requests
        let max_data = context.get_slave_mut(slave).mailbox.length - 0x10;

        // For small data use expedited transfer
        if parameter_buffer.len() <= 4 && !complete_access {
            // Copy parameter data to mailbox
            let mut data = ServiceData::Long([0; 128]);
            data.as_bytes_mut().copy_from_slice(parameter_buffer);

            let sdo = Self {
                mailbox_header: MailboxHeader::new(
                    Ethercat::from_host(0xA),
                    Ethercat::default(),
                    0,
                    // Get new mailbox counter, used for session handle
                    u8::from(MailboxType::CanopenOverEthercat)
                        + mailbox_header_set_count(
                            context.get_slave_mut(slave).mailbox.next_count(),
                        ),
                ),
                can_open: Ethercat::from_host(u16::from(COEMailboxType::SdoRequest) << 12),
                command: u8::from(CanopenOverEthercatSdoCommand::DownExp)
                    | (((4 - parameter_buffer.len()) << 2) & 0xC) as u8,
                index: Ethercat::from_host(index),
                sub_index,
                data,
            };

            // Send mailbox SDO download request to slave
            let mut mailbox_out = MailboxOut::default();
            sdo.write_to(&mut mailbox_out)?;
            mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

            mailbox_in.clear();

            // Read slave response
            mailbox_in.receive(context, slave, timeout)?;
            let a_sdo = Self::read_from(&mut mailbox_in)?;

            if a_sdo.mailbox_header.mailbox_type & 0xF != u8::from(MailboxType::CanopenOverEthercat)
                || (a_sdo.can_open.to_host() >> 12) as u8 != u8::from(COEMailboxType::SdoResponse)
                || a_sdo.index != sdo.index
                || a_sdo.sub_index != sdo.sub_index
            {
                return Err(handle_invalid_slave_response(
                    &a_sdo, context, slave, index, sub_index,
                ));
            }
            Ok(())
        } else {
            Self::write_segments(
                context,
                parameter_buffer,
                max_data,
                complete_access,
                index,
                sub_index,
                slave,
                timeout,
            )
        }
    }
}

/// Service Data Object service structure
struct ServiceDataObjectService {
    mailbox_header: MailboxHeader,
    can_open: Ethercat<u16>,
    opcode: COEObjectDescriptionCommand,
    reserved: u8,
    fragments: Ethercat<u16>,
    data: ServiceData,
}

impl Default for ServiceDataObjectService {
    fn default() -> Self {
        Self {
            mailbox_header: MailboxHeader::default(),
            can_open: Ethercat::default(),
            opcode: COEObjectDescriptionCommand::default(),
            reserved: 0,
            fragments: Ethercat::default(),
            data: ServiceData::Byte([0; 512]),
        }
    }
}

impl<R: Read> ReadFrom<R> for ServiceDataObjectService {
    type Err = CoEError;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let mailbox_header = MailboxHeader::read_from(reader)?;
        let can_open = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let opcode = COEObjectDescriptionCommand::try_from(u8::read_from(reader)?)?;
        let reserved = u8::read_from(reader)?;
        let fragments = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let data = ServiceData::Byte(<[u8; 512]>::read_from(reader)?);
        Ok(Self {
            mailbox_header,
            can_open,
            opcode,
            reserved,
            fragments,
            data,
        })
    }
}

impl<W: Write> WriteTo<W> for ServiceDataObjectService {
    fn write_to(&self, bytes: &mut W) -> io::Result<()> {
        self.mailbox_header.write_to(bytes)?;
        bytes.write_all(&self.can_open.to_bytes())?;
        bytes.write_all(&[u8::from(self.opcode), self.reserved])?;
        bytes.write_all(&self.fragments.to_bytes())?;
        bytes.write_all(self.data.as_bytes())
    }
}

impl ServiceDataObjectService {
    pub const fn size() -> usize {
        MailboxHeader::size()
            + 2 * size_of::<u16>()
            + COEObjectDescriptionCommand::size()
            + size_of::<u8>()
            + ServiceData::size()
    }

    pub fn bytes(&self) -> [u8; Self::size()] {
        let mut result = [0; Self::size()];
        self.write_to(&mut result.as_mut_slice()).unwrap();
        result
    }
}

/// Max entries in object description list
pub const MAX_OBJECT_DESCRIPTION_LIST_SIZE: u16 = 1024;

/// Max entries in Object Entries list
pub const MAX_OBJECT_ENTRY_LIST_SIZE: usize = 256;

#[derive(Debug, Default)]
pub struct ObjectDescription {
    index: u16,
    data_type: Datatype,
    object_code: u16,
    max_sub: u8,
    name: HeaplessString<{ MAX_NAME_LENGTH as usize }>,
}

impl ObjectDescription {
    pub const fn index(&self) -> u16 {
        self.index
    }

    pub const fn data_type(&self) -> Datatype {
        self.data_type
    }

    pub const fn object_code(&self) -> u16 {
        self.object_code
    }

    pub const fn max_sub(&self) -> u8 {
        self.max_sub
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// CANopen over EtherCAT read Object Description. Adds textual information to object indexes.
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `item`: Item number in Object  Description List
    /// - `od_list`: Referencing Object Description list
    ///
    /// # Panics
    /// Panics if the name of an object entry doesn't fit in the string.
    ///
    /// # Errors
    /// Returns an error if:
    /// - A message couldn't be send/received
    /// - Received data couldn't be parsed into an object
    ///
    /// # Returns
    /// Unit or error
    pub fn read(context: &mut Context, slave: u16, index: u16) -> Result<Self, CoEError> {
        let mut mailbox_in = MailboxIn::default();

        // Clear pending out mailbox in slave if available with timeout set to default.
        let _ = mailbox_in.receive(context, slave, Duration::default());

        let mut data = ServiceData::Word([0; 256]);
        // Data of index
        data.set_word_compile_time_checked::<0>(Ethercat::from_host(index));

        let sdo = ServiceDataObjectService {
            mailbox_header: MailboxHeader::new(
                Ethercat::from_host(8),
                Ethercat::default(),
                0,
                u8::from(MailboxType::CanopenOverEthercat)
        // Get new mailbox counter value
        + mailbox_header_set_count(context.get_slave_mut(slave).mailbox.next_count()),
            ),
            can_open: Ethercat::from_host(u16::from(COEMailboxType::SdoInfo) << 12),
            // Get object description request
            opcode: COEObjectDescriptionCommand::ObjectDesciptionRequest,
            reserved: 0,
            // Fragments left
            fragments: Ethercat::default(),
            data,
        };

        // Send get request to slave
        let mut mailbox_out = MailboxOut::default();
        sdo.write_to(&mut mailbox_out)?;
        mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

        // Read slave response
        mailbox_in.clear();
        mailbox_in.receive(context, slave, TIMEOUT_RX_MAILBOX)?;

        let a_sdo = ServiceDataObjectService::read_from(&mut mailbox_in)?;

        if a_sdo.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::CanopenOverEthercat)
            && a_sdo.opcode == COEObjectDescriptionCommand::ObjectDesciptionResponse
        {
            let object_name_length =
                (a_sdo.mailbox_header.length.to_host() - 12).min(MAX_NAME_LENGTH);
            Ok(Self {
                data_type: Datatype::try_from(u8::try_from(
                    a_sdo.data.get_word_compile_time_checked::<1>().to_host(),
                )?)?,
                object_code: u16::from(a_sdo.data.as_bytes()[5]),
                max_sub: a_sdo.data.as_bytes()[4],
                name: HeaplessString::from_utf8(
                    heapless::Vec::from_slice(
                        &a_sdo.data.as_bytes()[..usize::from(object_name_length)],
                    )
                    .unwrap(),
                )
                .unwrap(),
                index,
            })
        } else {
            // Got unexpected response from slave

            // SDO info error received
            if a_sdo.opcode == COEObjectDescriptionCommand::ServiceDataObjectInformationError {
                let abort_error = AbortError::Abort(
                    a_sdo.data.get_long_compile_time_checked::<0>().to_host() as i32,
                );
                context.sdo_info_error(slave, index, 0, abort_error);
                return Err(abort_error.into());
            }
            context.packet_error(slave, index, 0, PacketError::UnexpectedFrameReturned);
            Err(PacketError::UnexpectedFrameReturned.into())
        }
    }
}

/// Storage for object description list
#[derive(Debug)]
pub struct ObjectDescriptionList {
    /// Slave number
    slave: u16,

    /// Number of entries in list
    entries: heapless::Vec<ObjectDescription, { MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize }>,
}

impl ObjectDescriptionList {
    pub const fn slave(&self) -> u16 {
        self.slave
    }

    pub fn get_entry(&self, index: u16) -> &ObjectDescription {
        &self.entries[usize::from(index)]
    }

    /// CANopen over EtherCAT read Object Description List
    ///
    /// # Parameters
    /// - `context`: context struct
    /// - `slave`: slave number
    ///
    /// # Errors
    /// Returns an error if:
    /// - A message couldn't be send/received
    ///
    /// # Returns
    /// Unit or error
    #[expect(clippy::missing_panics_doc, reason = "List size handled manually")]
    pub fn read(context: &mut Context, slave: u16) -> Result<Self, CoEError> {
        let mut mailbox_in = MailboxIn::default();

        // Clear pending out mailbox in slave if available with timeout set to 0.
        let _ = mailbox_in.receive(context, slave, Duration::default());

        let mut data = ServiceData::Word([0; 256]);
        // All objects
        data.set_word_compile_time_checked::<0>(Ethercat::from_host(1));

        let sdo = ServiceDataObjectService {
            mailbox_header: MailboxHeader::new(
                Ethercat::from_host(8),
                Ethercat::default(),
                0,
                u8::from(MailboxType::CanopenOverEthercat)
        // Get new mailbox counter value
        + mailbox_header_set_count(context.get_slave_mut(slave).mailbox.next_count()),
            ),
            can_open: Ethercat::from_host(u16::from(COEMailboxType::SdoInfo) << 12),
            // Get object description list request
            opcode: COEObjectDescriptionCommand::ObjectDesciptionListRequest,
            reserved: 0,
            // Fragments left
            fragments: Ethercat::default(),
            data,
        };

        // Send get object description list request to slave
        let mut mailbox_out = MailboxOut::default();
        sdo.write_to(&mut mailbox_out)?;
        mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

        let mut first = true;
        let mut entries = heapless::Vec::new();
        let mut offset = 1;
        let mut counter = 0;
        let mut result: Result<(), CoEError> = Ok(());
        loop {
            // Assume this is the last iteration
            let mut stop = true;
            mailbox_in.clear();

            // Read slave response
            if mailbox_in
                .receive(context, slave, TIMEOUT_RX_MAILBOX)
                .is_err()
            {
                counter += 1;
            }
            let Ok(a_sdo) = ServiceDataObjectService::read_from(&mut mailbox_in) else {
                counter += 1;
                continue;
            };

            // Response should be CANopen over EtherCAT and the get object description list response
            if a_sdo.mailbox_header.mailbox_type & 0xF != u8::from(MailboxType::CanopenOverEthercat)
                || a_sdo.opcode != COEObjectDescriptionCommand::ObjectDesciptionListRequest
            {
                // Got unexpected response from slave
                if a_sdo.opcode == COEObjectDescriptionCommand::ServiceDataObjectInformationError {
                    // SDO info error received
                    let abort_error = AbortError::Abort(
                        a_sdo.data.get_long_compile_time_checked::<0>().to_host() as i32,
                    );
                    context.sdo_info_error(slave, 0, 0, abort_error);
                    return Err(abort_error.into());
                }
                context.packet_error(slave, 0, 0, PacketError::UnexpectedFrameReturned);

                counter += 21;
                continue;
            }

            let mut index_count = if first {
                // Extract number of indexes from mailbox data size
                (a_sdo.mailbox_header.length.to_host() - (6 + 2)) / 2
            } else {
                (a_sdo.mailbox_header.length.to_host() - 6) / 2
            };

            // Check if indexes fit in buffer structure
            if entries.len() as u16 + index_count > MAX_OBJECT_DESCRIPTION_LIST_SIZE {
                index_count = MAX_OBJECT_DESCRIPTION_LIST_SIZE + 1 - entries.len() as u16;
                context.sdo_info_error(slave, 0, 0, AbortError::TooManyMasterBufferEntries);
                stop = true;
                result = Err(AbortError::TooManyMasterBufferEntries.into());
            }

            // Trim to maximum number of Object Description list entries defined
            if entries.len() as u16 + index_count > MAX_OBJECT_DESCRIPTION_LIST_SIZE {
                index_count = MAX_OBJECT_DESCRIPTION_LIST_SIZE - entries.len() as u16;
            }
            a_sdo.data.as_words()[offset..]
                .iter()
                .copied()
                .take(index_count.into())
                .map(Ethercat::from_raw)
                .map(Ethercat::to_host)
                .for_each(|src| {
                    entries
                        .push(ObjectDescription {
                            index: src,
                            ..Default::default()
                        })
                        .unwrap();
                });

            // Check if more fragments will follow
            if a_sdo.fragments.to_host() > 0 {
                stop = false;
            }
            first = false;
            offset = 0;

            counter += 1;
            if counter > 128 || stop {
                break;
            }
        }

        if counter > 128 && result.is_ok() {
            Err(PacketError::UnexpectedFrameReturned.into())
        } else if let Err(err) = result {
            Err(err)
        } else {
            Ok(Self { slave, entries })
        }
    }
}

#[derive(Debug)]
pub struct ObjectEntry {
    /// Array of value info, see EtherCAT specification
    value_info: u8,

    /// Array of value data types, see EtherCAT specification
    data_type: Datatype,

    /// Array of bit lengths, see EtherCAT specification
    bit_length: u16,

    /// Array of object access bits, see EtherCAT specification
    object_access: u16,

    /// Textual description of each index
    name: HeaplessString<{ MAX_NAME_LENGTH as usize }>,
}

impl ObjectEntry {
    pub const fn value_info(&self) -> u8 {
        self.value_info
    }

    pub const fn data_type(&self) -> Datatype {
        self.data_type
    }

    pub const fn bit_length(&self) -> u16 {
        self.bit_length
    }

    pub const fn object_access(&self) -> u16 {
        self.object_access
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    /// CANopen read SDO Service object entry, single subindex.
    /// Used in `read_oe`.
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `item`: Item in Object Description List
    /// - `sub_index`: Subindex of item in Object Description List
    /// - `object_description_list`: Object Description list for reference
    /// - `object_entry_list`: Resulting object entry structure
    ///
    /// # Panics
    /// Panics if the name of an object couldn't be read.
    ///
    /// # Errors
    /// Returns an error if:
    /// - A message couldn't be send/received
    /// - Read data couldn't be parsed into an object.
    ///
    /// # Returns
    /// Unit or error
    pub fn read(
        context: &mut Context,
        item: u16,
        object_description_list: &ObjectDescriptionList,
        sub_index: u8,
    ) -> Result<Self, CoEError> {
        let slave = object_description_list.slave;
        let index = object_description_list.entries[usize::from(item)].index;
        let mut mailbox_in = MailboxIn::default();

        // Clear pending out mailbox in slave if available, with timeout set to default.
        let _ = mailbox_in.receive(context, slave, Duration::default());

        let mut data = ServiceData::Word([0; 256]);
        data.set_word_compile_time_checked::<0>(Ethercat::from_host(index));
        data.as_bytes_mut()[2] = sub_index;

        // Get access rights, object category, PDO
        data.as_bytes_mut()[3] = 1 + 2 + 4;
        let sdo = ServiceDataObjectService {
            mailbox_header: MailboxHeader::new(
                Ethercat::from_host(0xA),
                Ethercat::default(),
                0,
                u8::from(MailboxType::CanopenOverEthercat)
        // Get new mailbox counter value
        + mailbox_header_set_count(context.get_slave_mut(slave).mailbox.next_count()),
            ),
            can_open: Ethercat::from_host(u16::from(COEMailboxType::SdoInfo) << 12),
            // Get object entry description request
            opcode: COEObjectDescriptionCommand::ObjectEntryRequest,
            reserved: 0,
            // Fragments left
            fragments: Ethercat::default(),
            data,
        };

        // Send get object entry description request to slave
        let mut mailbox_out = MailboxOut::default();
        sdo.write_to(&mut mailbox_out)?;
        mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

        // Read slave response
        mailbox_in.clear();
        mailbox_in.receive(context, slave, TIMEOUT_RX_MAILBOX)?;

        let a_sdo = ServiceDataObjectService::read_from(&mut mailbox_in)?;

        if a_sdo.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::CanopenOverEthercat)
            && a_sdo.opcode == COEObjectDescriptionCommand::ObjectEntryResponse
        {
            let object_name_length =
                usize::from((a_sdo.mailbox_header.length.to_host() - 16).clamp(0, MAX_NAME_LENGTH));
            Ok(Self {
                value_info: a_sdo.data.as_bytes()[3],
                data_type: Datatype::try_from(
                    a_sdo.data.get_word_compile_time_checked::<2>().to_host() as u8,
                )?,
                bit_length: a_sdo.data.get_word_compile_time_checked::<3>().to_host(),
                object_access: a_sdo.data.get_word_compile_time_checked::<4>().to_host(),
                name: heapless::String::from_utf8(
                    heapless::Vec::from_slice(&a_sdo.data.as_bytes()[10..10 + object_name_length])
                        .unwrap(),
                )?,
            })
        } else if a_sdo.opcode == COEObjectDescriptionCommand::ServiceDataObjectInformationError {
            // SDO info error received
            let abort_error =
                AbortError::Abort(a_sdo.data.get_long_compile_time_checked::<0>().to_host() as i32);
            context.sdo_info_error(slave, index, sub_index, abort_error);
            Err(abort_error.into())
        } else {
            let error_code = PacketError::UnexpectedFrameReturned;
            context.packet_error(slave, index, sub_index, error_code);
            Err(error_code.into())
        }
    }
}

/// Storage for object list entry information
#[derive(Debug, Default)]
pub struct ObjectEntryList {
    /// Number of entries in list
    entries: heapless::Vec<ObjectEntry, MAX_OBJECT_ENTRY_LIST_SIZE>,
}

impl ObjectEntryList {
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn get(&self, index: usize) -> Option<&ObjectEntry> {
        self.entries.get(index)
    }

    fn push(&mut self, entry: ObjectEntry) -> bool {
        self.entries.push(entry).is_ok()
    }

    /// CANopen read Service Data Object Service object entry
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `item`: Item in Object Description list
    /// - `object_description_list`: Object description list for reference
    /// - `object_entry_list`: Object entry structure
    ///
    /// # Panics
    /// If the name of an object couldn't be read
    ///
    /// # Errors
    /// Returns an error if:
    /// - A message couldn't be read/received
    /// - Read data couldn't be parsed into an object
    ///
    /// # Returns Unit or workcounter
    pub fn read(
        context: &mut Context,
        item: u16,
        object_description_list: &mut ObjectDescriptionList,
    ) -> Result<Self, CoEError> {
        (0..object_description_list.entries[usize::from(item)].max_sub).try_fold(
            Self::default(),
            |mut object_entry_list, sub_index| {
                object_entry_list.push(ObjectEntry::read(
                    context,
                    item,
                    object_description_list,
                    sub_index,
                )?);
                Ok(object_entry_list)
            },
        )
    }
}

/// Stores the received abort error or an unexpected frame error
fn handle_invalid_slave_response(
    a_sdo: &ServiceDataObject,
    context: &mut Context,
    slave: u16,
    index: u16,
    sub_index: u8,
) -> CoEError {
    if a_sdo.command == CanopenOverEthercatSdoCommand::Abort.into() {
        let abort_error =
            AbortError::Abort(a_sdo.data.get_long_compile_time_checked::<0>().to_host() as i32);
        // SDO abort frame received
        context.sdo_error(slave, index, sub_index, abort_error);
        abort_error.into()
    } else {
        // Unexpected frame returned
        context.packet_error(
            slave,
            index,
            sub_index,
            PacketError::UnexpectedFrameReturned,
        );
        PacketError::UnexpectedFrameReturned.into()
    }
}

/// CANopen over EtherCAT RxPDO write, blocking
///
/// A RxPDO download request is issued.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `RxPDOnumber`: Related RxPDO buffer
/// - `pdo_buffer`: Reference to PDO buffer
///
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
///
/// # Returns
/// Unit or an error
pub fn rx_pdo(
    context: &mut Context,
    slave: u16,
    rx_pdo_number: u16,
    pdo_buffer: &[u8],
) -> Result<(), MainError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox, if something is in it. With timeout set to default.
    let _ = mailbox_in.receive(context, slave, Duration::default());

    // Data section = mailbox size - 6 mailbox - 2 CANopen over EtherCAT
    let max_data = context.get_slave(slave).mailbox.length - 8;
    let framedatasize = (pdo_buffer.len() as u16).min(max_data);
    let sdo = ServiceDataObject {
        mailbox_header: MailboxHeader::new(
            Ethercat::from_host(2 + framedatasize),
            Ethercat::default(),
            0,
            u8::from(MailboxType::CanopenOverEthercat)
            // Get new mailbox counter, used for session handle
        + mailbox_header_set_count(context.get_slave_mut(slave).mailbox.next_count()),
        ),
        can_open: Ethercat::from_host(
            (rx_pdo_number & 0x1FF) + (u16::from(COEMailboxType::RxPdo) << 12),
        ),
        ..Default::default()
    };

    // Copy PDO data to mailbox
    let command_offset = ServiceDataObject::command_offset();
    let mut mailbox_out = MailboxOut::default();
    sdo.write_to(&mut mailbox_out)?;
    mailbox_out.as_mut()[command_offset..]
        .copy_from_slice(&pdo_buffer[..usize::from(framedatasize)]);

    // Send mailbox Receive PDO request
    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;
    Ok(())
}

/// CANopen over EtherCAT TxPDO read remote request (blocking)
///
/// A TxPDO download request is issued
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `tx_pdo_number`: Related TxPDO number
/// - `pdo_buffer`: Reference to PDO buffer
/// - `timeout`: Timeout duration, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
/// - Received data couldn't be parsed into an object
///
/// # Returns
/// Bytes written to pdo or error
pub fn tx_pdo(
    context: &mut Context,
    slave: u16,
    tx_pdo_number: u16,
    pdo_buffer: &mut [u8],
    timeout: Duration,
) -> Result<usize, CoEError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave output mailbox if something is in it. With timeout set to 0.
    let _ = mailbox_in.receive(context, slave, Duration::default());
    let sdo = ServiceDataObject {
        mailbox_header: MailboxHeader::new(
            Ethercat::from_host(2),
            Ethercat::default(),
            0,
            u8::from(MailboxType::CanopenOverEthercat)
    // Get new mailbox counter, used for session handle
    + mailbox_header_set_count(context.get_slave_mut(slave).mailbox.next_count()),
        ),
        can_open: Ethercat::from_host(
            (tx_pdo_number & 0x1FF) + ((COEMailboxType::TxPdoRR as u16) << 12),
        ),
        ..Default::default()
    };

    let mut mailbox_out = MailboxOut::default();
    sdo.write_to(&mut mailbox_out)?;
    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

    // Clear mailboxbuffer
    mailbox_in.clear();

    // Read slave response
    mailbox_in.receive(context, slave, timeout)?;
    let a_sdo = ServiceDataObject::read_from(&mut mailbox_in)?;

    if a_sdo.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::CanopenOverEthercat)
        && a_sdo.can_open.to_host() >> 12 == u16::from(COEMailboxType::TxPdo)
    {
        // TxPDO response
        let framedatasize = a_sdo.mailbox_header.length.to_host() - 2;

        // If the parameterbuffer is large enough
        if pdo_buffer.len() >= framedatasize.into() {
            // Copy parameter in parameter buffer
            let command_offset = ServiceDataObject::command_offset();
            pdo_buffer[..usize::from(framedatasize)]
                .copy_from_slice(&mailbox_in.as_ref()[command_offset..]);
            Ok(usize::from(framedatasize))
        } else {
            // Data container too small for type
            context.packet_error(slave, 0, 0, PacketError::DataContainerTooSmallForType);
            Err(PacketError::DataContainerTooSmallForType.into())
        }
    } else {
        // Other slave response

        // SDO abort frame received
        if a_sdo.command == u8::from(CanopenOverEthercatSdoCommand::Abort) {
            let abort_error =
                AbortError::Abort(a_sdo.data.get_long_compile_time_checked::<0>().to_host() as i32);
            context.sdo_error(
                slave,
                0,
                0,
                AbortError::Abort(a_sdo.data.get_long_compile_time_checked::<0>().to_host() as i32),
            );
            Err(abort_error.into())
        } else {
            context.packet_error(slave, 0, 0, PacketError::UnexpectedFrameReturned);
            Err(PacketError::UnexpectedFrameReturned.into())
        }
    }
}

/// Read PDO assign structure
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `pdo_assign`: PDO assign object
///
/// # Returns total bitlength of PDO assign
fn read_pdo_assign(context: &mut Context, slave: u16, pdo_assign: u16) -> Result<u32, CoEError> {
    let read_data = Ethercat::<u16>::from_bytes(ServiceDataObject::read_bytes(
        context,
        slave,
        pdo_assign,
        0,
        TIMEOUT_RX_MAILBOX,
    )?)
    .to_host();

    // Check whether the response from slave was positive
    if read_data == 0 {
        return Ok(0);
    }

    // Number of available sub indexes
    let index_count = read_data;
    Ok((1..=index_count)
        .filter_map(|index_loop| {
            // Read PDO assign
            // Result is index of PDO
            let index = Ethercat::<u16>::from_bytes(
                ServiceDataObject::read_bytes(
                    context,
                    slave,
                    pdo_assign,
                    index_loop as u8,
                    TIMEOUT_RX_MAILBOX,
                )
                .ok()?,
            )
            .to_host();
            if index == 0 {
                return None;
            }

            let sub_index =
                ServiceDataObject::read_bytes::<1>(context, slave, index, 0, TIMEOUT_RX_MAILBOX)
                    .ok()?[0];
            Some(
                (1..=sub_index)
                    .filter_map(|sub_index_loop| {
                        let read_data = Ethercat::<u16>::from_bytes(
                            ServiceDataObject::read_bytes(
                                context,
                                slave,
                                index,
                                sub_index_loop,
                                TIMEOUT_RX_MAILBOX,
                            )
                            .ok()?,
                        )
                        .to_host();

                        // Extract bitlength of SDO
                        if low_byte(read_data) < 0xFF {
                            Some(u32::from(low_byte(read_data)))
                        } else {
                            // Used `readOEsingle(idx, (uint8)SubCount, pODlist, pOElist);` in old SOEM
                            // codebase, but is already outcomented in current original SOEM codebase.
                            Some(0xFF)
                        }
                    })
                    .sum::<u32>(),
            )
        })
        .sum())
}

/// Read PDO assign structure in complete access mode.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `thread_number`: Call thread index
/// - `pdo_assign`: PDO assign object
///
/// # Returns
/// Total bitlength of PDO assign
fn read_pdo_assign_complete_access(
    context: &mut Context,
    slave: u16,
    thread_number: usize,
    pdo_assign: u16,
) -> Result<u32, CoEError> {
    // Find maximum size of PDOassign buffer
    let mut pdo_assign_bytes = [0; PdoAssign::size()];

    // Read rxPDOassign in complete access mode, all subindexes areread in one struct
    ServiceDataObject::read(
        context,
        slave,
        pdo_assign,
        0,
        true,
        &mut pdo_assign_bytes,
        TIMEOUT_RX_MAILBOX,
    )?;

    let pdo_assign = PdoAssign::read_from(&mut pdo_assign_bytes.as_slice())?;
    if pdo_assign.number == 0 {
        return Ok(0);
    }

    let bit_length = pdo_assign
        .index()
        .iter()
        .take(pdo_assign.number.into())
        .copied()
        .map(Ethercat::to_host)
        .filter(|index| *index > 0)
        .filter_map(|index| {
            let mut pdo_description = [0; PdoDescription::size()];
            ServiceDataObject::read(
                context,
                slave,
                index,
                0,
                true,
                &mut pdo_description,
                TIMEOUT_RX_MAILBOX,
            )
            .ok()?;
            let pdo_description =
                PdoDescription::read_from(&mut pdo_description.as_slice()).ok()?;

            // Extract all bitlengths of SDO'
            let bit_length = pdo_description
                .pdo()
                .iter()
                .take(pdo_description.number.into())
                .copied()
                .map(|long| u32::from(low_byte(long.to_host() as u16)))
                .sum::<u32>();

            *context.get_pdo_description_mut(thread_number) = pdo_description;
            Some(bit_length)
        })
        .sum::<u32>();
    *context.get_pdo_assign_mut(thread_number) = pdo_assign;

    Ok(bit_length)
}

/// CANopen over EtherCAT read PDO mapping
///
/// CANopen has standard indexes defined for PDO mapping. This function
/// tries to read them and collect a full input and output mapping size
/// of designated slave.
///
/// Principal structure in slave:
/// - 1C00:00 is number of SM defined
/// - 1C00:01 SM0 type -> 1c10
/// - 1C00:02 SM1 type -> 1c11
/// - 1C00:03 SM2 type -> 1c12
/// - 1C00:04 SM3 type -> 1c13
///
/// Type 0 = unused, 1 = mailbox in, 3 = outputs(RxPDO), 4 = inputs (TxPDO).
///
/// 1C12:00 is number of PDO's defined for SM2.
/// 1. 1C12:01 PDO assign SDO #1 -> f.e. 1A00.
/// 2. 1C12:02 PDO assign SDO #2 -> f.e. 1A04
///
/// - 1A00:00 is number of objects defined for this PDO
/// - 1A00:01 object mapping #1, f.e. 60100710 (SDO 6010 SI 07 bitlength 0x10)
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `output_size`: Size in bits of output mapping (rxPDO) found
/// - `input_size`: Size in bits of input mapping (txPDO) found
///
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
/// - Received data couldn't be parsed into an object
///
/// # Returns
/// Whether any IO was found if successful, error otherwise
pub fn read_pdo_map(
    context: &mut Context,
    slave: u16,
    output_size: &mut u32,
    input_size: &mut u32,
) -> Result<bool, CoEError> {
    // Limit the maximum number of Sync Manager defined
    let sync_manager_count =
        ServiceDataObject::read_bytes::<1>(context, slave, SDO_SMCOMMTYPE, 0, TIMEOUT_RX_MAILBOX)?
            [0]
        .min(MAX_SM);

    *input_size = 0;
    *output_size = 0;

    // Was a valid result received from slave
    if sync_manager_count <= 2 {
        return Err(CoEError::NotEnoughSyncManagers);
    }

    let mut sync_manager_bug_add = 0;

    for input_sync_manager in 2..sync_manager_count {
        let mut sync_manager_type = match ServiceDataObject::read_bytes::<1>(
            context,
            slave,
            SDO_SMCOMMTYPE,
            input_sync_manager + 1,
            TIMEOUT_RX_MAILBOX,
        ) {
            Ok(bytes) => bytes[0],
            Err(_) => continue,
        };

        // Start slave bug prevention code, remove if possible
        if input_sync_manager == 2 && sync_manager_type == 2 {
            // SM2 has type 2 == mailbox out, this is a bug in the slave!
            // Try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
            sync_manager_bug_add = 1;
        }
        if sync_manager_type != 0 {
            // Only add if `sync_manager_type` > 0
            sync_manager_type += sync_manager_bug_add;
        } else if input_sync_manager == 2 && sync_manager_type == 0 {
            // SM2 has type 0, this is a bug in the slave
            sync_manager_type = 3;
        } else if input_sync_manager == 3 && sync_manager_type == 0 {
            // SM3 has type 0, this is a bug in the slave
            sync_manager_type = 4;
        }
        // End slave bug prevention code

        let Ok(sync_manager_type) = SyncManagerType::try_from(sync_manager_type) else {
            continue;
        };
        *context
            .get_slave_mut(slave)
            .get_sync_manager_type_mut(input_sync_manager) = sync_manager_type;

        if sync_manager_type == SyncManagerType::Unused {
            let flags = &mut context
                .get_slave_mut(slave)
                .get_sync_manager_mut(input_sync_manager)
                .sm_flags;
            *flags = Ethercat::from_host(flags.to_host() & SYNC_MANAGER_ENABLE_MASK);
        }
        if !matches!(
            sync_manager_type,
            SyncManagerType::Outputs | SyncManagerType::Inputs
        ) {
            continue;
        }
        // Read the assign PDO
        let Ok(mapping_bit_size) = read_pdo_assign(
            context,
            slave,
            SDO_PDO_ASSIGNMENT + u16::from(input_sync_manager),
        ) else {
            continue;
        };

        // Skip if the mapping isn't found
        if mapping_bit_size == 0 {
            continue;
        }

        context
            .get_slave_mut(slave)
            .get_sync_manager_mut(input_sync_manager)
            .sm_length = Ethercat::from_host(mapping_bit_size.div_ceil(8) as u16);
        *if sync_manager_type == SyncManagerType::Outputs {
            // It's an output mapping
            &mut *output_size
        } else {
            // It's an input mapping
            &mut *input_size
        } += mapping_bit_size;
    }

    Ok(*input_size != 0 || *output_size != 0)
}

/// CANopen over EtherCAT mapping in Complete Access mode
///
/// CANopen has standard indexes defined for PDO mapping. This function
/// tries to read them and collect a full input and output mapping size
/// of designated slave. Slave has to support complete access, otherwise
/// use `read_pdo_map`.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `thread_number`: Calling thread index
/// - `output_size`: Size in bits of output mapping (rxPDO) found
/// - `input_size`: Size in bits of input mapping (txPDO) found
///
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
/// - Received data couldn't be parsed into an object
///
/// # Returns
/// Unit or error
pub fn read_pdo_map_complete_access(
    context: &mut Context,
    slave: u16,
    thread_number: usize,
    output_size: &mut u32,
    input_size: &mut u32,
) -> Result<bool, CoEError> {
    (*input_size, *output_size) = (0, 0);
    context
        .get_sync_manager_communication_type_mut(thread_number)
        .number = 0;

    let mut sync_manager_communication_type: [u8; size_of::<SyncManagerCommunicationType>()] =
        context
            .get_sync_manager_communication_type_mut(thread_number)
            .clone()
            .into();

    // Read sync manager communication type object count with complete access
    ServiceDataObject::read(
        context,
        slave,
        SDO_SMCOMMTYPE,
        0,
        true,
        &mut sync_manager_communication_type,
        TIMEOUT_RX_MAILBOX,
    )?;
    let sync_manager_communication_type =
        SyncManagerCommunicationType::try_from(sync_manager_communication_type)?;

    // Check whether the result from slave was positive
    let sync_manager_count = sync_manager_communication_type.number;
    if sync_manager_count <= 2 {
        return Err(CoEError::NotEnoughSyncManagers);
    }

    if sync_manager_count > MAX_SM {
        context.packet_error(slave, 0, 0, PacketError::TooManySyncManagers);
        return Err(PacketError::TooManySyncManagers.into());
    }

    let mut sync_manager_bug_add = 0;
    for input_sync_manager in 2..sync_manager_count {
        let mut sync_manager_type =
            u8::from(sync_manager_communication_type.get_sync_manager_type(input_sync_manager));

        // Start slave bug prevention code, remove if possible
        if input_sync_manager == 2 && sync_manager_type == 2 {
            // SM2 has type 2 == mailbox out, this is a bug in the slave!
            // Try to correct, this works if the types are 0 1 2 3 and should be 1 2 3 4
            sync_manager_bug_add = 1;
        }
        if sync_manager_type != 0 {
            // Only add if `sync_manager_type` > 0
            sync_manager_type += sync_manager_bug_add;
        }
        // End slave bug prevention code

        let Ok(sync_manager_type) = SyncManagerType::try_from(sync_manager_type) else {
            continue;
        };
        *context
            .get_slave_mut(slave)
            .get_sync_manager_type_mut(input_sync_manager) = sync_manager_type;

        // Check if SyncManagr is unused -> clear enable flag
        if sync_manager_type == SyncManagerType::Unused {
            let flags = &mut context
                .get_slave_mut(slave)
                .get_sync_manager_mut(input_sync_manager)
                .sm_flags;
            *flags = Ethercat::from_host(flags.to_host() & SYNC_MANAGER_ENABLE_MASK);
        }
        if !matches!(
            sync_manager_type,
            SyncManagerType::Outputs | SyncManagerType::Inputs
        ) {
            continue;
        }

        // Read the assign PDO
        let Ok(mapping_size) = read_pdo_assign_complete_access(
            context,
            slave,
            thread_number,
            SDO_PDO_ASSIGNMENT + u16::from(input_sync_manager),
        ) else {
            continue;
        };

        // Skip if no mapping was found
        if mapping_size == 0 {
            continue;
        }

        context
            .get_slave_mut(slave)
            .get_sync_manager_mut(input_sync_manager)
            .sm_length = Ethercat::from_host(mapping_size.div_ceil(8) as u16);

        *if sync_manager_type == SyncManagerType::Outputs {
            // It's an output mapping
            &mut *output_size
        } else {
            // It's an input mapping
            &mut *input_size
        } += mapping_size;
    }
    *context.get_sync_manager_communication_type_mut(thread_number) =
        sync_manager_communication_type;

    Ok(*input_size == 0 && *output_size == 0)
}

#[cfg(test)]
mod tests {
    use crate::ethercat::coe::ServiceDataObjectService;

    #[test]
    fn service_data_object_service_bytes() {
        assert_eq!(
            ServiceDataObjectService::default().bytes(),
            [0; ServiceDataObjectService::size()]
        );
    }
}
