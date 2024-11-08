//! File over EtherCAT (FoE) module.
//!
//! SDO read/write and SDO service functions.

use std::{
    io::{self, Read, Write},
    str::FromStr,
    time::Duration,
};

use crate::ethercat::{
    main::{MailboxIn, MailboxOut},
    r#type::{
        mailbox_header_set_count, ErrorType, FileOverEthercatOpcode, MailboxType,
        TIMEOUT_TX_MAILBOX,
    },
};

use super::{
    main::{Context, MailboxHeader, MainError},
    r#type::{Ethercat, InvalidFOEOpcode},
    ReadFrom,
};

const MAX_FOE_DATA: usize = 512;

#[derive(Debug)]
pub enum FoEError {
    Main(MainError),
    Io(io::Error),
    InvalidOpcode(InvalidFOEOpcode),
    ErrorType(ErrorType),
}

impl From<MainError> for FoEError {
    fn from(value: MainError) -> Self {
        Self::Main(value)
    }
}

impl From<io::Error> for FoEError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<InvalidFOEOpcode> for FoEError {
    fn from(value: InvalidFOEOpcode) -> Self {
        Self::InvalidOpcode(value)
    }
}

impl From<ErrorType> for FoEError {
    fn from(value: ErrorType) -> Self {
        Self::ErrorType(value)
    }
}

enum PacketInfo {
    Password(Ethercat<u32>),
    PacketNumber(Ethercat<u32>),
    #[expect(dead_code, reason = "Wasn't used in original code")]
    ErrorCode(Ethercat<u32>),
}

impl Default for PacketInfo {
    fn default() -> Self {
        Self::PacketNumber(Ethercat::default())
    }
}

impl PacketInfo {
    pub const fn inner(&self) -> Ethercat<u32> {
        match self {
            PacketInfo::Password(value)
            | PacketInfo::PacketNumber(value)
            | PacketInfo::ErrorCode(value) => *value,
        }
    }
}

enum PacketData {
    FileName(heapless::String<MAX_FOE_DATA>),
    Data(heapless::Vec<u8, MAX_FOE_DATA>),
    #[expect(dead_code, reason = "Wasn't used in the original code")]
    ErrorText(heapless::String<MAX_FOE_DATA>),
}

impl Default for PacketData {
    fn default() -> Self {
        Self::Data(heapless::Vec::default())
    }
}

impl PacketData {
    pub fn as_bytes(&self) -> &[u8] {
        match self {
            PacketData::FileName(string) | PacketData::ErrorText(string) => string.as_bytes(),
            PacketData::Data(vec) => vec,
        }
    }
}

/// File over Ethercat data
#[derive(Default)]
struct FileOverEthercat {
    mailbox_header: MailboxHeader,
    opcode: FileOverEthercatOpcode,
    reserved: u8,
    packet_info: PacketInfo,
    packet_data: PacketData,
}

impl<R: Read> ReadFrom<R> for FileOverEthercat {
    type Err = FoEError;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let mailbox_header = MailboxHeader::read_from(reader)?;
        let opcode = Self::read_byte(reader)?.try_into()?;
        let reserved = Self::read_byte(reader)?;
        let packet_info =
            PacketInfo::PacketNumber(Ethercat::<u32>::from_bytes(Self::read_bytes(reader)?));
        let packet_data = PacketData::Data(
            heapless::Vec::from_slice(&Self::read_bytes::<MAX_FOE_DATA>(reader)?).unwrap(),
        );
        Ok(Self {
            mailbox_header,
            opcode,
            reserved,
            packet_info,
            packet_data,
        })
    }
}

impl FileOverEthercat {
    pub fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        self.mailbox_header.write_to(writer)?;
        writer.write_all(&[u8::from(self.opcode), self.reserved])?;
        writer.write_all(&self.packet_info.inner().to_bytes())?;
        writer.write_all(self.packet_data.as_bytes())
    }
}

/// File over Ethercat read, blocking.
///
/// # Parameters
/// `context`: Context struct
/// `slave`: Slave number
/// `filename`: Filename of file to read
/// `password`: Password
/// `size`: returns bytes read from file
/// `buffer`: File buffer
/// `timeout`: Timeout per mailbox cycle, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
/// - Received data couldn't be parsed into a FileOverEthercat struct
/// - The file over Ethercat buffer is too small
/// - An error packet was received
/// - An invalid packet was received
///
/// # Returns
/// `Ok(())` or Error
#[expect(clippy::missing_panics_doc)]
pub fn foe_read(
    context: &mut Context,
    slave: u16,
    mut file_name: &str,
    password: u32,
    size: &mut usize,
    buffer: &mut [u8],
    timeout: Duration,
) -> Result<(), FoEError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox if something is in it, timeout set to 0.
    mailbox_in.receive(context, slave, Duration::default())?;

    let mut foe = FileOverEthercat::default();
    let max_data = usize::from(context.get_slave(slave).mailbox().length() - 12);

    // Limit the size of the filename to `MAX_FOE_DATA` or `max_data`
    file_name = &file_name[..(0..=file_name.len().min(MAX_FOE_DATA).min(max_data))
        .rev()
        .find(|&length| file_name.get(..length).is_some())
        .unwrap_or_default()];

    *foe.mailbox_header.length_mut() = Ethercat::from_host(6 + file_name.len() as u16);
    *foe.mailbox_header.address_mut() = Ethercat::from_host(0);
    *foe.mailbox_header.priority_mut() = 0;

    // Get new mailbox count value, used as session handle
    *foe.mailbox_header.mailbox_type_mut() = u8::from(MailboxType::FileOverEthercat)
        + mailbox_header_set_count(context.get_slave_mut(slave).mailbox_mut().next_count());
    foe.opcode = FileOverEthercatOpcode::Read;
    foe.packet_info = PacketInfo::Password(Ethercat::from_host(password));

    // Copy filename into packet
    foe.packet_data = PacketData::FileName(heapless::String::from_str(file_name).unwrap());

    // Send File over Ethercat request to slave
    let mut mailbox_out = MailboxOut::default();
    foe.write_to(&mut mailbox_out)?;
    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

    let mut previous_packet = 0;
    let mut data_read = 0;
    loop {
        let mut work_to_do = false;

        // Clean mailboxbuffer
        mailbox_in.clear();

        // Read slave response
        mailbox_in.receive(context, slave, timeout)?;

        // Slave response should be File over Ethercat
        let a_foe = FileOverEthercat::read_from(&mut mailbox_in)?;
        if a_foe.mailbox_header.mailbox_type() & 0xF != u8::from(MailboxType::FileOverEthercat) {
            return Err(ErrorType::PacketError.into());
        }
        // Slave response should be a data response File over Ethercat
        match a_foe.opcode {
            FileOverEthercatOpcode::Data => {
                let segment_data = a_foe.mailbox_header.length().to_host() - 6;
                let packetnumber = a_foe.packet_info.inner().to_host();
                previous_packet += 1;
                if packetnumber != previous_packet
                    || usize::from(data_read + segment_data) <= buffer.len()
                {
                    return Err(ErrorType::FileOverEthernetBufferTooSmall.into());
                }
                buffer[usize::from(data_read)..]
                    .copy_from_slice(&a_foe.packet_data.as_bytes()[..usize::from(segment_data)]);
                data_read += segment_data;
                if usize::from(segment_data) == max_data {
                    work_to_do = true;
                }
                *foe.mailbox_header.length_mut() = Ethercat::from_host(6);
                *foe.mailbox_header.address_mut() = Ethercat::from_host(0);
                *foe.mailbox_header.priority_mut() = 0;

                // Get new mailbox count value
                *foe.mailbox_header.mailbox_type_mut() = u8::from(MailboxType::FileOverEthercat)
                    + mailbox_header_set_count(
                        context.get_slave_mut(slave).mailbox_mut().next_count(),
                    );
                foe.opcode = FileOverEthercatOpcode::Ack;
                foe.packet_info = PacketInfo::PacketNumber(Ethercat::from_host(packetnumber));
                mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;
                if let Some(file_over_ethercat_hook) = context.file_over_ethercat_hook() {
                    file_over_ethercat_hook(slave, packetnumber as i32, i32::from(data_read));
                }
            }
            FileOverEthercatOpcode::Error => return Err(ErrorType::FileOverEthercatError.into()),
            _ => return Err(ErrorType::PacketError.into()),
        }
        *size = usize::from(data_read);

        if !work_to_do {
            break;
        }
    }
    Ok(())
}

/// File over Ethercat write, blocking.
///
/// # Parameters
/// `context`: Context struct
/// `slave`: Slave number
/// `filename`: Filename of file to write
/// `password`: Password
/// `buffer`: File buffer
/// `timeout`: Timeout per mailbox cycle, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Panics
/// Panics if the received data is larger than the maximum packet data size.
///
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
/// - The wrong packet type was received
/// - An error packet was received
///
/// # Returns
/// `Ok(())` or error
pub fn foe_write(
    context: &mut Context,
    slave: u16,
    mut file_name: &str,
    password: u32,
    buffer: &mut [u8],
    timeout: Duration,
) -> Result<(), FoEError> {
    // Empty slave out mailobx if something is in it, with timeout set to 0.
    let mut mailbox_in = MailboxIn::default();
    mailbox_in.receive(context, slave, Duration::default())?;

    let mut do_final_zero = true;
    let max_data = usize::from(context.get_slave(slave).mailbox().length()) - 12;
    file_name = &file_name[..(0..=file_name.len().min(MAX_FOE_DATA).min(max_data))
        .rev()
        .find(|&length| file_name.get(..length).is_some())
        .unwrap_or_default()];

    let mut foe = FileOverEthercat::default();
    *foe.mailbox_header.length_mut() = Ethercat::from_host(6 + file_name.len() as u16);
    *foe.mailbox_header.address_mut() = Ethercat::from_host(0);
    *foe.mailbox_header.priority_mut() = 0;

    // Get new mailbox count value, used as session handle
    *foe.mailbox_header.mailbox_type_mut() = u8::from(MailboxType::FileOverEthercat)
        + mailbox_header_set_count(context.get_slave_mut(slave).mailbox_mut().next_count());

    foe.opcode = FileOverEthercatOpcode::Write;
    foe.packet_info = PacketInfo::Password(Ethercat::from_host(password));

    // Copy filename into mailbox
    foe.packet_data = PacketData::FileName(heapless::String::from_str(file_name).unwrap());

    // Send File over Ethercat request to slave
    let mut mailbox_out = MailboxOut::default();
    foe.write_to(&mut mailbox_out)?;
    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;
    let mut send_packet = 0;
    let mut buffer_start = 0;
    let mut segment_data = 0;
    loop {
        let mut work_to_do = false;

        // Clean slave mailboxbuffer
        mailbox_in.clear();

        // Read slave response
        mailbox_in.receive(context, slave, timeout)?;
        let a_foe = FileOverEthercat::read_from(&mut mailbox_in)?;
        if a_foe.mailbox_header.mailbox_type() & 0xF != u8::from(MailboxType::FileOverEthercat) {
            return Err(ErrorType::PacketError.into());
        }
        match a_foe.opcode {
            FileOverEthercatOpcode::Ack => {
                let packet_number = a_foe.packet_info.inner().to_host();
                if packet_number != send_packet {
                    return Err(ErrorType::PacketNumber.into());
                }
                if let Some(file_over_ethercat_hook) = context.file_over_ethercat_hook() {
                    file_over_ethercat_hook(
                        slave,
                        packet_number as i32,
                        (buffer.len() - buffer_start) as i32,
                    );
                }
                let tsize = (buffer.len() - buffer_start).min(max_data);

                if tsize != 0 || do_final_zero {
                    work_to_do = true;
                    do_final_zero = false;
                    segment_data = tsize;

                    // If last packet was full size, add a zero size packet as final
                    // End of File is defined as packetsize < full packetsize
                    if buffer.len() - segment_data - buffer_start == 0 && segment_data == max_data {
                        do_final_zero = true;
                    }
                    *foe.mailbox_header.length_mut() = Ethercat::from_host(6 + segment_data as u16);
                    *foe.mailbox_header.address_mut() = Ethercat::from_host(0);
                    *foe.mailbox_header.priority_mut() = 0;

                    // Get new mailbox count value
                    *foe.mailbox_header.mailbox_type_mut() =
                        u8::from(MailboxType::FileOverEthercat)
                            + mailbox_header_set_count(
                                context.get_slave_mut(slave).mailbox_mut().next_count(),
                            );
                    foe.opcode = FileOverEthercatOpcode::Data;
                    send_packet += 1;
                    foe.packet_info = PacketInfo::PacketNumber(Ethercat::from_host(send_packet));
                    foe.packet_data = PacketData::Data(
                        heapless::Vec::from_slice(
                            &buffer[buffer_start..buffer_start + segment_data],
                        )
                        .unwrap(),
                    );
                    buffer_start += segment_data;

                    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;
                }
            }
            FileOverEthercatOpcode::Busy => {
                // Resend if data has been send before, otherwise ignore.
                if send_packet != 0 {
                    if buffer.len() - buffer_start == 0 {
                        do_final_zero = true;
                    }
                    buffer_start -= segment_data;
                    send_packet -= 1;
                }
            }
            FileOverEthercatOpcode::Error => {
                if a_foe.packet_info.inner().to_host() == 0x8001 {
                    return Err(ErrorType::FileOverEthernetFileNotFoundError.into());
                }
                return Err(ErrorType::FileOverEthercatError.into());
            }
            FileOverEthercatOpcode::Read
            | FileOverEthercatOpcode::Write
            | FileOverEthercatOpcode::Data => return Err(ErrorType::PacketError.into()),
        }

        if !work_to_do {
            break;
        }
    }

    Ok(())
}
