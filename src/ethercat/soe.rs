//! Servo over EtherCAT (SoE) module

use std::{
    io::{self, Read, Write},
    time::Duration,
};

use crate::ethercat::{
    main::{MailboxOut, PacketError},
    r#type::{
        ethercat_to_host, host_to_ethercat, mailbox_header_set_count, MailboxType,
        ServoOverEthercatOpcode,
    },
};

use super::{
    main::{Context, MailboxHeader, MailboxIn, MainError},
    r#type::{Ethercat, InvalidServoOverEthercatOpcode},
};

pub const DATASTATE_BIT: u8 = 0x01;
pub const NAME_BIT: u8 = 0x02;
pub const ATTRIBUTE_BIT: u8 = 0x04;
pub const UNIT_BIT: u8 = 0x08;
pub const MIN_BIT: u8 = 0x10;
pub const MAX_BIT: u8 = 0x20;
pub const VALUE_BIT: u8 = 0x40;
pub const DEFAULT_BIT: u8 = 0x80;

pub const MAX_NAME_LENGTH: usize = 60;
pub const MAX_MAPPING_LENGTH: usize = 64;

pub const IDN_MDT_CONFIG: u8 = 24;
pub const IDN_AT_CONFIG: u8 = 16;

pub const LENGTH1: u8 = 0;
pub const LENGTH2: u8 = 1;
pub const LENGTH4: u8 = 2;
pub const LENGTH8: u8 = 3;

pub enum ServoOverEthercatError {
    Main(MainError),
    Io(io::Error),
    InvalidServoOverEthercatOpcode(InvalidServoOverEthercatOpcode),
    ErrorCodeReceived(u16),
    NoResponse,
    UnexpectedFrameReturned,
}

impl From<MainError> for ServoOverEthercatError {
    fn from(value: MainError) -> Self {
        Self::Main(value)
    }
}

impl From<io::Error> for ServoOverEthercatError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<InvalidServoOverEthercatOpcode> for ServoOverEthercatError {
    fn from(value: InvalidServoOverEthercatOpcode) -> Self {
        Self::InvalidServoOverEthercatOpcode(value)
    }
}

/// SoE name
pub struct SoEName {
    /// Current length in bytes of list
    current_length: u16,

    /// Maximum length in bytes of list
    max_length: u16,

    name: heapless::String<MAX_NAME_LENGTH>,
}

/// SoE list
pub enum SoEListValue {
    Byte([u8; 8]),
    Word([u16; 4]),
    Dword([u32; 2]),
    Lword(u64),
}

pub struct SoEList {
    /// Current length in bytes of list
    current_length: u16,

    /// Maximum length in bytes of list
    max_length: u16,

    data: SoEListValue,
}

/// SoE IDN mapping
pub struct SoeMapping {
    /// Current length in bytes of list
    current_length: u16,

    /// Maximum length in bytes of list
    max_length: u16,

    idn: [u16; MAX_MAPPING_LENGTH],
}

pub enum SoeType {
    Binary,
    Uint,
    Int,
    Hex,
    String,
    Idn,
    Float,
    Parameter,
}

pub struct SoeAttribute {
    /// Evaluation factor for display purposes
    eval_factor: [u32; 16],

    /// Length of IDN element(s)
    length: [u32; 2],

    /// IDN is list
    list: u32,

    /// IDN is command
    command: u32,

    /// datatype
    datatype: [u32; 3],
    reserved1: u32,

    /// Decimals to display in float datatype
    decimals: [u32; 4],

    /// Write protected pre-op
    wppreop: u32,

    /// Write protected in safe-op
    wpsafeop: u32,

    /// Write protected in op
    wpop: u32,
    reserved2: u32,
}

enum SoEValue {
    Idn(Ethercat<u16>),
    FragmentsLeft(Ethercat<u16>),
}

impl SoEValue {
    const fn inner(&self) -> Ethercat<u16> {
        match self {
            SoEValue::Idn(ethercat) | SoEValue::FragmentsLeft(ethercat) => *ethercat,
        }
    }
}

/// Servo over EtherCAT mailbox structure
struct ServoOverEthercat {
    mailbox_header: MailboxHeader,
    opcode: ServoOverEthercatOpcode,
    incomplete: bool,
    error: u8,
    drive_number: u8,
    element_flags: u8,
    value: SoEValue,
}

impl Default for ServoOverEthercat {
    fn default() -> Self {
        Self {
            mailbox_header: MailboxHeader::default(),
            opcode: ServoOverEthercatOpcode::ReadRequest,
            incomplete: false,
            error: 0,
            drive_number: 0,
            element_flags: 0,
            value: SoEValue::Idn(Ethercat::default()),
        }
    }
}

impl ServoOverEthercat {
    const fn size() -> usize {
        MailboxHeader::size() + 5 * size_of::<u8>() + size_of::<u16>()
    }

    pub fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        self.mailbox_header.write_to(writer)?;
        writer.write_all(&[
            u8::from(self.opcode),
            u8::from(self.incomplete),
            self.error,
            self.drive_number,
            self.element_flags,
        ])?;
        writer.write_all(&self.value.inner().to_bytes())
    }

    pub fn read_from<R: Read>(reader: &mut R) -> Result<Self, ServoOverEthercatError> {
        let mailbox_header = MailboxHeader::read_from(reader)?;
        let mut bytes = [0; 5];
        reader.read_exact(&mut bytes);
        let opcode = ServoOverEthercatOpcode::try_from(bytes[0])?;
        let incomplete = bytes[1] != 0;
        let error = bytes[2];
        let drive_number = bytes[3];
        let element_flags = bytes[4];

        let mut word = [0; 2];
        reader.read_exact(&mut word)?;
        let value = SoEValue::FragmentsLeft(Ethercat::<u16>::from_bytes(word));
        Ok(Self {
            mailbox_header,
            opcode,
            incomplete,
            error,
            drive_number,
            element_flags,
            value,
        })
    }
}

/// Servo over Ethercat read, blocking.
///
/// The IDN object of the selected slave and drive number is read. If a response
/// is larger than the mailbox size, the response is segmented. The function will
/// combine all segments and copy them to the parameter buffer.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `drive_number`: Drive number in slave
/// - `element_flags`: Flags to select what properties of IDN are to be transferred.
/// - `idn`: IDN
/// - `timeout`: Duration
///
/// # Returns
/// Bytes read or `Error`
pub fn soe_read(
    context: &mut Context,
    slave: u16,
    drive_number: u8,
    element_flags: u8,
    idn: u16,
    parameter_buffer: &mut [u8],
    timeout: Duration,
) -> Result<u16, ServoOverEthercatError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox if something is in with timeout set to 0.
    mailbox_in.receive(context, slave, Duration::default())?;

    let soe = ServoOverEthercat {
        mailbox_header: MailboxHeader::new(
            host_to_ethercat((ServoOverEthercat::size() - MailboxHeader::size()) as u16),
            host_to_ethercat(0),
            0,
            u8::from(MailboxType::ServoOverEthercat)
                + mailbox_header_set_count(context.get_slave_mut(slave).mailbox_mut().next_count()),
        ),
        opcode: ServoOverEthercatOpcode::ReadRequest,
        incomplete: false,
        error: 0,
        drive_number,
        element_flags,
        value: SoEValue::Idn(host_to_ethercat(idn)),
    };

    // Send Servo over Ethercat request to slave
    let mut mailbox_out = MailboxOut::default();
    soe.write_to(&mut mailbox_out)?;
    mailbox_out.send(context, slave, timeout)?;

    let mut total_size = 0;
    let mut base_index = 0;
    let mailbox_index = ServoOverEthercat::size();
    loop {
        mailbox_in.clear();
        if let Err(error) = mailbox_in.receive(context, slave, timeout) {
            context.packet_error(slave, idn, 0, PacketError::NoResponse);
            return Err(error.into());
        }

        // Slave response should be Servo over EtherCAT, read response
        let a_soe = ServoOverEthercat::read_from(&mut mailbox_in)?;
        if a_soe.mailbox_header.mailbox_type() & 0xF == u8::from(MailboxType::ServoOverEthercat)
            && a_soe.opcode == ServoOverEthercatOpcode::ReadResponse
            && a_soe.error == 0
            && a_soe.drive_number == drive_number
            && a_soe.element_flags == element_flags
        {
            let mut frame_data_size = ethercat_to_host(a_soe.mailbox_header.length())
                + (MailboxHeader::size() - ServoOverEthercat::size()) as u16;
            total_size += frame_data_size;

            // If the parameter fits in the parameter buffer
            if usize::from(total_size) <= parameter_buffer.len() {
                // Copy parameter data in parameter buffer
                parameter_buffer[base_index..]
                    .copy_from_slice(&mailbox_in.as_ref()[mailbox_index..]);

                // Increment buffer pointer
                base_index += usize::from(frame_data_size);
            } else {
                frame_data_size -= total_size - parameter_buffer.len() as u16;
                total_size = parameter_buffer.len() as u16;

                // Copy parameter data in parameter buffer
                if frame_data_size > 0 {
                    parameter_buffer[base_index..]
                        .copy_from_slice(&mailbox_in.as_ref()[mailbox_index..]);
                }
            }

            if !a_soe.incomplete {
                return Ok(total_size);
            }
        } else {
            // Other slave response
            if a_soe.mailbox_header.mailbox_type() & 0xF == u8::from(MailboxType::ServoOverEthercat)
                && a_soe.opcode == ServoOverEthercatOpcode::ReadResponse
                && a_soe.error == 1
            {
                let error_code = u16::from_ne_bytes(
                    (&mailbox_in.as_ref()[usize::from(ethercat_to_host(
                        a_soe.mailbox_header.length(),
                    )) + MailboxHeader::size()
                        - size_of::<u16>()..])[..2]
                        .try_into()
                        .unwrap(),
                );
                context.servo_over_ethercat_error(slave, idn, error_code);
                return Err(ServoOverEthercatError::ErrorCodeReceived(error_code));
            }
            // Unexpected frame returned
            context.packet_error(slave, idn, 0, PacketError::UnexpectedFrameReturned);
            return Err(ServoOverEthercatError::UnexpectedFrameReturned);
        }
    }
}

pub fn soe_write(
    context: &mut Context,
    slave: u16,
    drive_no: u8,
    elementflags: u8,
    idn: u16,
    pointer: &mut [u8],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn read_id_nmap(context: &mut Context, slave: u16, osize: &mut u32, isize: &mut u32) -> i32 {
    todo!()
}
