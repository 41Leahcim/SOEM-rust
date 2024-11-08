//! Servo over EtherCAT (SoE) module

use std::{
    io::{self, Read, Write},
    time::Duration,
};

use bytemuck::{AnyBitPattern, NoUninit, Zeroable};

use crate::{
    ethercat::{
        main::{MailboxOut, PacketError},
        r#type::{mailbox_header_set_count, MailboxType, ServoOverEthercatOpcode},
    },
    oshw::nicdrv::NicdrvError,
};

use super::{
    main::{Context, MailboxHeader, MailboxIn, MainError},
    r#type::{Ethercat, InvalidServoOverEthercatOpcode, TIMEOUT_RX_MAILBOX},
    ReadFrom,
};

const DATASTATE_B: u8 = 0x01;
const NAME_B: u8 = 0x02;
const ATTRIBUTE_B: u8 = 0x04;
const UNIT_B: u8 = 0x08;
const MIN_B: u8 = 0x10;
const MAX_B: u8 = 0x20;
const VALUE_B: u8 = 0x40;
const DEFAULT_B: u8 = 0x80;

const MAX_NAME_LENGTH: usize = 60;
const MAX_MAPPING_LENGTH: usize = 64;

const IDN_MDT_CONFIG: u8 = 24;
const IDN_AT_CONFIG: u8 = 16;

const LENGTH1: u8 = 0;
const LENGTH2: u8 = 1;
const LENGTH4: u8 = 2;
const LENGTH8: u8 = 3;

const SOE_MAX_DRIVES: u8 = 8;

pub enum ServoOverEthercatError {
    Main(MainError),
    NicDrv(NicdrvError),
    Io(io::Error),
    InvalidServoOverEthercatOpcode(InvalidServoOverEthercatOpcode),
    ErrorCodeReceived(u16),
    InvalidDataType(u8),
    NoResponse,
    UnexpectedFrameReturned,
}

impl From<MainError> for ServoOverEthercatError {
    fn from(value: MainError) -> Self {
        Self::Main(value)
    }
}

impl From<NicdrvError> for ServoOverEthercatError {
    fn from(value: NicdrvError) -> Self {
        Self::NicDrv(value)
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
struct SoEName {
    /// Current length in bytes of list
    current_length: u16,

    /// Maximum length in bytes of list
    max_length: u16,

    name: heapless::String<MAX_NAME_LENGTH>,
}

/// SoE list
enum SoEListValue {
    Byte([u8; 8]),
    Word([u16; 4]),
    Dword([u32; 2]),
    Lword(u64),
}

struct SoEList {
    /// Current length in bytes of list
    current_length: u16,

    /// Maximum length in bytes of list
    max_length: u16,

    data: SoEListValue,
}

/// SoE IDN mapping
#[derive(Debug, Clone, Copy)]
struct SoeMapping {
    /// Current length in bytes of list
    current_length: Ethercat<u16>,

    /// Maximum length in bytes of list
    max_length: Ethercat<u16>,

    idn: [Ethercat<u16>; MAX_MAPPING_LENGTH],
}

impl Default for SoeMapping {
    fn default() -> Self {
        Self {
            current_length: Ethercat::default(),
            max_length: Ethercat::default(),
            idn: [Ethercat::default(); MAX_MAPPING_LENGTH],
        }
    }
}

unsafe impl NoUninit for SoeMapping {}

unsafe impl Zeroable for SoeMapping {}
unsafe impl AnyBitPattern for SoeMapping {}

impl SoeMapping {
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        bytemuck::bytes_of_mut(self)
    }
}

#[derive(Debug, Clone, Copy)]
enum SoeType {
    Binary,
    Uint,
    Int,
    Hex,
    String,
    Idn,
    Float,
    Parameter,
}

impl TryFrom<u8> for SoeType {
    type Error = ServoOverEthercatError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Binary),
            1 => Ok(Self::Uint),
            2 => Ok(Self::Int),
            3 => Ok(Self::Hex),
            4 => Ok(Self::String),
            5 => Ok(Self::Idn),
            6 => Ok(Self::Float),
            7 => Ok(Self::Parameter),
            _ => Err(ServoOverEthercatError::InvalidDataType(value)),
        }
    }
}

enum IdnType {
    /// IDN is list and IDN is command are both not set
    Value,

    /// IDN is list (1-bit)
    List,

    /// IDN is command (1-bit)
    Command,

    /// IDN is list and is command are both set
    CommandList,
}

impl From<u8> for IdnType {
    fn from(value: u8) -> Self {
        match value & 3 {
            0 => Self::Value,
            1 => Self::List,
            2 => Self::Command,
            _ => Self::CommandList,
        }
    }
}

impl IdnType {
    pub const fn is_list(&self) -> bool {
        matches!(self, Self::CommandList | Self::List)
    }
}

struct SoeAttribute {
    /// Evaluation factor for display purposes (16-bit)
    eval_factor: Ethercat<u16>,

    /// Length of IDN element(2-bit)
    length: u8,

    idn_type: IdnType,

    /// datatype (3-bit)
    datatype: SoeType,

    /// 1-bit reserved
    reserved1: (),

    /// Decimals to display in float datatype (4-bit)
    decimals: u8,

    /// Write protected in pre-op (1-bit)
    wppreop: bool,

    /// Write protected in safe-op (1-bit)
    wpsafeop: bool,

    /// Write protected in op (1-bit)
    wpop: bool,

    /// 1-bit reserved
    reserved2: (),
}

impl<R: Read> ReadFrom<R> for SoeAttribute {
    type Err = ServoOverEthercatError;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let eval_factor = Ethercat::<u16>::from_bytes(Self::read_bytes(reader)?);
        let flags = Ethercat::<u16>::from_bytes(Self::read_bytes(reader)?).to_host();
        let length = (flags & 3) as u8;
        let idn_type = IdnType::from(flags as u8 >> 2);
        let datatype = SoeType::try_from(((flags >> 4) & 7) as u8)?;
        let decimals = ((flags >> 8) & 15) as u8;
        let wppreop = (flags >> 12) & 1 != 0;
        let wpsafeop = (flags >> 13) & 1 != 0;
        let wpop = (flags >> 14) & 1 != 0;
        Ok(Self {
            eval_factor,
            length,
            idn_type,
            datatype,
            reserved1: (),
            decimals,
            wppreop,
            wpsafeop,
            wpop,
            reserved2: (),
        })
    }
}

impl SoeAttribute {
    pub const fn size() -> usize {
        size_of::<Ethercat<u32>>()
    }
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

    /// 3-bit opcode
    opcode: ServoOverEthercatOpcode,

    /// 1-bit
    incomplete: bool,

    /// 1-bit
    error: bool,

    /// 3-bit
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
            error: false,
            drive_number: 0,
            element_flags: 0,
            value: SoEValue::Idn(Ethercat::default()),
        }
    }
}

impl<R: Read> ReadFrom<R> for ServoOverEthercat {
    type Err = ServoOverEthercatError;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let mailbox_header = MailboxHeader::read_from(reader)?;
        let flags = Self::read_byte(reader)?;
        let opcode = ServoOverEthercatOpcode::try_from(flags & 7)?;
        let incomplete = (flags >> 3) & 1 != 0;
        let error = (flags >> 4) & 1 != 0;
        let drive_number = flags >> 5;
        let element_flags = Self::read_byte(reader)?;
        let value = SoEValue::FragmentsLeft(Ethercat::<u16>::from_bytes(Self::read_bytes(reader)?));
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

impl ServoOverEthercat {
    const fn size() -> usize {
        MailboxHeader::size() + 5 * size_of::<u8>() + size_of::<u16>()
    }

    pub fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
        self.mailbox_header.write_to(writer)?;
        writer.write_all(&[
            u8::from(self.opcode)
                | (u8::from(self.incomplete) << 3)
                | (u8::from(self.error) << 4)
                | (self.drive_number << 5),
            self.element_flags,
        ])?;
        writer.write_all(&self.value.inner().to_bytes())
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
/// # Errors
/// Returns an error if:
/// - Failed to send Servo over Ethercat request
/// - Received response wasn't a valid Servo over EtherCAT response
/// - Received an error code
/// - Received unexpected frame type
///
/// # Panics
/// Panics if:
/// Received mailbox header length is higher than the actual received response length.
///
/// # Returns
/// Bytes read or `Error`
pub fn soe_read(
    context: &mut Context,
    slave: u16,
    drive_number: u8,
    element_flags: u8,
    idn: Ethercat<u16>,
    parameter_buffer: &mut [u8],
    timeout: Duration,
) -> Result<u16, ServoOverEthercatError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox if something is in with timeout set to 0.
    mailbox_in.receive(context, slave, Duration::default())?;

    let soe = ServoOverEthercat {
        mailbox_header: MailboxHeader::new(
            Ethercat::from_host((ServoOverEthercat::size() - MailboxHeader::size()) as u16),
            Ethercat::from_host(0),
            0,
            u8::from(MailboxType::ServoOverEthercat)
                + mailbox_header_set_count(context.get_slave_mut(slave).mailbox_mut().next_count()),
        ),
        opcode: ServoOverEthercatOpcode::ReadRequest,
        incomplete: false,
        error: false,
        drive_number,
        element_flags,
        value: SoEValue::Idn(idn),
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
            context.packet_error(slave, idn.into_inner(), 0, PacketError::NoResponse);
            return Err(error.into());
        }

        // Slave response should be Servo over EtherCAT, read response
        let a_soe = ServoOverEthercat::read_from(&mut mailbox_in)?;
        if a_soe.mailbox_header.mailbox_type() & 0xF != u8::from(MailboxType::ServoOverEthercat)
            || a_soe.opcode != ServoOverEthercatOpcode::ReadResponse
            || a_soe.error
            || a_soe.drive_number != drive_number
            || a_soe.element_flags != element_flags
        {
            // Other slave response
            if a_soe.mailbox_header.mailbox_type() & 0xF == u8::from(MailboxType::ServoOverEthercat)
                && a_soe.opcode == ServoOverEthercatOpcode::ReadResponse
                && a_soe.error
            {
                let error_code = u16::from_ne_bytes(
                    (&mailbox_in.as_ref()[usize::from(a_soe.mailbox_header.length().to_host())
                        + MailboxHeader::size()
                        - size_of::<u16>()..])[..2]
                        .try_into()
                        .unwrap(),
                );
                context.servo_over_ethercat_error(slave, idn.into_inner(), error_code);
                return Err(ServoOverEthercatError::ErrorCodeReceived(error_code));
            }
            // Unexpected frame returned
            context.packet_error(
                slave,
                idn.into_inner(),
                0,
                PacketError::UnexpectedFrameReturned,
            );
            return Err(ServoOverEthercatError::UnexpectedFrameReturned);
        }
        let mut frame_data_size = a_soe.mailbox_header.length().to_host()
            + (MailboxHeader::size() - ServoOverEthercat::size()) as u16;
        total_size += frame_data_size;

        // If the parameter fits in the parameter buffer
        if usize::from(total_size) <= parameter_buffer.len() {
            // Copy parameter data in parameter buffer
            parameter_buffer[base_index..].copy_from_slice(&mailbox_in.as_ref()[mailbox_index..]);

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
    }
}

/// Servo over Ethercat write, blocking.
///
/// The IDN object of the selected slave and drive number is written. If a response
/// is larger than the mailbox size, the response is segmented.
///
/// # Parameters
/// `context`: Context struct
/// `slave`: Slave number
/// `drive_number`: Drive number in slave
/// `element_flags`: Flags to select what properties of IDN are to be transfered.
/// `idn`: IDN
/// `parameter_buffer`: Parameter buffer
/// `timeout`: Timeout duration, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Panics
/// - If an invalid was received
///
/// # Errors
/// Returns an error if:
/// - Failed to clear slave mailbox
/// - Failed to send data to slave
/// - Failed to check if slave mailbox is empty
/// - Received data that wasn't a valid Servo over Ethercat message
///
/// # Returns
/// `Ok(())` or error
pub fn soe_write(
    context: &mut Context,
    slave: u16,
    drive_number: u8,
    element_flags: u8,
    idn: u16,
    parameter_buffer: &mut [u8],
    timeout: Duration,
) -> Result<(), ServoOverEthercatError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox if something is in it, with timeout set to 0.
    mailbox_in.receive(context, slave, Duration::default())?;

    let mut buffer_index = 0;
    let max_data =
        usize::from(context.get_slave(slave).mailbox().length()) - size_of::<ServoOverEthercat>();
    loop {
        let mut frames_left = false;
        let mut frame_data_size = parameter_buffer[buffer_index..].len();
        let (incomplete, value) = if frame_data_size > max_data {
            frame_data_size = max_data;
            frames_left = true;
            (
                true,
                SoEValue::FragmentsLeft(Ethercat::from_host(
                    (parameter_buffer[buffer_index..].len() / max_data) as u16,
                )),
            )
        } else {
            (false, SoEValue::Idn(Ethercat::from_host(idn)))
        };

        // Create a servo over Ethercat write request
        let soe = ServoOverEthercat {
            mailbox_header: MailboxHeader::new(
                Ethercat::from_host(
                    (size_of::<ServoOverEthercat>() - size_of::<MailboxHeader>() + frame_data_size)
                        as u16,
                ),
                Ethercat::default(),
                0,
                // Get new mailbox counter, used for session handle
                u8::from(MailboxType::ServoOverEthercat)
                    + mailbox_header_set_count(
                        context.get_slave_mut(slave).mailbox_mut().next_count(),
                    ),
            ),
            opcode: ServoOverEthercatOpcode::WriteRequest,
            incomplete,
            error: false,
            drive_number,
            element_flags,
            value,
        };

        let mut mailbox_out = MailboxOut::default();
        soe.write_to(&mut mailbox_out)?;
        mailbox_out.write_all(&parameter_buffer[buffer_index..buffer_index + frame_data_size])?;
        buffer_index += frame_data_size;

        if frames_left && context.mailbox_empty(slave, timeout)? {
            continue;
        }

        // Clean mailboxbuffer
        mailbox_in.clear();

        // Read slave response
        if let Err(error) = mailbox_in.receive(context, slave, timeout) {
            context.packet_error(slave, idn, 0, PacketError::NoResponse);
            return Err(error.into());
        }

        // Slave response should be Servo over Ethercat Write response
        let a_soe = ServoOverEthercat::read_from(&mut mailbox_in)?;
        if a_soe.mailbox_header.mailbox_type() & 0xF == u8::from(MailboxType::ServoOverEthercat)
            && a_soe.opcode == ServoOverEthercatOpcode::WriteResponse
            && !a_soe.error
            && a_soe.drive_number == drive_number
            && a_soe.element_flags == element_flags
        {
            // Servo over EtherCAT succeeded
            return Ok(());
        }
        if a_soe.mailbox_header.mailbox_type() & 0xF == MailboxType::ServoOverEthercat.into()
            && a_soe.opcode == ServoOverEthercatOpcode::ReadResponse
            && a_soe.error
        {
            // Received an error
            let error_code = u16::from_ne_bytes(
                mailbox_in.as_ref()[usize::from(a_soe.mailbox_header.length().to_host())
                    + MailboxHeader::size()
                    - size_of::<u16>()..][..2]
                    .try_into()
                    .unwrap(),
            );
            context.servo_over_ethercat_error(slave, idn, error_code);
            return Err(ServoOverEthercatError::ErrorCodeReceived(error_code));
        }
        context.packet_error(slave, idn, 0, PacketError::UnexpectedFrameReturned);
        return Err(ServoOverEthercatError::UnexpectedFrameReturned);
    }
}

/// Servo over Ethercat read AT and MTD mapping.
///
/// Servo over Ethercat has standard indexes defined for mapping. This function
/// tries to read them and collect a full input and output mapping size of
/// designated slave.
///
/// # Parameters
/// `context`: Context struct
/// `slave`: Slave number
/// `output_size`: Size in bits of output mapping (MTD) found
/// `input_size`: Size in bits of input mapping (AT) found
///
/// # Returns
/// whether any IO was found
pub fn read_id_nmap(
    context: &mut Context,
    slave: u16,
    output_size: &mut u32,
    input_size: &mut u32,
) -> bool {
    *input_size = 0;
    *output_size = 0;
    for drive_number in 0..SOE_MAX_DRIVES {
        // Read output mapping via Servo over Ethercat
        let mut soe_mapping = SoeMapping::default();
        if soe_read(
            context,
            slave,
            drive_number,
            VALUE_B,
            Ethercat::from_host(u16::from(IDN_MDT_CONFIG)),
            soe_mapping.as_bytes_mut(),
            TIMEOUT_RX_MAILBOX,
        )
        .is_ok_and(|bytes_read| bytes_read >= 4)
        {
            let entries = soe_mapping.current_length.to_host() / 2;
            if (1..=MAX_MAPPING_LENGTH).contains(&usize::from(entries)) {
                // Command word (u16) is always mapped but not in list
                *output_size += 16;
                for item_count in 0..entries {
                    let mut bytes = [0; SoeAttribute::size()];
                    if soe_read(
                        context,
                        slave,
                        drive_number,
                        ATTRIBUTE_B,
                        soe_mapping.idn[usize::from(item_count)],
                        &mut bytes,
                        TIMEOUT_RX_MAILBOX,
                    )
                    .is_err()
                    {
                        continue;
                    }
                    let Some(soe_attribute) = SoeAttribute::read_from(&mut bytes.as_slice())
                        .ok()
                        .filter(|soe_attribute| !soe_attribute.idn_type.is_list())
                    else {
                        continue;
                    };
                    *output_size += 8 << soe_attribute.length;
                }
            }
        }

        if soe_read(
            context,
            slave,
            drive_number,
            VALUE_B,
            Ethercat::from_host(u16::from(IDN_AT_CONFIG)),
            soe_mapping.as_bytes_mut(),
            TIMEOUT_RX_MAILBOX,
        )
        .is_ok_and(|read_size| read_size >= 4)
        {
            let entries = soe_mapping.current_length.to_host() / 2;
            if (1..=MAX_MAPPING_LENGTH).contains(&usize::from(entries)) {
                // Status word (u16) is always mapped but not in list
                *input_size += 16;
                for item_count in 0..entries {
                    let mut bytes = [0; SoeAttribute::size()];
                    if soe_read(
                        context,
                        slave,
                        drive_number,
                        ATTRIBUTE_B,
                        soe_mapping.idn[usize::from(item_count)],
                        &mut bytes,
                        TIMEOUT_RX_MAILBOX,
                    )
                    .is_err()
                    {
                        continue;
                    }
                    let Ok(soe_attribute) = SoeAttribute::read_from(&mut bytes.as_slice()) else {
                        continue;
                    };
                    *input_size += 8 << soe_attribute.length;
                }
            }
        }
    }
    *input_size != 0 && *output_size != 0
}
