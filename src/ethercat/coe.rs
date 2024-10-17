use core::slice;
use core::str;
use std::array;
use std::mem::transmute;
use std::num::TryFromIntError;
use std::str::Utf8Error;
use std::time::Duration;
use std::time::SystemTime;

use heapless::String as HeaplessString;

use crate::ethercat::main::SyncManagerType;
use crate::ethercat::main::{next_mailbox_count, packet_error, MAX_SM, SYNC_MANAGER_ENABLE_MASK};
use crate::ethercat::r#type::{
    ethercat_to_host, host_to_ethercat, low_byte, mailbox_header_set_count, COEMailboxType,
    CanopenOverEthercatSdoCommand, MailboxType, SDO_PDO_ASSIGNMENT, SDO_SMCOMMTYPE,
    TIMEOUT_TX_MAILBOX,
};

use super::main::InvalidSyncManagerType;
use super::main::{
    push_error, Context, MailboxHeader, MailboxIn, MailboxOut, PacketError, MAX_NAME_LENGTH,
};
use super::r#type::Datatype;
use super::r#type::InvalidDataType;
use super::r#type::TIMEOUT_RX_MAILBOX;
use super::r#type::{AbortError, COEObjectDescriptionCommand, ErrorInfo, ErrorType, MailboxError};

/// Invalid Service Data Object size
#[derive(Debug)]
pub struct InvalidSDOSize;

/// Invalid Service Data Object Service size
#[derive(Debug)]
pub enum SDOServiceTryFromBytesError {
    InvalidSize(usize),
    InvalidOpcode,
}

#[derive(Debug)]
pub enum CoEError {
    InvalidSDOSize(InvalidSDOSize),
    SDOServiceTryFromBytes(SDOServiceTryFromBytesError),
    Mailbox(MailboxError),
    Packet(PacketError),
    Abort(AbortError),
    InvalidSyncManagerType(InvalidSyncManagerType),
    NoIoFoundInPdoMap,
    TryFromInt(TryFromIntError),
    InvalidDataType(InvalidDataType),
    Utf8(Utf8Error),
}

impl From<InvalidSDOSize> for CoEError {
    fn from(value: InvalidSDOSize) -> Self {
        Self::InvalidSDOSize(value)
    }
}

impl From<SDOServiceTryFromBytesError> for CoEError {
    fn from(value: SDOServiceTryFromBytesError) -> Self {
        Self::SDOServiceTryFromBytes(value)
    }
}

impl From<MailboxError> for CoEError {
    fn from(value: MailboxError) -> Self {
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

impl From<InvalidSyncManagerType> for CoEError {
    fn from(value: InvalidSyncManagerType) -> Self {
        Self::InvalidSyncManagerType(value)
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

#[allow(dead_code)]
enum ServiceData {
    Byte([u8; 0x200]),
    Word([u16; 0x100]),
    Long([u32; 0x80]),
}

impl ServiceData {
    pub const fn as_bytes(&self) -> &[u8] {
        match self {
            ServiceData::Byte(bytes) => bytes,
            ServiceData::Word(words) => unsafe { transmute::<&[u16; 256], &[u8; 512]>(words) },
            ServiceData::Long(longs) => unsafe { transmute::<&[u32; 128], &[u8; 512]>(longs) },
        }
    }

    pub const fn as_words(&self) -> &[u16] {
        match self {
            ServiceData::Byte(bytes) => unsafe { transmute::<&[u8; 512], &[u16; 256]>(bytes) },
            ServiceData::Word(words) => words,
            ServiceData::Long(longs) => unsafe { transmute::<&[u32; 128], &[u16; 256]>(longs) },
        }
    }

    pub const fn as_longs(&self) -> &[u32] {
        match self {
            ServiceData::Byte(bytes) => unsafe { transmute::<&[u8; 512], &[u32; 128]>(bytes) },
            ServiceData::Word(words) => unsafe { transmute::<&[u16; 256], &[u32; 128]>(words) },
            ServiceData::Long(longs) => longs,
        }
    }

    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        match self {
            ServiceData::Byte(bytes) => bytes,
            ServiceData::Word(words) => unsafe {
                transmute::<&mut [u16; 256], &mut [u8; 512]>(words)
            },
            ServiceData::Long(longs) => unsafe {
                transmute::<&mut [u32; 128], &mut [u8; 512]>(longs)
            },
        }
    }

    pub fn as_words_mut(&mut self) -> &mut [u16] {
        match self {
            ServiceData::Byte(bytes) => unsafe {
                transmute::<&mut [u8; 512], &mut [u16; 256]>(bytes)
            },
            ServiceData::Word(words) => words,
            ServiceData::Long(longs) => unsafe {
                transmute::<&mut [u32; 128], &mut [u16; 256]>(longs)
            },
        }
    }

    pub fn as_longs_mut(&mut self) -> &mut [u32] {
        match self {
            ServiceData::Byte(bytes) => unsafe {
                transmute::<&mut [u8; 512], &mut [u32; 128]>(bytes)
            },
            ServiceData::Word(words) => unsafe {
                transmute::<&mut [u16; 256], &mut [u32; 128]>(words)
            },
            ServiceData::Long(longs) => longs,
        }
    }
}

/// Service Data Object structure, not to be confused with `ServiceDataObjectService`

struct ServiceDataObject {
    mailbox_header: MailboxHeader,
    can_open: u16,
    command: u8,
    index: u16,
    sub_index: u8,
    data: ServiceData,
}

impl TryFrom<&mut [u8]> for &mut ServiceDataObject {
    type Error = InvalidSDOSize;

    fn try_from(value: &mut [u8]) -> Result<Self, Self::Error> {
        if value.len() < size_of::<Self>() {
            Err(InvalidSDOSize)
        } else {
            Ok(unsafe { &mut slice::from_raw_parts_mut((value as *mut [u8]).cast(), 1)[0] })
        }
    }
}

impl From<&mut ServiceDataObject> for &mut [u8] {
    fn from(value: &mut ServiceDataObject) -> Self {
        unsafe {
            slice::from_raw_parts_mut(
                (value as *mut ServiceDataObject).cast(),
                size_of::<ServiceDataObject>(),
            )
        }
    }
}

impl From<&ServiceDataObject> for &[u8] {
    fn from(value: &ServiceDataObject) -> Self {
        unsafe {
            slice::from_raw_parts(
                (value as *const ServiceDataObject).cast(),
                size_of::<ServiceDataObject>(),
            )
        }
    }
}

impl ServiceDataObject {
    pub const fn mailbox_header_offset(&self) -> usize {
        0
    }

    pub const fn can_open_offset(&self) -> usize {
        self.mailbox_header_offset() + size_of::<MailboxHeader>()
    }

    pub const fn command_offset(&self) -> usize {
        self.can_open_offset() + size_of::<u16>()
    }

    pub const fn index_offset(&self) -> usize {
        self.command_offset() + size_of::<u8>()
    }
}

impl ServiceDataObject {
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        self.into()
    }
}

/// Service Data Object service structure

struct ServiceDataObjectService {
    mailbox_header: MailboxHeader,
    can_open: u16,
    opcode: COEObjectDescriptionCommand,
    reserved: u8,
    fragments: u16,
    data: ServiceData,
}

impl TryFrom<&mut [u8]> for &mut ServiceDataObjectService {
    type Error = SDOServiceTryFromBytesError;

    fn try_from(value: &mut [u8]) -> Result<Self, Self::Error> {
        if value.len() < size_of::<Self>() {
            Err(SDOServiceTryFromBytesError::InvalidSize(value.len()))
        } else {
            let sdo_info: &mut Self =
                unsafe { &mut slice::from_raw_parts_mut((value as *mut [u8]).cast(), 1)[0] };
            if sdo_info.opcode.is_valid() {
                Ok(sdo_info)
            } else {
                Err(SDOServiceTryFromBytesError::InvalidOpcode)
            }
        }
    }
}

/// Max entries in object description list
pub const MAX_OBJECT_DESCRIPTION_LIST_SIZE: u16 = 1024;

/// Max entries in Object Entries list
pub const MAX_OBJECT_ENTRY_LIST_SIZE: usize = 256;

/// Storage for object description list

#[derive(Debug)]
pub struct ObjectDescriptionList {
    /// Slave number
    pub slave: u16,

    /// Number of entries in list
    pub entries: u16,

    /// Array of indexes
    pub index: [u16; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],

    /// Array of datatypes, see EtherCAT specification
    pub data_type: [Datatype; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],

    /// Array of object codes, see EtherCAT specification
    pub object_code: [u16; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],

    /// Number of subindexes for each index
    pub max_sub: [u8; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],

    pub name:
        [HeaplessString<{ MAX_NAME_LENGTH as usize }>; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],
}

impl Default for ObjectDescriptionList {
    fn default() -> Self {
        Self {
            slave: 0,
            entries: 0,
            index: [0; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],
            data_type: [Datatype::Invalid; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],
            object_code: [0; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],
            max_sub: [0; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize],
            name: array::from_fn(|_| HeaplessString::default()),
        }
    }
}

/// Storage for object list entry information

pub struct ObjectEntryList {
    /// Number of entries in list
    pub entries: u16,

    /// Array of value info, see EtherCAT specification
    pub value_info: [u8; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of value data types, see EtherCAT specification
    pub data_type: [Datatype; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of bit lengths, see EtherCAT specification
    pub bit_length: [u16; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of object access bits, see EtherCAT specification
    pub object_access: [u16; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Textual description of each index
    pub name: [HeaplessString<{ MAX_NAME_LENGTH as usize }>; MAX_OBJECT_ENTRY_LIST_SIZE],
}

/// Report Service Data Object error
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: slave number
/// - `index`: index that generated the error
/// - `sub_index`: Subindex that generated the error
/// - `abort_error`: Abort code or error, see EtherCAT documentation for list
pub fn sdo_error(
    context: &mut Context,
    slave: u16,
    index: u16,
    sub_index: u8,
    abort_error: AbortError,
) {
    *context.ecaterror.lock().unwrap() = true;
    push_error(
        context,
        ErrorInfo {
            time: SystemTime::now(),
            slave,
            index,
            sub_index,
            error_type: ErrorType::ServiceDataObjectError,
            abort_error,
            signal: false,
        },
    );
}

/// Report Service Data Object info error
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `index`: Index that generated the error
/// - `sub_index`: Subindex that generated the error
/// - `abort_error`: Abort code or error, see EtherCAT documentation for list
fn sdo_info_error(
    context: &mut Context,
    slave: u16,
    index: u16,
    sub_index: u8,
    abort_error: AbortError,
) {
    *context.ecaterror.lock().unwrap() = true;
    push_error(
        context,
        ErrorInfo {
            time: SystemTime::now(),
            signal: false,
            slave,
            index,
            sub_index,
            error_type: ErrorType::SdoInfoError,
            abort_error,
        },
    );
}

/// Stores the received abort error or an unexpected frame error
#[must_use]
fn handle_invalid_slave_response(
    a_sdo: &ServiceDataObject,
    context: &mut Context,
    slave: u16,
    index: u16,
    sub_index: u8,
) -> CoEError {
    if a_sdo.command == CanopenOverEthercatSdoCommand::Abort.into() {
        let abort_error = AbortError::Abort(ethercat_to_host(a_sdo.data.as_longs()[0] as i32));
        // SDO abort frame received
        sdo_error(context, slave, index, sub_index, abort_error);
        abort_error.into()
    } else {
        // Unexpected frame returned
        packet_error(
            context,
            slave,
            index,
            sub_index,
            PacketError::UnexpectedFrameReturned,
        );
        PacketError::UnexpectedFrameReturned.into()
    }
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
/// # Returns
/// Bytes read into `parameter_buffer` or error
pub fn sdo_read(
    context: &mut Context,
    slave: u16,
    index: u16,
    mut sub_index: u8,
    complete_access: bool,
    mut parameter_buffer: &mut [u8],
    timeout: Duration,
) -> Result<usize, CoEError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox, if something is in it. Timeout set to 0.
    mailbox_in.receive(context, slave, Duration::default())?;
    let mut mailbox_out = MailboxOut::default();
    let mut sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut())?;
    sdo.mailbox_header.length = host_to_ethercat(0xa);
    sdo.mailbox_header.address = host_to_ethercat(0);
    sdo.mailbox_header.priority = 0;

    // Get new mailbox count value, used as session handle
    let count = {
        let slavelist = &mut context.slavelist;
        let count = next_mailbox_count(slavelist[usize::from(slave)].mailbox_count);
        slavelist[usize::from(slave)].mailbox_count = count;
        count
    };
    sdo.mailbox_header.mailbox_type =
        u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count); // CANopen over EtherCAT

    // Number 9bits service, upper 4 bits (SDO request)
    sdo.can_open = host_to_ethercat(u16::from(COEMailboxType::SdoRequest) << 12);

    sdo.command = if complete_access {
        // Upload request complete access
        CanopenOverEthercatSdoCommand::UpReqCa.into()
    } else {
        // Upload request normal
        CanopenOverEthercatSdoCommand::UpReq.into()
    };
    sdo.index = host_to_ethercat(index);
    if complete_access {
        sub_index = sub_index.min(1);
    }
    sdo.sub_index = sub_index;
    sdo.data.as_longs_mut()[0] = 0;

    // Send CANopen over EtherCAT request to slave
    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

    // Clean mailbox buffer
    mailbox_in.clear();

    // Read slave response
    mailbox_in.receive(context, slave, timeout)?;

    // Slave response should be CANopen over EtherCAT, a Service Data Object response, and use the correct index
    let a_sdo = <&mut ServiceDataObject>::try_from(mailbox_in.as_mut())?;
    if matches!(
        MailboxType::try_from(a_sdo.mailbox_header.mailbox_type & 0xF)?,
        MailboxType::CanopenOverEthercat
    ) && matches!(
        COEMailboxType::try_from((ethercat_to_host(a_sdo.can_open) >> 12) as u8)?,
        COEMailboxType::SdoResponse
    ) && a_sdo.index == sdo.index
    {
        if a_sdo.command & 0x2 > 0 {
            let bytesize = 4 - a_sdo.command / 4 % 4;

            // Check if the parameter in the parameter buffer is big enough
            if parameter_buffer.len() >= usize::from(bytesize) {
                parameter_buffer.copy_from_slice(&a_sdo.data.as_bytes()[..usize::from(bytesize)]);
                parameter_buffer = &mut parameter_buffer[..usize::from(bytesize)];
            } else {
                // Data container too small for type
                packet_error(
                    context,
                    slave,
                    index,
                    sub_index,
                    PacketError::DataContainerTooSmallForType,
                );
                return Err(PacketError::DataContainerTooSmallForType.into());
            }
        } else {
            // Normal frame response
            let sdo_len = ethercat_to_host(a_sdo.data.as_longs()[0]) as usize;

            // Check whether the parameter fits in the parameter buffer
            if sdo_len <= parameter_buffer.len() {
                let mut hp = 0;

                // Calculate mailblox transfer size
                let frame_data_size =
                    usize::from(ethercat_to_host(a_sdo.mailbox_header.length) - 10);

                // Check whether the transfer is segmented
                if frame_data_size < sdo_len {
                    parameter_buffer[hp..].copy_from_slice(
                        &a_sdo.data.as_bytes()
                            [size_of::<u32>()..frame_data_size + size_of::<u32>()],
                    );

                    // Increment the buffer pointer
                    hp += frame_data_size;
                    let mut written = frame_data_size;
                    let mut segments_left = true;
                    let mut toggle = 0x00;
                    while segments_left {
                        sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut())?;
                        sdo.mailbox_header.length = host_to_ethercat(0xA);
                        sdo.mailbox_header.address = host_to_ethercat(0);
                        sdo.mailbox_header.priority = 0;
                        {
                            let slave = &mut context.slavelist[usize::from(slave)];
                            let count = next_mailbox_count(slave.mailbox_count);
                            slave.mailbox_count = count;
                            // CANopen over Ethercat
                            sdo.mailbox_header.mailbox_type =
                                u8::from(MailboxType::CanopenOverEthercat)
                                    + mailbox_header_set_count(count);
                        }

                        // Number 9 bits service 4 bits (SDO request)
                        sdo.can_open =
                            host_to_ethercat(u16::from(COEMailboxType::SdoRequest) << 12);

                        // Segment upload request
                        sdo.command = u8::from(CanopenOverEthercatSdoCommand::SegUpReq) + toggle;

                        sdo.index = host_to_ethercat(index);
                        sdo.sub_index = sub_index;
                        sdo.data.as_longs_mut()[0] = 0;

                        // Send segmented upload request to slave
                        mailbox_out.send(context, slave, timeout)?;

                        mailbox_in.clear();

                        // Read slave response
                        mailbox_in.receive(context, slave, timeout)?;

                        // Slave response should be CANopen over EtherCAT, SDO response
                        if matches!(
                            MailboxType::try_from(a_sdo.mailbox_header.mailbox_type & 0xF)?,
                            MailboxType::CanopenOverEthercat
                        ) && matches!(
                            COEMailboxType::try_from(
                                (ethercat_to_host(a_sdo.can_open) >> 12) as u8
                            )?,
                            COEMailboxType::SdoRequest
                        ) && sdo.command & 0xE0 == 0
                        {
                            // Calculate mailbox transfer size
                            let mut frame_data_size =
                                ethercat_to_host(a_sdo.mailbox_header.length) - 3;

                            // Check if this is the last segment
                            if a_sdo.command & 1 > 0 {
                                // Last segment
                                segments_left = false;
                                if frame_data_size == 7 {
                                    // Subtract unused bytes from frame
                                    frame_data_size -= u16::from((sdo.command & 0xE) >> 1);
                                }

                                // Copy to parameter buffer
                                let index_offset = a_sdo.index_offset();
                                parameter_buffer[hp..].copy_from_slice(
                                    &a_sdo.as_bytes_mut()
                                        [index_offset..index_offset + usize::from(frame_data_size)],
                                );
                            } else {
                                // Segments follow

                                // Copy to parameter buffer
                                let index_offset = a_sdo.index_offset();
                                parameter_buffer[hp..].copy_from_slice(
                                    &a_sdo.as_bytes_mut()
                                        [index_offset..index_offset + usize::from(frame_data_size)],
                                );

                                // Increment buffer pointer
                                hp += usize::from(frame_data_size);
                            }

                            // Update parameter size
                            written += usize::from(frame_data_size);
                        } else {
                            // Unexpected frame returned from slave
                            return Err(handle_invalid_slave_response(
                                a_sdo, context, slave, index, sub_index,
                            ));
                        }
                        toggle ^= 0x10;
                    }
                    parameter_buffer = &mut parameter_buffer[..written];
                } else {
                    // Non segmented transfer
                    parameter_buffer.copy_from_slice(
                        &a_sdo.data.as_bytes()[size_of::<u32>()..sdo_len + size_of::<u32>()],
                    );
                    parameter_buffer = &mut parameter_buffer[..sdo_len];
                }
            } else {
                // Data container too small for type
                packet_error(
                    context,
                    slave,
                    index,
                    sub_index,
                    PacketError::DataContainerTooSmallForType,
                );
                return Err(PacketError::DataContainerTooSmallForType.into());
            }
        }
    } else {
        // Other slave response

        // SDO abort frame received
        return Err(handle_invalid_slave_response(
            a_sdo, context, slave, index, sub_index,
        ));
    }

    Ok(parameter_buffer.len())
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
/// # Returns
/// Unit or error
pub fn sdo_write(
    context: &mut Context,
    slave: u16,
    index: u16,
    sub_index: u8,
    complete_access: bool,
    mut parameter_buffer: &mut [u8],
    timeout: Duration,
) -> Result<(), CoEError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox if something is in it. With timeout set to default.
    mailbox_in.receive(context, slave, Duration::default())?;

    let mut mailbox_out = MailboxOut::default();
    let a_sdo = <&mut ServiceDataObject>::try_from(mailbox_in.as_mut())?;
    let sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut())?;

    // Data section = mailbox size - 6 mailbox - 2 CANopen over EtherCAT - 8 service data object requests
    let mut maxdata = context.slavelist[usize::from(slave)].mailbox_length - 0x10;

    // For small data use expedited transfer
    if parameter_buffer.len() <= 4 && !complete_access {
        sdo.mailbox_header.length = host_to_ethercat(0xA);
        sdo.mailbox_header.address = host_to_ethercat(0);
        sdo.mailbox_header.priority = 0;

        {
            let slave_object = &mut context.slavelist[usize::from(slave)];

            // Get new mailbox counter, used for session handle
            let count = next_mailbox_count(slave_object.mailbox_count);
            slave_object.mailbox_count = count;
            sdo.mailbox_header.mailbox_type =
                u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count);
        }
        sdo.can_open = host_to_ethercat(u16::from(COEMailboxType::SdoRequest) << 12);
        sdo.command = u8::from(CanopenOverEthercatSdoCommand::DownExp)
            | (((4 - parameter_buffer.len()) << 2) & 0xC) as u8;
        sdo.index = host_to_ethercat(index);
        sdo.sub_index = sub_index;

        // Copy parameter data to mailbox
        sdo.data.as_bytes_mut().copy_from_slice(parameter_buffer);

        // Send mailbox SDO download request to slave
        mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

        mailbox_in.clear();

        // Read slave response
        mailbox_in.receive(context, slave, timeout)?;

        if !(matches!(
            MailboxType::try_from(a_sdo.mailbox_header.mailbox_type & 0xF)?,
            MailboxType::CanopenOverEthercat
        ) && matches!(
            COEMailboxType::try_from(ethercat_to_host(a_sdo.can_open >> 12) as u8)?,
            COEMailboxType::SdoResponse
        ) && a_sdo.index == sdo.index
            && a_sdo.sub_index == sdo.sub_index)
        {
            return Err(handle_invalid_slave_response(
                a_sdo, context, slave, index, sub_index,
            ));
        }
    } else {
        let (frame_data_size, mut segments_left) = if parameter_buffer.len() <= maxdata.into() {
            (parameter_buffer.len() as u16, false)
        } else {
            (maxdata, true)
        };
        sdo.mailbox_header.length = host_to_ethercat(0xA + frame_data_size);
        sdo.mailbox_header.address = host_to_ethercat(0);
        sdo.mailbox_header.priority = 0;

        // Get new mailbox counter, used for session handle
        {
            let slave_object = &mut context.slavelist[usize::from(slave)];
            let count = next_mailbox_count(slave_object.mailbox_count);
            slave_object.mailbox_count = count;
            sdo.mailbox_header.mailbox_type =
                u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count);
            sdo.can_open = host_to_ethercat(u16::from(COEMailboxType::SdoRequest) << 12);
        }
        sdo.command = if complete_access {
            // Complete access, normal Service Data Object init download transfer
            CanopenOverEthercatSdoCommand::DownInitCa
        } else {
            // Normal Service Data Object init download transfer
            CanopenOverEthercatSdoCommand::DownInit
        }
        .into();

        sdo.index = host_to_ethercat(index);
        sdo.sub_index = if complete_access {
            sub_index.min(1)
        } else {
            sub_index
        };
        sdo.data.as_longs_mut()[0] = host_to_ethercat(parameter_buffer.len()) as u32;

        // Copy data to mailbox
        sdo.data.as_bytes_mut()[size_of::<u32>()..]
            .copy_from_slice(&parameter_buffer[..usize::from(frame_data_size)]);
        parameter_buffer = &mut parameter_buffer[usize::from(frame_data_size)..];

        // Send mailbox SDO download request to slave
        mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

        mailbox_in.clear();

        // Read slave response
        mailbox_in.receive(context, slave, timeout)?;

        // Response should be a SDO response send over CANopen over EtherCAT, containing the correct index and subindex
        if matches!(
            MailboxType::try_from(a_sdo.mailbox_header.mailbox_type & 0xF)?,
            MailboxType::CanopenOverEthercat
        ) && matches!(
            COEMailboxType::try_from(ethercat_to_host(a_sdo.can_open >> 12) as u8)?,
            COEMailboxType::SdoResponse
        ) && a_sdo.index == sdo.index
            && a_sdo.sub_index == sdo.sub_index
        {
            // All ok
            maxdata += 7;
            let mut toggle = 0;

            // Repeat while there are segments left
            while segments_left {
                // Last segment
                segments_left = false;
                sdo.command = 1;
                if frame_data_size > maxdata {
                    segments_left = true;
                    sdo.command = 0;
                }
                if segments_left && frame_data_size >= 7 {
                    sdo.mailbox_header.length = host_to_ethercat(frame_data_size + 3);
                } else {
                    sdo.mailbox_header.length = host_to_ethercat(0xA);
                    sdo.command = (1 + ((7 - frame_data_size) << 1)) as u8;
                }

                sdo.mailbox_header.address = host_to_ethercat(0);
                sdo.mailbox_header.priority = 0;

                // Get new mailbox counter value
                {
                    let slave_object = &mut context.slavelist[usize::from(slave)];
                    let count = next_mailbox_count(slave_object.mailbox_count);
                    slave_object.mailbox_count = count;
                    sdo.mailbox_header.mailbox_type = u8::from(MailboxType::CanopenOverEthercat)
                        + mailbox_header_set_count(count);
                }

                // Service data object request
                sdo.can_open = host_to_ethercat(u16::from(COEMailboxType::SdoRequest) << 12);

                // Add toggle to command byte
                sdo.command += toggle;

                // Copy parameter data to mailbox
                let index_offset = a_sdo.index_offset();
                sdo.as_bytes_mut()[index_offset..]
                    .copy_from_slice(&parameter_buffer[..usize::from(frame_data_size)]);
                parameter_buffer = &mut parameter_buffer[usize::from(frame_data_size)..];

                // Send Service Data Object download request
                mailbox_out.send(context, slave, timeout)?;

                mailbox_in.clear();

                // Read slave response
                mailbox_in.receive(context, slave, timeout)?;
                if !(matches!(
                    MailboxType::try_from(sdo.mailbox_header.mailbox_type & 0xF)?,
                    MailboxType::CanopenOverEthercat
                ) && matches!(
                    COEMailboxType::try_from(ethercat_to_host(a_sdo.can_open >> 12) as u8)?,
                    COEMailboxType::SdoResponse
                ) && a_sdo.command & 0xE0 == 0x20)
                {
                    return Err(handle_invalid_slave_response(
                        a_sdo, context, slave, index, sub_index,
                    ));
                }
                toggle ^= 0x10;
            }
        } else {
            return Err(handle_invalid_slave_response(
                a_sdo, context, slave, index, sub_index,
            ));
        }
    }
    Ok(())
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
/// # Returns
/// Unit or an error
pub fn rx_pdo(
    context: &mut Context,
    slave: u16,
    rx_pdo_number: u16,
    pdo_buffer: &[u8],
) -> Result<(), CoEError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox, if something is in it. With timeout set to default.
    mailbox_in.receive(context, slave, Duration::default())?;

    let mut mailbox_out = MailboxOut::default();
    let sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut())?;

    // Data section = mailbox size - 6 mailbox - 2 CANopen over EtherCAT
    let max_data = context.slavelist[usize::from(slave)].mailbox_length - 8;
    let framedatasize = (pdo_buffer.len() as u16).min(max_data);
    sdo.mailbox_header.length = host_to_ethercat(2 + framedatasize);
    sdo.mailbox_header.address = host_to_ethercat(0);
    sdo.mailbox_header.priority = host_to_ethercat(0);

    // Get new mailbox counter, used for session handle
    {
        let slave_object = &mut context.slavelist[usize::from(slave)];
        let count = next_mailbox_count(slave_object.mailbox_count);
        slave_object.mailbox_count = count;
        sdo.mailbox_header.mailbox_type =
            u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count);
    }
    sdo.can_open =
        host_to_ethercat(rx_pdo_number & 0x1FF) + (u16::from(COEMailboxType::RxPdo) << 12);

    // Copy PDO data to mailbox
    let command_offset = sdo.command_offset();
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
/// # Returns
/// Bytes written to pdo or error
pub fn tx_pdo(
    context: &mut Context,
    slave: u16,
    tx_pdo_number: u16,
    mut pdo_buffer: &mut [u8],
    timeout: Duration,
) -> Result<usize, CoEError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave output mailbox if something is in it. With timeout set to 0.
    mailbox_in.receive(context, slave, Duration::default())?;
    let mut mailbox_out = MailboxOut::default();
    let sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut())?;
    sdo.mailbox_header.length = host_to_ethercat(2);
    sdo.mailbox_header.address = host_to_ethercat(0);
    sdo.mailbox_header.priority = 0;

    // Get new mailbox counter, used for session handle
    {
        let slave = &mut context.slavelist[usize::from(slave)];
        let count = next_mailbox_count(slave.mailbox_count);
        sdo.mailbox_header.mailbox_type =
            u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count);
    }
    // Number 9bits service upper 4 bits
    sdo.can_open =
        host_to_ethercat(tx_pdo_number & 0x1FF) + ((COEMailboxType::TxPdoRR as u16) << 12);

    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

    // Clear mailboxbuffer
    mailbox_in.clear();

    // Read slave response
    mailbox_in.receive(context, slave, timeout)?;

    let a_sdo = <&mut ServiceDataObject>::try_from(mailbox_in.as_mut())?;

    if a_sdo.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::CanopenOverEthercat)
        && ethercat_to_host(a_sdo.can_open) >> 12 == u16::from(COEMailboxType::TxPdo)
    {
        // TxPDO response
        let framedatasize = a_sdo.mailbox_header.length - 2;

        if pdo_buffer.len() >= framedatasize.into() {
            // If the parameterbuffer is large enough

            // Copy parameter in parameter buffer
            let command_offset = a_sdo.command_offset();
            pdo_buffer.copy_from_slice(
                &mailbox_in.as_ref()[command_offset..command_offset + usize::from(framedatasize)],
            );
            pdo_buffer = &mut pdo_buffer[..usize::from(framedatasize)];
        } else {
            // Dara container too small for type
            packet_error(
                context,
                slave,
                0,
                0,
                PacketError::DataContainerTooSmallForType,
            );
            return Err(PacketError::DataContainerTooSmallForType.into());
        }
    } else {
        // Other slave response

        // SDO abort frame received
        if a_sdo.command == u8::from(CanopenOverEthercatSdoCommand::Abort) {
            let abort_error = AbortError::Abort(ethercat_to_host(a_sdo.data.as_longs()[0]) as i32);
            sdo_error(
                context,
                slave,
                0,
                0,
                AbortError::Abort(ethercat_to_host(a_sdo.data.as_longs()[0]) as i32),
            );
            return Err(abort_error.into());
        } else {
            packet_error(
                context,
                slave,
                0,
                0,
                PacketError::DataContainerTooSmallForType,
            );
            return Err(PacketError::DataContainerTooSmallForType.into());
        }
    }
    Ok(pdo_buffer.len())
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
    let mut read_word = [0; 2];
    sdo_read(
        context,
        slave,
        pdo_assign,
        0,
        false,
        &mut read_word,
        TIMEOUT_RX_MAILBOX,
    )?;
    let read_data = ethercat_to_host(u16::from_ne_bytes(read_word));

    // Check whether the response from slave was positive
    if read_data == 0 {
        return Ok(0);
    }

    // Number of available sub indexes
    let index_count = read_data;
    (1..=index_count)
        .map(|index_loop| {
            // Read PDO assign
            sdo_read(
                context,
                slave,
                pdo_assign,
                index_loop as u8,
                false,
                &mut read_word,
                TIMEOUT_RX_MAILBOX,
            )?;

            // Result is index of PDO
            let index = ethercat_to_host(u16::from_ne_bytes(read_word));
            if index == 0 {
                return Ok(0);
            }

            let mut sub_count = [0];
            sdo_read(
                context,
                slave,
                index,
                0,
                false,
                &mut sub_count,
                TIMEOUT_RX_MAILBOX,
            )?;
            let sub_index = u8::from_ne_bytes(sub_count);
            (1..=sub_index)
                .map(|sub_index_loop| {
                    sdo_read(
                        context,
                        slave,
                        index,
                        sub_index_loop,
                        false,
                        &mut read_word,
                        TIMEOUT_RX_MAILBOX,
                    )?;
                    let read_data = ethercat_to_host(u16::from_ne_bytes(read_word));

                    // Extract bitlength of SDO
                    if low_byte(read_data) < 0xFF {
                        Ok(low_byte(read_data) as u32)
                    } else {
                        // Used `readOEsingle(idx, (uint8)SubCount, pODlist, pOElist);` in old SOEM
                        // codebase, but is already outcomented in current original SOEM codebase.
                        Ok(0xFF)
                    }
                })
                .sum::<Result<u32, CoEError>>()
        })
        .sum()
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
    let mut pdo_assign_copy = context.pdo_assign[thread_number].clone();
    pdo_assign_copy.number = 0;

    // Read rxPDOassign in complete access mode, all subindexes areread in one struct
    sdo_read(
        context,
        slave,
        pdo_assign,
        0,
        true,
        <&mut [u8]>::from(&mut pdo_assign_copy),
        TIMEOUT_RX_MAILBOX,
    )?;

    if pdo_assign_copy.number == 0 {
        return Ok(0);
    }

    let bit_length = pdo_assign_copy
        .index
        .iter()
        .copied()
        .map(ethercat_to_host)
        .filter(|index| *index > 0)
        .map(|index| {
            let mut pdo_description = context.pdo_description[thread_number].clone();
            pdo_description.number = 0;
            sdo_read(
                context,
                slave,
                index,
                0,
                true,
                <&mut [u8]>::from(&mut pdo_description),
                TIMEOUT_RX_MAILBOX,
            )?;

            // Extract all bitlengths of SDO'
            let bit_length = pdo_description
                .pdo
                .iter()
                .copied()
                .map(|long| low_byte(ethercat_to_host(long) as u16) as u32)
                .sum::<u32>();

            context.pdo_description[thread_number] = pdo_description;
            Ok(bit_length)
        })
        .sum::<Result<u32, CoEError>>()?;
    context.pdo_assign[thread_number] = pdo_assign_copy;

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
/// # Returns
/// Unit if successful, error otherwise
pub fn read_pdo_map(
    context: &mut Context,
    slave: u16,
    output_size: &mut u32,
    input_size: &mut u32,
) -> Result<(), CoEError> {
    let mut byte = [0];
    sdo_read(
        context,
        slave,
        SDO_SMCOMMTYPE,
        0,
        false,
        &mut byte,
        TIMEOUT_RX_MAILBOX,
    )?;

    *input_size = 0;
    *output_size = 0;

    // Limit the maximum number of Sync Manager defined
    let number_sync_manager = u8::from_ne_bytes(byte).min(MAX_SM);

    // Valid result from slave
    if number_sync_manager <= 2 {
        return Err(CoEError::NoIoFoundInPdoMap);
    }

    let mut sync_manager_bug_add = 0;

    for input_sync_manager in 2..number_sync_manager {
        sdo_read(
            context,
            slave,
            SDO_SMCOMMTYPE,
            input_sync_manager + 1,
            false,
            &mut byte,
            TIMEOUT_RX_MAILBOX,
        )?;
        let mut sync_manager_type = u8::from_ne_bytes(byte);

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

        context.slavelist[usize::from(slave)].sync_manager_type[usize::from(input_sync_manager)] =
            SyncManagerType::try_from(sync_manager_type)?;

        if sync_manager_type == 0 {
            let flags = &mut context.slavelist[usize::from(slave)].sync_manager
                [usize::from(input_sync_manager)]
            .sm_flags;
            *flags = host_to_ethercat(ethercat_to_host(*flags) & SYNC_MANAGER_ENABLE_MASK);
        }
        if !matches!(sync_manager_type, 3 | 4) {
            continue;
        }
        // Read the assign PDO
        let mapping_bit_size = read_pdo_assign(
            context,
            slave,
            SDO_PDO_ASSIGNMENT + u16::from(input_sync_manager),
        )?;

        // Skip if the mapping isn't found
        if mapping_bit_size == 0 {
            continue;
        }

        context.slavelist[usize::from(slave)].sync_manager[usize::from(input_sync_manager)]
            .sm_length = host_to_ethercat(mapping_bit_size.div_ceil(8) as u16);
        *if sync_manager_type == 3 {
            // It's an output mapping
            &mut *output_size
        } else {
            // It's an input mapping
            &mut *input_size
        } += mapping_bit_size;
    }

    if *input_size == 0 && *output_size == 0 {
        Err(CoEError::NoIoFoundInPdoMap)
    } else {
        Ok(())
    }
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
/// # Returns
/// Unit or error
pub fn read_pdo_map_complete_access(
    context: &mut Context,
    slave: u16,
    thread_number: usize,
    output_size: &mut u32,
    input_size: &mut u32,
) -> Result<(), CoEError> {
    (*input_size, *output_size) = (0, 0);
    context.sync_manager_communication_type[thread_number].number = 0;

    let mut sync_manager_communication_type =
        context.sync_manager_communication_type[thread_number].clone();

    // Read sync manager communication type object count with complete access
    sdo_read(
        context,
        slave,
        SDO_SMCOMMTYPE,
        0,
        true,
        <&mut [u8]>::from(&mut sync_manager_communication_type),
        TIMEOUT_RX_MAILBOX,
    )?;

    // Check whether the result from slave was positive
    if sync_manager_communication_type.number <= 2 {
        return Err(CoEError::NoIoFoundInPdoMap);
    }

    let number_sync_manager = sync_manager_communication_type.number;
    if number_sync_manager > MAX_SM {
        packet_error(context, slave, 0, 0, PacketError::TooManySyncManagers);
        return Err(PacketError::TooManySyncManagers.into());
    }

    let mut sync_manager_bug_add = 0;
    for input_sync_manager in 2..number_sync_manager {
        let mut sync_manager_type = u8::from(
            sync_manager_communication_type.sync_manager_type[usize::from(input_sync_manager)],
        );

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

        context.slavelist[usize::from(slave)].sync_manager_type[usize::from(input_sync_manager)] =
            SyncManagerType::try_from(sync_manager_type)?;

        // Check if SyncManagr is unused -> clear enable flag
        if sync_manager_type == 0 {
            let flags = &mut context.slavelist[usize::from(slave)].sync_manager
                [usize::from(input_sync_manager)]
            .sm_flags;
            *flags = host_to_ethercat(ethercat_to_host(*flags) & SYNC_MANAGER_ENABLE_MASK);
        }
        if !matches!(sync_manager_type, 3 | 4) {
            continue;
        }

        // Read the assign PDO
        let mapping_size = read_pdo_assign_complete_access(
            context,
            slave,
            thread_number,
            SDO_PDO_ASSIGNMENT + u16::from(input_sync_manager),
        )?;

        // Skip if no mapping was found
        if mapping_size == 0 {
            continue;
        }

        context.slavelist[usize::from(slave)].sync_manager[usize::from(input_sync_manager)]
            .sm_length = host_to_ethercat(mapping_size.div_ceil(8) as u16);

        *if sync_manager_type == 3 {
            // It's an output mapping
            &mut *output_size
        } else {
            // It's an input mapping
            &mut *input_size
        } += mapping_size;
    }

    if *input_size == 0 && *output_size == 0 {
        Err(CoEError::NoIoFoundInPdoMap)
    } else {
        Ok(())
    }
}

/// CANopen over EtherCAT read Object Description List
///
/// # Parameters
/// - `context`: context struct
/// - `slave`: slave number
///
/// # Returns
/// Unit or error
pub fn read_od_list(context: &mut Context, slave: u16) -> Result<ObjectDescriptionList, CoEError> {
    let mut mailbox_in = MailboxIn::default();

    // Clear pending out mailbox in slave if available with timeout set to 0.
    mailbox_in.receive(context, slave, Duration::default())?;
    let mut mailbox_out = MailboxOut::default();
    let sdo = <&mut ServiceDataObjectService>::try_from(mailbox_out.as_mut())?;
    sdo.mailbox_header.length = host_to_ethercat(8);
    sdo.mailbox_header.address = host_to_ethercat(0);
    sdo.mailbox_header.priority = 0;

    // Get new mailbox counter value
    {
        let slave = &mut context.slavelist[usize::from(slave)];
        let count = next_mailbox_count(slave.mailbox_count);
        sdo.mailbox_header.mailbox_type =
            u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count);
    }
    // Number 9 bits service upper 4 bits
    sdo.can_open = host_to_ethercat(u16::from(COEMailboxType::SdoInfo) << 12);

    // Get object description list request
    sdo.opcode = COEObjectDescriptionCommand::ObjectDesciptionListRequest;
    sdo.reserved = 0;

    // Fragments left
    sdo.fragments = 0;

    // All objects
    sdo.data.as_words_mut()[0] = host_to_ethercat(1);

    // Send get object description list request to slave
    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

    let a_sdo = <&mut ServiceDataObjectService>::try_from(mailbox_in.as_mut())?;
    let mut first = true;
    let mut sp = 0;
    let mut entries = 0;
    let mut offset = 0;
    let mut errors = 0;
    let mut result: Result<(), CoEError> = Ok(());
    let mut index = [0; MAX_OBJECT_DESCRIPTION_LIST_SIZE as usize];
    loop {
        // Assume this is the last iteration
        let mut stop = true;
        mailbox_in.clear();

        // Read slave response
        mailbox_in.receive(context, slave, TIMEOUT_RX_MAILBOX)?;

        // Response should be CANopen over EtherCAT and the get object description list response
        if a_sdo.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::CanopenOverEthercat)
            && matches!(
                a_sdo.opcode,
                COEObjectDescriptionCommand::ObjectDesciptionListRequest
            )
        {
            let mut index_count = if first {
                // Extract number of indexes from mailbox data size
                (ethercat_to_host(a_sdo.mailbox_header.length) - (6 + 2)) / 2
            } else {
                (ethercat_to_host(a_sdo.mailbox_header.length) - 6) / 2
            };

            // Check if indexes fit in buffer structure
            if sp + index_count > MAX_OBJECT_DESCRIPTION_LIST_SIZE {
                index_count = MAX_OBJECT_DESCRIPTION_LIST_SIZE + 1 - sp;
                sdo_info_error(context, slave, 0, 0, AbortError::TooManyMasterBufferEntries);
                stop = true;
                result = Err(AbortError::TooManyMasterBufferEntries.into());
            }

            // Trim to maximum number of Object Description list entries defined
            if entries + index_count > MAX_OBJECT_DESCRIPTION_LIST_SIZE {
                index_count = MAX_OBJECT_DESCRIPTION_LIST_SIZE - entries;
            }
            entries += index_count;
            index[usize::from(sp)..]
                .iter_mut()
                .zip(
                    a_sdo.data.as_words()[offset..]
                        .iter()
                        .copied()
                        .map(ethercat_to_host),
                )
                .take(index_count.into())
                .for_each(|(dest, src)| *dest = src);
            sp += index_count;
            // Check if more fragments will follow
            if a_sdo.fragments > 0 {
                stop = false;
            }
            first = false;
            offset = 0;
        } else {
            // Got unexpected response from slave
            if matches!(
                a_sdo.opcode,
                COEObjectDescriptionCommand::ServiceDataObjectInformationError
            ) {
                // SDO info error received
                let abort_error =
                    AbortError::Abort(ethercat_to_host(a_sdo.data.as_longs()[0]) as i32);
                sdo_info_error(context, slave, 0, 0, abort_error);
                result = Err(abort_error.into());
                stop = true;
            } else {
                packet_error(context, slave, 0, 0, PacketError::UnexpectedFrameReturned);
            }
            errors += 1;
        }
        if errors > 6 || stop {
            break;
        }
    }

    if errors > 6 && result.is_ok() {
        Err(PacketError::UnexpectedFrameReturned.into())
    } else if let Err(err) = result {
        Err(err)
    } else {
        Ok(ObjectDescriptionList {
            slave,
            entries,
            index,
            ..Default::default()
        })
    }
}

/// CANopen over EtherCAT read Object Description. Adds textual information to object indexes.
///
/// # Parameters
/// - `context`: Context struct
/// - `item`: Item number in Object  Description List
/// - `od_list`: Referencing Object Description list
///
/// # Returns
/// Unit or error
pub fn read_od_description(
    context: &mut Context,
    item: u16,
    od_list: &mut ObjectDescriptionList,
) -> Result<(), CoEError> {
    let slave = od_list.slave;
    let item = usize::from(item);

    let mut mailbox_in = MailboxIn::default();

    // Clear pending out mailbox in slave if available with timeout set to default.
    mailbox_in.receive(context, slave, Duration::default())?;
    let mut mailbox_out = MailboxOut::default();
    let sdo = <&mut ServiceDataObjectService>::try_from(mailbox_out.as_mut())?;
    sdo.mailbox_header.length = host_to_ethercat(8);
    sdo.mailbox_header.address = host_to_ethercat(0);
    sdo.mailbox_header.priority = 0;

    // Get new mailbox counter value
    {
        let slave = &mut context.slavelist[usize::from(slave)];
        let count = next_mailbox_count(slave.mailbox_count);
        slave.mailbox_count = count;
        sdo.mailbox_header.mailbox_type =
            u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count);
    }
    // Number 9 bits service upper 4 bits
    sdo.can_open = host_to_ethercat(u16::from(COEMailboxType::SdoInfo) << 12);

    // Get object description request
    sdo.opcode = COEObjectDescriptionCommand::ObjectDesciptionRequest;

    sdo.reserved = 0;

    // Fragments left
    sdo.fragments = 0;

    // Data of index
    sdo.data.as_words_mut()[0] = host_to_ethercat(od_list.index[item]);

    // Send get request to slave
    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

    // Read slave response
    mailbox_in.clear();
    mailbox_in.receive(context, slave, TIMEOUT_RX_MAILBOX)?;

    let a_sdo = <&mut ServiceDataObjectService>::try_from(mailbox_in.as_mut())?;

    if a_sdo.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::CanopenOverEthercat)
        && matches!(
            a_sdo.opcode,
            COEObjectDescriptionCommand::ObjectDesciptionResponse
        )
    {
        let object_name_length =
            (ethercat_to_host(a_sdo.mailbox_header.length) - 12).min(MAX_NAME_LENGTH);
        od_list.data_type[item] =
            Datatype::try_from(u8::try_from(ethercat_to_host(a_sdo.data.as_words()[1]))?)?;
        od_list.object_code[item] = u16::from(a_sdo.data.as_bytes()[5]);
        od_list.max_sub[item] = a_sdo.data.as_bytes()[4];
        od_list.name[item].clear();
        od_list.name[item]
            .push_str(str::from_utf8(
                &a_sdo.data.as_bytes()[..usize::from(object_name_length)],
            )?)
            .unwrap();
    } else {
        // Got unexpected response from slave

        // SDO info error received
        if matches!(
            a_sdo.opcode,
            COEObjectDescriptionCommand::ServiceDataObjectInformationError
        ) {
            let abort_error = AbortError::Abort(ethercat_to_host(a_sdo.data.as_longs()[0]) as i32);
            sdo_info_error(context, slave, od_list.index[item], 0, abort_error);
            return Err(abort_error.into());
        } else {
            packet_error(
                context,
                slave,
                od_list.index[item],
                0,
                PacketError::UnexpectedFrameReturned,
            );
            return Err(PacketError::UnexpectedFrameReturned.into());
        }
    }
    Ok(())
}

/// CANopen read SDO Service object entry, single subindex.
/// Used in `read_oe`.
///
/// # Parameters
/// - `context`: Context struct
/// - `item`: Item in Object Description List
/// - `sub_index`: Subindex of item in Object Description List
/// - `od_list`: Object Description list for reference
/// - `oe_list`: Resulting object entry structure
///
/// # Returns
/// Unit or error
pub fn read_oe_single(
    context: &mut Context,
    item: u16,
    sub_index: u8,
    od_list: &mut ObjectDescriptionList,
    oe_list: &mut ObjectEntryList,
) -> Result<(), CoEError> {
    let slave = od_list.slave;
    let index = od_list.index[usize::from(item)];
    let mut mailbox_in = MailboxIn::default();

    // Clear pending out mailbox in slave if available, with timeout set to default.
    mailbox_in.receive(context, slave, Duration::default())?;

    let mut mailbox_out = MailboxOut::default();
    let sdo = <&mut ServiceDataObjectService>::try_from(mailbox_out.as_mut())?;
    sdo.mailbox_header.length = host_to_ethercat(0xA);
    sdo.mailbox_header.address = host_to_ethercat(0);
    sdo.mailbox_header.priority = 0;

    // Get new mailbox counter value
    {
        let slave = &mut context.slavelist[usize::from(slave)];
        let count = next_mailbox_count(slave.mailbox_count);
        sdo.mailbox_header.mailbox_type =
            u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count);
    }

    sdo.can_open = host_to_ethercat(u16::from(COEMailboxType::SdoInfo) << 12);

    // Get object entry description request
    sdo.opcode = COEObjectDescriptionCommand::ObjectEntryRequest;
    sdo.reserved = 0;

    // Fragments left
    sdo.fragments = 0;
    sdo.data.as_words_mut()[0] = host_to_ethercat(index);
    sdo.data.as_bytes_mut()[2] = sub_index;
    // Get access rights, object category, PDO
    sdo.data.as_bytes_mut()[3] = 1 + 2 + 4;

    // Send get object entry description request to slave
    mailbox_out.send(context, slave, TIMEOUT_TX_MAILBOX)?;

    // Read slave response
    mailbox_in.receive(context, slave, TIMEOUT_RX_MAILBOX)?;

    let a_sdo = <&mut ServiceDataObjectService>::try_from(mailbox_in.as_mut())?;

    if a_sdo.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::CanopenOverEthercat)
        && matches!(
            a_sdo.opcode,
            COEObjectDescriptionCommand::ObjectEntryResponse
        )
    {
        oe_list.entries += 1;
        let object_name_length = usize::from(
            (ethercat_to_host(a_sdo.mailbox_header.length) - 16).clamp(0, MAX_NAME_LENGTH),
        );
        let sub_index = usize::from(sub_index);
        oe_list.value_info[sub_index] = a_sdo.data.as_bytes()[3];
        oe_list.data_type[sub_index] =
            Datatype::try_from(ethercat_to_host(a_sdo.data.as_words()[2]) as u8)?;
        oe_list.bit_length[sub_index] = ethercat_to_host(a_sdo.data.as_words()[3]);
        oe_list.object_access[sub_index] = ethercat_to_host(a_sdo.data.as_words()[4]);
        oe_list.name[sub_index].clear();
        oe_list.name[sub_index]
            .push_str(str::from_utf8(
                &a_sdo.data.as_bytes()[10..10 + object_name_length],
            )?)
            .unwrap();
        Ok(())
    } else if matches!(
        a_sdo.opcode,
        COEObjectDescriptionCommand::ServiceDataObjectInformationError
    ) {
        // SDO info error received
        let abort_error = AbortError::Abort(ethercat_to_host(a_sdo.data.as_longs()[0]) as i32);
        sdo_info_error(context, slave, index, sub_index, abort_error);
        Err(abort_error.into())
    } else {
        let error_code = PacketError::UnexpectedFrameReturned;
        packet_error(context, slave, index, sub_index, error_code);
        Err(error_code.into())
    }
}

/// CANopen read Service Data Object Service object entry
///
/// # Parameters
/// - `context`: Context struct
/// - `item`: Item in Object Description list
/// - `od_list`: Object description list for reference
/// - `oe_list`: Object entry structure
///
/// # Returns Unit or workcounter
pub fn read_oe(
    context: &mut Context,
    item: u16,
    od_list: &mut ObjectDescriptionList,
    oe_list: &mut ObjectEntryList,
) -> Result<(), CoEError> {
    oe_list.entries = 0;
    for sub_count in 0..od_list.max_sub[usize::from(item)] {
        read_oe_single(context, item, sub_count, od_list, oe_list)?;
    }
    Ok(())
}
