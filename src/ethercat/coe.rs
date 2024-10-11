use core::slice;
use std::mem::transmute;
use std::time::SystemTime;
use std::{any::Any, time::Duration};

use heapless::String as HeaplessString;

use crate::ethercat::main::{clear_mailbox, mailbox_send, next_mailbox_count, packet_error};
use crate::ethercat::r#type::{
    ethercat_to_host, host_to_ethercat, mailbox_header_set_count, COEMailboxType,
    CanopenOverEthercatSdoCommand, MailboxType, TIMEOUT_TX_MAILBOX,
};

use super::main::{
    mailbox_receive, push_error, Context, MailboxBuffer, MailboxHeader, MAX_MAILBOX_SIZE,
    MAX_NAME_LENGTH,
};
use super::r#type::Datatype;
use super::r#type::{AbortError, COEObjectDescriptionCommand, ErrorInfo, ErrorType, MailboxError};

/// Invalid Service Data Object size
pub struct InvalidSDOSize;

/// Invalid Service Data Object Service size
pub struct InvalidSDOServiceSize;

pub enum CoEError {
    InvalidSDOSize(InvalidSDOSize),
    InvalidSDOServiceSize(InvalidSDOServiceSize),
    Mailbox(MailboxError),
}

impl From<InvalidSDOSize> for CoEError {
    fn from(value: InvalidSDOSize) -> Self {
        Self::InvalidSDOSize(value)
    }
}

impl From<InvalidSDOServiceSize> for CoEError {
    fn from(value: InvalidSDOServiceSize) -> Self {
        Self::InvalidSDOServiceSize(value)
    }
}

impl From<MailboxError> for CoEError {
    fn from(value: MailboxError) -> Self {
        Self::Mailbox(value)
    }
}

/// Variants for easy data access
#[repr(C)]
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
#[repr(C)]
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

impl ServiceDataObject {
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        self.into()
    }
}

/// Service Data Object service structure
#[repr(C)]
struct ServiceDataObjectService {
    mailbox_header: MailboxHeader,
    can_open: u16,
    opcode: COEObjectDescriptionCommand,
    reserved: u8,
    fragments: u16,
    data: ServiceData,
}

impl TryFrom<&mut [u8]> for &mut ServiceDataObjectService {
    type Error = InvalidSDOServiceSize;

    fn try_from(value: &mut [u8]) -> Result<Self, Self::Error> {
        if value.len() < size_of::<Self>() {
            Err(InvalidSDOServiceSize)
        } else {
            Ok(unsafe { &mut slice::from_raw_parts_mut((value as *mut [u8]).cast(), 1)[0] })
        }
    }
}

/// Max entries in object description list
pub const MAX_OBJECT_DESCRIPTION_LIST_SIZE: usize = 1024;

/// Max entries in Object Entries list
pub const MAX_OBJECT_ENTRY_LIST_SIZE: usize = 256;

/// Storage for object description list
#[repr(C)]
pub struct ObjectDescriptionList {
    /// Slave number
    pub slave: u16,

    /// Number of entries in list
    pub entries: u16,

    /// Array of indexes
    pub index: [u16; MAX_OBJECT_DESCRIPTION_LIST_SIZE],

    /// Array of datatypes, see EtherCAT specification
    pub data_type: [Datatype; MAX_OBJECT_DESCRIPTION_LIST_SIZE],

    /// Array of object codes, see EtherCAT specification
    pub object_code: [u16; MAX_OBJECT_DESCRIPTION_LIST_SIZE],

    /// Number of subindexes for each index
    pub max_sub: [u8; MAX_OBJECT_DESCRIPTION_LIST_SIZE],

    pub name: [HeaplessString<MAX_NAME_LENGTH>; MAX_OBJECT_DESCRIPTION_LIST_SIZE],
}

/// Storage for object list entry information
#[repr(C)]
pub struct ObjectEntryList {
    /// Number of entries in list
    pub entries: u16,

    /// Array of value info, see EtherCAT specification
    pub value_info: [u8; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of value data types, see EtherCAT specification
    pub data_type: [u16; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of bit lengths, see EtherCAT specification
    pub bit_length: [u16; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Array of object access bits, see EtherCAT specification
    pub object_access: [u16; MAX_OBJECT_ENTRY_LIST_SIZE],

    /// Textual description of each index
    pub name: [HeaplessString<MAX_NAME_LENGTH>; MAX_OBJECT_ENTRY_LIST_SIZE],
}

#[cfg(feature = "ec_ver1")]
pub mod ec_ver1 {
    use std::{any::Any, time::Duration};

    use super::{ObjectDescriptionList, ObjectEntryList};

    pub fn sdo_error(slave: u16, index: u16, sub_index: u8, abort_code: i32) {
        todo!()
    }

    pub fn sdo_read(
        slave: u16,
        index: u16,
        sub_index: u8,
        ca: bool,
        pointer_length: &mut usize,
        pointer: &mut [Box<dyn Any>],
        timeout: Duration,
    ) -> i32 {
        todo!()
    }

    pub fn sdo_write(
        slave: u16,
        index: u16,
        sub_index: u8,
        ca: bool,
        pointer: &[Box<dyn Any>],
        timeout: Duration,
    ) {
        todo!()
    }

    pub fn rx_pdo(slave: u16, rx_pdo_number: u16, pointer: &[Box<dyn Any>]) -> i32 {
        todo!()
    }

    pub fn tx_pdo(
        slave: u16,
        tx_pdo_number: u16,
        length: &mut usize,
        pointer: &[Box<dyn Any>],
        timeout: Duration,
    ) -> i32 {
        todo!()
    }

    pub fn read_pdo_map(slave: u16, osize: &mut [u32], isize: &mut [u32]) -> i32 {
        todo!()
    }

    pub fn read_pdo_map_ca(slave: u16, thread_n: i32, osize: &mut [u32], isize: &mut [u32]) -> i32 {
        todo!()
    }

    pub fn read_od_list(slave: u16, od_list: &mut ObjectDescriptionList) -> i32 {
        todo!()
    }

    pub fn read_od_description(item: u16, od_list: &mut ObjectDescriptionList) -> i32 {
        todo!()
    }

    pub fn read_oe_single(
        item: u16,
        sub_index: u8,
        od_list: &mut ObjectDescriptionList,
        oe_list: &mut ObjectEntryList,
    ) -> i32 {
        todo!()
    }

    pub fn read_oe(
        item: u16,
        od_list: ObjectDescriptionList,
        oe_list: &mut ObjectEntryList,
    ) -> i32 {
        todo!()
    }
}

/// Report Service Data Object error
///
/// # Parameters
/// `context`: Context struct
/// `slave`: slave number
/// `index`: index that generated the error
/// `sub_index`: Subindex that generated the error
/// `abort_error`: Abort code or error, see EtherCAT documentation for list
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
/// `context`: Context struct
/// `slave`: Slave number
/// `index`: Index that generated the error
/// `sub_index`: Subindex that generated the error
/// `abort_error`: Abort code or error, see EtherCAT documentation for list
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
fn handle_invalid_slave_response(
    a_sdo: &ServiceDataObject,
    context: &mut Context,
    slave: u16,
    index: u16,
    sub_index: u8,
) {
    if a_sdo.command == CanopenOverEthercatSdoCommand::Abort.into() {
        // SDO abort frame received
        sdo_error(
            context,
            slave,
            index,
            sub_index,
            AbortError::Abort(ethercat_to_host(a_sdo.data.as_longs()[0] as i32)),
        );
    } else {
        // Unexpected frame returned
        packet_error(context, slave, index, sub_index, 1);
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
/// `context`: Context struct
/// `slave`: Slave number
/// `index`: Index to read
/// `subindex`: Subindex to read, must be 0 or 1 if CA is used
/// `ca`: False for single subindex, true for complete access, all subindexes read
/// `parameter_buffer`: Reference to parameter buffer
/// `timeout`: Timeout duration, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Returns
/// Unit or error
pub fn sdo_read(
    context: &mut Context,
    slave: u16,
    index: u16,
    mut sub_index: u8,
    complete_access: bool,
    size: &mut usize,
    parameter_buffer: &mut [u8],
    timeout: Duration,
) -> Result<(), CoEError> {
    let mut mailbox_in: MailboxBuffer = [0; MAX_MAILBOX_SIZE];

    // Empty slave out mailbox, if something is in it. Timeout set to 0.
    mailbox_receive(context, slave, &mut mailbox_in, Duration::default())?;
    let mut mailbox_out: MailboxBuffer = [0; MAX_MAILBOX_SIZE];
    let mut sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut_slice())?;
    sdo.mailbox_header.length = host_to_ethercat(0xa);
    sdo.mailbox_header.address = host_to_ethercat(0);
    sdo.mailbox_header.priority = 0;

    // Get new mailbox count value, used as session handle
    let count = {
        let mut slavelist = context.slavelist.lock().unwrap();
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
    mailbox_send(context, slave, &mut mailbox_out, TIMEOUT_TX_MAILBOX)?;

    // Clean mailbox buffer
    clear_mailbox(&mut mailbox_in);

    // Read slave response
    mailbox_receive(context, slave, &mut mailbox_in, timeout)?;

    // Slave response should be CANopen over EtherCAT, a Service Data Object response, and use the correct index
    let a_sdo = <&mut ServiceDataObject>::try_from(mailbox_in.as_mut_slice())?;
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
            if *size >= usize::from(bytesize) {
                parameter_buffer
                    .iter_mut()
                    .zip(a_sdo.data.as_bytes())
                    .take(usize::from(bytesize))
                    .for_each(|(dest, src)| *dest = *src);
                *size = usize::from(bytesize);
            } else {
                // Data container too small for type
                packet_error(context, slave, index, sub_index, 3);
            }
        } else {
            // Normal frame response
            let sdo_len = ethercat_to_host(a_sdo.data.as_longs()[0]) as usize;

            // Check whether the parameter fits in the parameter buffer
            if sdo_len <= *size {
                let mut hp = 0;
                let bp = 0;

                // Calculate mailblox transfer size
                let frame_data_size =
                    usize::from(ethercat_to_host(a_sdo.mailbox_header.length) - 10);

                // Check whether the transfer is segmented
                if frame_data_size < sdo_len {
                    parameter_buffer[hp..]
                        .iter_mut()
                        .zip(a_sdo.data.as_bytes())
                        .take(frame_data_size)
                        .for_each(|(dest, src)| *dest = *src);

                    // Increment the buffer pointer
                    hp += frame_data_size;
                    *size = frame_data_size;
                    let mut segments_left = true;
                    let mut toggle = 0x00;
                    while segments_left {
                        sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut_slice())?;
                        sdo.mailbox_header.length = host_to_ethercat(0xA);
                        sdo.mailbox_header.address = host_to_ethercat(0);
                        sdo.mailbox_header.priority = 0;
                        {
                            let slave = &mut context.slavelist.lock().unwrap()[usize::from(slave)];
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
                        mailbox_send(context, slave, &mut mailbox_out, timeout)?;

                        clear_mailbox(&mut mailbox_in);

                        // Read slave response
                        mailbox_receive(context, slave, &mut mailbox_in, timeout)?;

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
                                parameter_buffer[hp..]
                                    .iter_mut()
                                    .zip(a_sdo.as_bytes_mut())
                                    .skip(
                                        size_of::<MailboxHeader>()
                                            + size_of::<u16>()
                                            + size_of::<u8>(),
                                    )
                                    .take(frame_data_size.into())
                                    .for_each(|(dest, src)| *dest = *src);
                            } else {
                                // Segments follow

                                // Copy to parameter buffer
                                parameter_buffer[hp..]
                                    .iter_mut()
                                    .zip(a_sdo.as_bytes_mut())
                                    .skip(
                                        size_of::<MailboxHeader>()
                                            + size_of::<u16>()
                                            + size_of::<u8>(),
                                    )
                                    .take(frame_data_size.into())
                                    .for_each(|(dest, src)| *dest = *src);

                                // Increment buffer pointer
                                hp += usize::from(frame_data_size);
                            }

                            // Update parameter size
                            *size += usize::from(frame_data_size);
                        } else {
                            // Unexpected frame returned from slave
                            segments_left = false;
                            handle_invalid_slave_response(a_sdo, context, slave, index, sub_index);
                        }
                        toggle ^= 0x10;
                    }
                } else {
                    // Non segmented transfer
                    parameter_buffer[bp..]
                        .iter_mut()
                        .zip(&a_sdo.data.as_bytes()[4..])
                        .take(sdo_len)
                        .for_each(|(dest, src)| *dest = *src);
                    *size = sdo_len;
                }
            } else {
                // Data container too small for type
                packet_error(context, slave, index, sub_index, 3);
            }
        }
    } else {
        // Other slave response

        // SDO abort frame received
        handle_invalid_slave_response(a_sdo, context, slave, index, sub_index);
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
/// `context`: Context struct
/// `slave`: Slave number
/// `index`: Index to write
/// `sub_index`: Subindex to write, must be 0 or 1 if complete access is used.
/// `comlete_access`: False for single subindex, true for complete access, all subindexes written
/// `parameter_buffer`: Timeout delay, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Returns
/// Unit or error
pub fn sdo_write(
    context: &mut Context,
    slave: u16,
    index: u16,
    sub_index: u8,
    complete_access: bool,
    parameter_buffer: &mut [u8],
    timeout: Duration,
) -> Result<(), CoEError> {
    let mut mailbox_in: MailboxBuffer = [0; MAX_MAILBOX_SIZE];

    // Empty slave out mailbox if something is in it. With timeout set to 0.
    mailbox_receive(context, slave, &mut mailbox_in, Duration::from_nanos(0))?;

    let mut mailbox_out = [0; MAX_MAILBOX_SIZE];
    let a_sdo = <&mut ServiceDataObject>::try_from(mailbox_in.as_mut_slice())?;
    let sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut_slice())?;

    // Data section = mailbox size - 6 mailbox - 2 CANopen over EtherCAT - 8 service data object requests
    let mut maxdata = context.slavelist.lock().unwrap()[usize::from(slave)].mailbox_length - 0x10;

    // For small data use expedited transfer
    if parameter_buffer.len() <= 4 && !complete_access {
        sdo.mailbox_header.length = host_to_ethercat(0xA);
        sdo.mailbox_header.address = host_to_ethercat(0);
        sdo.mailbox_header.priority = 0;

        {
            let slave_object = &mut context.slavelist.lock().unwrap()[usize::from(slave)];

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
        sdo.data
            .as_bytes_mut()
            .iter_mut()
            .zip(parameter_buffer.iter())
            .for_each(|(dest, src)| *dest = *src);

        // Send mailbox SDO download request to slave
        mailbox_send(context, slave, &mut mailbox_out, TIMEOUT_TX_MAILBOX)?;

        clear_mailbox(&mut mailbox_in);

        // Read slave response
        mailbox_receive(context, slave, &mut mailbox_in, timeout)?;

        if !(matches!(
            MailboxType::try_from(a_sdo.mailbox_header.mailbox_type & 0xF)?,
            MailboxType::CanopenOverEthercat
        ) && matches!(
            COEMailboxType::try_from(ethercat_to_host(a_sdo.can_open >> 12) as u8)?,
            COEMailboxType::SdoResponse
        ) && a_sdo.index == sdo.index
            && a_sdo.sub_index == sdo.sub_index)
        {
            handle_invalid_slave_response(a_sdo, context, slave, index, sub_index);
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
            let slave_object = &mut context.slavelist.lock().unwrap()[usize::from(slave)];
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
        sdo.data
            .as_bytes_mut()
            .iter_mut()
            .skip(4)
            .zip(parameter_buffer.iter())
            .take(frame_data_size.into())
            .for_each(|(dest, src)| *dest = *src);

        // Send mailbox SDO download request to slave
        mailbox_send(context, slave, &mut mailbox_out, TIMEOUT_TX_MAILBOX)?;

        clear_mailbox(&mut mailbox_in);

        // Read slave response
        mailbox_receive(context, slave, &mut mailbox_in, timeout)?;

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
                let parameter_index: usize = 0;
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
                    let slave_object = &mut context.slavelist.lock().unwrap()[usize::from(slave)];
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
                parameter_buffer[parameter_index..]
                    .iter_mut()
                    .zip(mailbox_in)
                    .skip(size_of::<MailboxHeader>() + size_of::<u16>() + size_of::<u8>())
                    .take(frame_data_size.into())
                    .for_each(|(dest, src)| *dest = src);

                // Send Service Data Object download request
                mailbox_send(context, slave, &mut mailbox_out, timeout)?;

                clear_mailbox(&mut mailbox_in);

                // Read slave response
                mailbox_receive(context, slave, &mut mailbox_in, timeout)?;
                if !(matches!(
                    MailboxType::try_from(sdo.mailbox_header.mailbox_type & 0xF)?,
                    MailboxType::CanopenOverEthercat
                ) && matches!(
                    COEMailboxType::try_from(ethercat_to_host(a_sdo.can_open >> 12) as u8)?,
                    COEMailboxType::SdoResponse
                ) && a_sdo.command & 0xE0 == 0x20)
                {
                    handle_invalid_slave_response(a_sdo, context, slave, index, sub_index);
                    segments_left = false;
                }
                toggle ^= 0x10;
            }
        } else {
            handle_invalid_slave_response(a_sdo, context, slave, index, sub_index);
        }
    }
    Ok(())
}

/// CANopen over EtherCAT RxPDO write, blocking
///
/// A RxPDO download request is issued.
///
/// # Parameters
/// `context`: Context struct
/// `slave`: Slave number
/// `RxPDOnumber`: Related RxPDO buffer
/// `pdo_buffer`: Reference to PDO buffer
///
/// # Returns
/// Unit or an error
pub fn rx_pdo(
    context: &mut Context,
    slave: u16,
    rx_pdo_number: u16,
    pdo_buffer: &[u8],
) -> Result<(), CoEError> {
    let mut mailbox_in: MailboxBuffer = [0; MAX_MAILBOX_SIZE];

    // Empty slave out mailbox, if something is in it. With timeout set to 0.
    mailbox_receive(context, slave, &mut mailbox_in, Duration::from_nanos(0))?;

    let mut mailbox_out: MailboxBuffer = [0; MAX_MAILBOX_SIZE];
    let sdo = <&mut ServiceDataObject>::try_from(mailbox_out.as_mut_slice())?;

    // Data section = mailbox size - 6 mailbox - 2 CANopen over EtherCAT
    let max_data = context.slavelist.lock().unwrap()[usize::from(slave)].mailbox_length - 8;
    let framedatasize = (pdo_buffer.len() as u16).min(max_data);
    sdo.mailbox_header.length = host_to_ethercat(2 + framedatasize);
    sdo.mailbox_header.address = host_to_ethercat(0);
    sdo.mailbox_header.priority = host_to_ethercat(0);

    // Get new mailbox counter, used for session handle
    {
        let slave_object = &mut context.slavelist.lock().unwrap()[usize::from(slave)];
        let count = next_mailbox_count(slave_object.mailbox_count);
        slave_object.mailbox_count = count;
        sdo.mailbox_header.mailbox_type =
            u8::from(MailboxType::CanopenOverEthercat) + mailbox_header_set_count(count);
    }
    sdo.can_open =
        host_to_ethercat(rx_pdo_number & 0x1FF) + (u16::from(COEMailboxType::RxPdo) << 12);

    // Copy PDO data to mailbox
    mailbox_out
        .iter_mut()
        .skip(size_of::<MailboxHeader>() + size_of::<u16>())
        .zip(<&mut [u8]>::from(sdo))
        .for_each(|(dest, src)| *dest = *src);

    mailbox_send(context, slave, &mut mailbox_out, TIMEOUT_TX_MAILBOX)?;
    Ok(())
}

pub fn tx_pdo(
    context: &mut Context,
    slave: u16,
    tx_pdo_number: u16,
    size: &mut usize,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn read_pdo_map(context: &mut Context, slave: u16, osize: &mut u32, isize: &mut u32) -> i32 {
    todo!()
}

pub fn read_pdo_map_ca(
    context: &mut Context,
    slave: u16,
    thread_n: i32,
    osize: &mut u32,
    isize: &mut u32,
) -> i32 {
    todo!()
}

pub fn read_od_list(context: &mut Context, slave: u16, od_list: &mut ObjectDescriptionList) -> i32 {
    todo!()
}

pub fn read_od_description(
    context: &mut Context,
    item: u16,
    od_list: &mut ObjectDescriptionList,
) -> i32 {
    todo!()
}

pub fn read_oe_single(
    context: &mut Context,
    item: u16,
    sub_index: u8,
    od_list: &mut ObjectDescriptionList,
    oe_list: &mut ObjectEntryList,
) -> i32 {
    todo!()
}

pub fn read_oe(
    context: &mut Context,
    item: u16,
    od_list: &mut ObjectDescriptionList,
    oe_list: &mut ObjectEntryList,
) -> i32 {
    todo!()
}
