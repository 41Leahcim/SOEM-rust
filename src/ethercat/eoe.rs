//! Ethernet over EtherCAT (EoE) module.
//!
//! Set/Get IP functions.
//! Blocking send/receive Ethernet frame
//! Read incoming EoE fragment to ethernet frame

use std::{
    io::{self, Write},
    net::Ipv4Addr,
    str::Utf8Error,
    time::Duration,
};

use num_traits::PrimInt;

use crate::ethercat::{
    main::{next_mailbox_count, MailboxOut},
    r#type::{ethercat_to_host, host_to_ethercat, mailbox_header_set_count, MailboxType},
};

use super::{
    main::{Context, MailboxHeader, MailboxIn, MAX_MAILBOX_SIZE},
    r#type::{Ethercat, MailboxError},
};

/// Use maximum size for EOE mailbox data - mailboxheader and 2 * frameinfo
pub const MAX_EOE_DATA_LENGTH: usize =
    MAX_MAILBOX_SIZE - size_of::<MailboxHeader>() - size_of::<u16>() * 2;

/// DNS length according to ETG 1000.6
pub const DNS_NAME_LENGTH: usize = 32;

/// Ethernet address length not including VLAN
pub const ETHERNET_ADDRESS_LENGTH: usize = 6;

/// IPv4 address length
pub const IP4_LENGTH: usize = size_of::<u32>();

pub const fn eoe_make_u32(a: u8, b: u8, c: u8, d: u8) -> u32 {
    ((a as u32) << 24) | ((b as u32) << 16) | ((c as u32) << 8) | d as u32
}

pub fn host_to_network<Int: PrimInt>(value: Int) -> Int {
    value.to_be()
}

pub fn network_to_host<Int: PrimInt>(value: Int) -> Int {
    if cfg!(target_endian = "little") {
        value.to_le()
    } else {
        value
    }
}

/// Header frame info 1
pub const EOE_HEADER_FRAME_TYPE_OFFSET: u8 = 0;
pub const EOE_HEADER_FRAME_TYPE: u8 = 0xF;

pub fn eoe_header_frame_type_set(x: EoEFrameType) -> u8 {
    u8::from(x) & 0xF
}

pub fn eoe_header_frame_type_get(x: u8) -> Result<EoEFrameType, EoEError> {
    EoEFrameType::try_from(x & 0xF)
}

pub const EOE_HEADER_FRAME_PORT_OFFSET: u8 = 4;
pub const EOE_HEADER_FRAME_PORT: u8 = 0xF0;

pub const fn eoe_header_frame_port_set(x: u8) -> u8 {
    (x & 0xF) << 4
}

pub const fn eoe_header_frame_port_get(x: u8) -> u8 {
    (x >> 4) & 0xF
}

pub const EOE_HEADER_LAST_FRAGMENT_OFFSET: u8 = 8;
pub const EOE_HEADER_LAST_FRAGMENT: u16 = 0x100;

pub const fn eoe_header_last_fragment_set(value: u16) -> u16 {
    (value & 1) << 8
}

pub const fn eoe_header_last_fragment_get(value: u16) -> u16 {
    (value >> 8) & 1
}

pub const EOE_HEADER_TIME_APPEND_OFFSET: u8 = 9;
pub const EOE_HEADER_TIME_APPEND: u16 = 1 << 9;

pub const fn eoe_header_time_append_set(x: u16) -> u16 {
    (x & 1) << 9
}

pub const fn eoe_header_time_append_get(x: u16) -> u16 {
    (x >> 9) & 1
}

pub const EOE_HEADER_TIME_REQUEST_OFFSET: u8 = 10;
pub const EOE_HEADER_TIME_REQUEST: u16 = 1 << 10;

pub const fn eoe_header_time_request_set(value: u16) -> u16 {
    (value & 1) << 10
}

pub const fn eoe_header_time_request_get(value: u16) -> u16 {
    (value >> 10) & 1
}

/// Header frame info 2
pub const EOE_HEADER_FRAGMENT_NUMBER_OFFSET: u8 = 0;
pub const EOE_HEADER_FRAGMENT_NUMBER: u8 = 0x3F;

pub const fn eoe_header_fragment_number_set(value: u8) -> u8 {
    value & 0x3F
}

pub const fn eoe_header_fragment_number_get(value: u8) -> u8 {
    value & 0x3F
}

pub const EOE_HEADER_FRAME_OFFSET_OFFSET: u8 = 6;
pub const EOE_HEADER_FRAME_OFFSET: u16 = 0x3F << 6;

pub const fn eoe_header_frame_offset_set(value: u16) -> u16 {
    (value & 0x3F) << 6
}

pub const fn eoe_header_frame_offset_get(value: u16) -> u16 {
    (value >> 6) & 0x3F
}

pub const EOE_HEADER_FRAME_NUMBER_OFFSET: u8 = 12;
pub const EOE_HEADER_FRAME_NUMBER: u16 = 0xF << 12;

pub const fn eoe_header_frame_number_set(value: u16) -> u16 {
    (value & 0xF) << 12
}

pub const fn eoe_header_frame_number_get(value: u16) -> u16 {
    (value >> 12) & 0xF
}

/// EOE param
pub const EOE_PARAMETER_OFFSET: u8 = 4;
pub const EOE_PARAMETER_MAC_INCLUDE: u8 = 1;
pub const EOE_PARAM_IP_INCLUDE: u8 = 2;
pub const EOE_PARAM_SUBNET_IP_INCLUDE: u8 = 4;
pub const EOE_PARAM_DEFAULT_GATEWAY_INCLUDE: u8 = 8;
pub const EOE_PARAM_DNS_IP_INCLUDE: u8 = 0x10;
pub const EOE_PARAM_DNS_NAME_INCLUDE: u8 = 0x20;

/// EoE frame types
#[derive(Debug, PartialEq, Eq)]
pub enum EoEFrameType {
    FragmentData,
    InitResponseTimestamp,

    /// Spec SET IP REQ
    InitRequest,

    /// Spec SET IP RESP
    InitResponse,

    SetAddressFilterRequest,
    SetAddressFilterResponse,
    GetIpParameterRequest,
    GetIpParameterResponse,
    GetAddressFilterRequest,
    GetAddressFilterResponse,
}

impl From<EoEFrameType> for u8 {
    fn from(value: EoEFrameType) -> Self {
        match value {
            EoEFrameType::FragmentData => 0,
            EoEFrameType::InitResponseTimestamp => 1,
            EoEFrameType::InitRequest => 2,
            EoEFrameType::InitResponse => 3,
            EoEFrameType::SetAddressFilterRequest => 4,
            EoEFrameType::SetAddressFilterResponse => 5,
            EoEFrameType::GetIpParameterRequest => 6,
            EoEFrameType::GetIpParameterResponse => 7,
            EoEFrameType::GetAddressFilterRequest => 8,
            EoEFrameType::GetAddressFilterResponse => 9,
        }
    }
}

impl TryFrom<u8> for EoEFrameType {
    type Error = EoEError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::FragmentData),
            1 => Ok(Self::InitResponseTimestamp),
            2 => Ok(Self::InitRequest),
            3 => Ok(Self::InitResponse),
            4 => Ok(Self::SetAddressFilterRequest),
            5 => Ok(Self::SetAddressFilterResponse),
            6 => Ok(Self::GetIpParameterRequest),
            7 => Ok(Self::GetIpParameterResponse),
            8 => Ok(Self::GetAddressFilterRequest),
            9 => Ok(Self::GetAddressFilterResponse),
            _ => Err(EoEError::InvalidFrameType(value)),
        }
    }
}

pub enum EoEResult {
    Success = 0,
    UnspecifiedError = 1,
    UnsupportedFrameType = 2,
    NoIpSupport = 0x201,
    NoDhcpSupport = 0x202,
    NoFilterSupport = 0x401,
}

impl From<EoEResult> for u16 {
    fn from(value: EoEResult) -> Self {
        value as u16
    }
}

pub struct EthernetAddress {
    pub address: [u8; ETHERNET_ADDRESS_LENGTH],
}

/// EoE IP request structure, storage only, no need to pack
pub struct EoeParameter {
    pub mac: Option<EthernetAddress>,
    pub ip: Option<Ipv4Addr>,
    pub subnet: Option<Ipv4Addr>,
    pub default_gateway: Option<Ipv4Addr>,
    pub dns_ip: Option<Ipv4Addr>,
    pub dns_name: Option<heapless::String<DNS_NAME_LENGTH>>,
}

pub enum EthernetOverEthercatInfoResult {
    FrameInfo2(Ethercat<u16>),
    Result(Ethercat<u16>),
}

impl EthernetOverEthercatInfoResult {
    fn inner(&self) -> Ethercat<u16> {
        match self {
            EthernetOverEthercatInfoResult::FrameInfo2(word)
            | EthernetOverEthercatInfoResult::Result(word) => *word,
        }
    }
}

pub enum EoEError {
    EoEFromBytes(EthernetOverEthercatFromBytesError),
    EoEWrite(EthernetOverEthercatWriteError),
    Mailbox(MailboxError),
    Utf8(Utf8Error),
    InvalidFrameType(u8),
    InvalidResponse,
    PacketError,
    UnsupportedFrameType,
    MailboxError,
    InvalidRxData,
}

impl From<MailboxError> for EoEError {
    fn from(value: MailboxError) -> Self {
        Self::Mailbox(value)
    }
}

impl From<EthernetOverEthercatFromBytesError> for EoEError {
    fn from(value: EthernetOverEthercatFromBytesError) -> Self {
        Self::EoEFromBytes(value)
    }
}

impl From<EthernetOverEthercatWriteError> for EoEError {
    fn from(value: EthernetOverEthercatWriteError) -> Self {
        Self::EoEWrite(value)
    }
}

impl From<Utf8Error> for EoEError {
    fn from(value: Utf8Error) -> Self {
        Self::Utf8(value)
    }
}

pub enum EthernetOverEthercatFromBytesError {
    InvalidSize(usize),
    Io(io::Error),
}

impl From<io::Error> for EthernetOverEthercatFromBytesError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

pub enum EthernetOverEthercatWriteError {
    Io(io::Error),
    FailedToWriteCompletely(usize),
}

impl From<io::Error> for EthernetOverEthercatWriteError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

pub struct EthernetOverEthercat {
    pub mailbox_header: MailboxHeader,
    pub frame_info1: Ethercat<u16>,
    pub info_result: EthernetOverEthercatInfoResult,
    pub data: [u8; MAX_EOE_DATA_LENGTH],
}

impl Default for EthernetOverEthercat {
    fn default() -> Self {
        Self {
            mailbox_header: MailboxHeader::default(),
            frame_info1: Ethercat::from_raw(0),
            info_result: EthernetOverEthercatInfoResult::FrameInfo2(Ethercat::default()),
            data: [0; MAX_EOE_DATA_LENGTH],
        }
    }
}

impl TryFrom<&[u8]> for EthernetOverEthercat {
    type Error = EthernetOverEthercatFromBytesError;
    fn try_from(mut value: &[u8]) -> Result<Self, Self::Error> {
        if value.len()
            < size_of::<MailboxHeader>()
                + size_of::<u16>() * 2
                + size_of::<u8>() * MAX_EOE_DATA_LENGTH
        {
            return Err(EthernetOverEthercatFromBytesError::InvalidSize(value.len()));
        }
        let mailbox_header = MailboxHeader::read_from(&mut value)?;
        Ok(Self {
            mailbox_header,
            frame_info1: Ethercat::from_raw(u16::from_ne_bytes(value[..2].try_into().unwrap())),
            info_result: EthernetOverEthercatInfoResult::FrameInfo2(Ethercat::from_raw(
                u16::from_ne_bytes(value[2..4].try_into().unwrap()),
            )),
            data: value[4..4 + MAX_EOE_DATA_LENGTH].try_into().unwrap(),
        })
    }
}

impl EthernetOverEthercat {
    fn write_to<W: Write>(&self, writer: &mut W) -> Result<usize, EthernetOverEthercatWriteError> {
        let mut written = self
            .mailbox_header
            .write_to(writer)
            .map_err(EthernetOverEthercatWriteError::FailedToWriteCompletely)?;
        match writer.write(&self.frame_info1.into_inner().to_ne_bytes()) {
            Err(err) => return Err(err.into()),
            Ok(bytes) if bytes < 2 => {
                return Err(EthernetOverEthercatWriteError::FailedToWriteCompletely(
                    written + bytes,
                ))
            }
            Ok(bytes) => written += bytes,
        }
        match writer.write(&self.info_result.inner().into_inner().to_ne_bytes()) {
            Err(err) => return Err(err.into()),
            Ok(bytes) if bytes < 2 => {
                return Err(EthernetOverEthercatWriteError::FailedToWriteCompletely(
                    written + bytes,
                ))
            }
            Ok(bytes) => written += bytes,
        }
        match writer.write(&self.data) {
            Err(err) => Err(err.into()),
            Ok(bytes) if bytes < self.data.len() => Err(
                EthernetOverEthercatWriteError::FailedToWriteCompletely(written + bytes),
            ),
            Ok(bytes) => Ok(written + bytes),
        }
    }
}

/// EoE utility function to convert `IPv4Addr` to EoE ip bytes
///
/// # Parameters
/// - `ip`: IPv4 address
/// - `bytes`: EoE ip octets output buffer
fn ip_to_bytes(ip_address: Ipv4Addr, bytes: &mut [u8]) {
    bytes[3] = ip_address.octets()[0];
    bytes[2] = ip_address.octets()[1];
    bytes[1] = ip_address.octets()[2];
    bytes[0] = ip_address.octets()[3];
}

/// EoE utility function to convert EoE ip bytes to `IPv4Addr`
///
/// # Parameters
/// `bytes`: EOE octets input buffer
///
/// # Returns
/// `Ipv4Addr`
const fn bytes_to_ip(bytes: &[u8]) -> Ipv4Addr {
    Ipv4Addr::new(bytes[3], bytes[2], bytes[1], bytes[0])
}

pub fn eoe_define_hook(context: &mut Context, hook: fn(&mut Context, u16, &mut [u8]) -> i32) {
    context.ethernet_over_ethercat_hook = hook;
}

/// EoE set IP, blocking. Waits for response from the slave.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `port`: Port number on slave if applicable
/// - `ip_parameter`: IP parameter data to be send
/// - `timeout`: Timeout duration, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Returns
/// `Ok(())` or Error
pub fn eoe_set_ip(
    context: &mut Context,
    slave: u16,
    port: u8,
    ip_parameter: &mut EoeParameter,
    timeout: Duration,
) -> Result<(), EoEError> {
    let mut mailbox_in = MailboxIn::default();

    // Empty slave out mailbox if something is in it, with timeout set to 0.
    mailbox_in.receive(context, slave, Duration::default())?;
    let mut eoe = EthernetOverEthercat::default();
    eoe.mailbox_header.address = host_to_ethercat(0);
    eoe.mailbox_header.priority = 0;
    let mut data_offset = usize::from(EOE_PARAMETER_OFFSET);

    // Get new mailbox count value, used as session handle
    let count = next_mailbox_count(context.slavelist[usize::from(slave)].mailbox_count);
    context.slavelist[usize::from(slave)].mailbox_count = count;

    eoe.mailbox_header.mailbox_type =
        u8::from(MailboxType::EthernetOverEthercat) | mailbox_header_set_count(count);

    eoe.frame_info1 = host_to_ethercat(
        u16::from(
            eoe_header_frame_type_set(EoEFrameType::InitRequest) | eoe_header_frame_port_set(port),
        ) | EOE_HEADER_LAST_FRAGMENT,
    );

    eoe.info_result = EthernetOverEthercatInfoResult::FrameInfo2(Ethercat::default());

    let mut flags = 0;
    if let Some(mac) = &ip_parameter.mac {
        flags |= EOE_PARAMETER_MAC_INCLUDE;
        eoe.data[data_offset..].copy_from_slice(&mac.address);
        data_offset += ETHERNET_ADDRESS_LENGTH;
    }
    if let Some(ip) = ip_parameter.ip {
        flags |= EOE_PARAM_IP_INCLUDE;
        ip_to_bytes(ip, &mut eoe.data[data_offset..]);
        data_offset += IP4_LENGTH;
    }
    if let Some(subnet) = ip_parameter.subnet {
        flags |= EOE_PARAM_SUBNET_IP_INCLUDE;
        ip_to_bytes(subnet, &mut eoe.data[data_offset..]);
        data_offset += IP4_LENGTH;
    }
    if let Some(default_gateway) = ip_parameter.default_gateway {
        flags |= EOE_PARAM_DEFAULT_GATEWAY_INCLUDE;
        ip_to_bytes(default_gateway, &mut eoe.data[data_offset..]);
        data_offset += IP4_LENGTH;
    }
    if let Some(dns_ip) = ip_parameter.dns_ip {
        flags |= EOE_PARAM_DNS_IP_INCLUDE;
        ip_to_bytes(dns_ip, &mut eoe.data[data_offset..]);
        data_offset += IP4_LENGTH;
    }
    if let Some(dns_name) = &ip_parameter.dns_name {
        flags |= EOE_PARAM_DNS_NAME_INCLUDE;
        eoe.data[data_offset..].copy_from_slice(dns_name.as_bytes());
        data_offset += IP4_LENGTH;
    }

    eoe.mailbox_header.length =
        host_to_ethercat(u16::from(EOE_PARAMETER_OFFSET) + data_offset as u16);
    eoe.data[0] = flags;

    // Send Ethernet over Ethercat request to slave
    let mut mailbox_out = MailboxOut::default();
    eoe.write_to(&mut mailbox_out)?;
    mailbox_out.send(context, slave, timeout)?;

    // Clean mailboxbuffer
    mailbox_in.clear();

    // Read slave response
    mailbox_in.receive(context, slave, timeout)?;
    let a_eoe = EthernetOverEthercat::try_from(mailbox_in.as_ref())?;

    if a_eoe.mailbox_header.mailbox_type & 0xF == MailboxType::EthernetOverEthercat.into() {
        let frame_info1 = ethercat_to_host(a_eoe.frame_info1);
        let result = ethercat_to_host(a_eoe.info_result.inner());

        // Slave response should be EoE
        if eoe_header_frame_type_get(frame_info1 as u8)? != EoEFrameType::InitResponse
            || result != u16::from(EoEResult::Success)
        {
            return Err(EoEError::InvalidResponse);
        }
    } else {
        return Err(EoEError::PacketError);
    }
    Ok(())
}

/// EoE get IP, blocking. Waits for response from the slave.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `port`: Port number on slave if applicable
/// - `ip_parameter`: Ip parameter data retrieved from slave
/// - `timeout`: Timeout duration,
///
/// # Returns
/// `Ok(())` or Error
pub fn eoe_get_ip(
    context: &mut Context,
    slave: u16,
    port: u8,
    ip_parameter: &mut EoeParameter,
    timeout: Duration,
) -> Result<(), EoEError> {
    // Empty slave out mailob if something is in, with timeout set to 0.
    let mut mailbox_in = MailboxIn::default();
    mailbox_in.receive(context, slave, timeout)?;

    let mut eoe = EthernetOverEthercat::default();
    eoe.mailbox_header.address = host_to_ethercat(0);
    eoe.mailbox_header.priority = 0;
    let mut data_offset = usize::from(EOE_PARAMETER_OFFSET);
    let count = next_mailbox_count(context.slavelist[usize::from(slave)].mailbox_count);
    context.slavelist[usize::from(slave)].mailbox_count = count;

    eoe.mailbox_header.mailbox_type =
        u8::from(MailboxType::EthernetOverEthercat) + mailbox_header_set_count(count);

    eoe.frame_info1 = host_to_ethercat(
        u16::from(
            eoe_header_frame_type_set(EoEFrameType::GetIpParameterRequest)
                | eoe_header_frame_port_set(port),
        ) | EOE_HEADER_LAST_FRAGMENT,
    );
    eoe.info_result = EthernetOverEthercatInfoResult::FrameInfo2(Ethercat::default());
    eoe.mailbox_header.length = host_to_ethercat(4);
    let mut flags = 0;
    eoe.data[0] = flags;

    // Send EoE request to slave
    let mut mailbox_out = MailboxOut::default();
    eoe.write_to(&mut mailbox_out)?;
    mailbox_out.send(context, slave, timeout)?;

    mailbox_in.clear();
    mailbox_in.receive(context, slave, timeout)?;

    // Slave response should be Ethernet over EtherCAT
    let a_eoe = EthernetOverEthercat::default();
    if a_eoe.mailbox_header.mailbox_type & 0xF == MailboxType::EthernetOverEthercat.into() {
        let frame_info1 = ethercat_to_host(eoe.frame_info1);
        let eoe_data_size = ethercat_to_host(a_eoe.mailbox_header.length) - 4;
        if eoe_header_frame_type_get(frame_info1 as u8)? != EoEFrameType::GetIpParameterResponse {
            return Err(EoEError::UnsupportedFrameType);
        } else {
            flags = a_eoe.data[0];
            if flags & EOE_PARAMETER_MAC_INCLUDE != 0 {
                let mut address = [0; ETHERNET_ADDRESS_LENGTH];
                address.copy_from_slice(&eoe.data[data_offset..]);
                ip_parameter.mac = Some(EthernetAddress { address });
                data_offset += ETHERNET_ADDRESS_LENGTH;
            }
            if flags & EOE_PARAM_IP_INCLUDE != 0 {
                ip_parameter.ip = Some(bytes_to_ip(&eoe.data[data_offset..]));
                data_offset += IP4_LENGTH;
            }
            if flags & EOE_PARAM_SUBNET_IP_INCLUDE != 0 {
                ip_parameter.subnet = Some(bytes_to_ip(&eoe.data[data_offset..]));
                data_offset += IP4_LENGTH;
            }
            if flags & EOE_PARAM_DEFAULT_GATEWAY_INCLUDE != 0 {
                ip_parameter.default_gateway = Some(bytes_to_ip(&eoe.data[data_offset..]));
                data_offset += IP4_LENGTH;
            }
            if flags & EOE_PARAM_DNS_IP_INCLUDE != 0 {
                ip_parameter.dns_ip = Some(bytes_to_ip(&eoe.data[data_offset..]));
                data_offset += IP4_LENGTH;
            }
            if flags & EOE_PARAM_DNS_NAME_INCLUDE != 0 {
                let dns_length = (usize::from(eoe_data_size) - data_offset).min(DNS_NAME_LENGTH);
                ip_parameter.dns_name = Some(heapless::String::from_utf8(
                    heapless::Vec::from_slice(&eoe.data[data_offset..data_offset + dns_length])
                        .unwrap(),
                )?);
                data_offset += dns_length;
            }
            // Something is not correct, flag the error
            if data_offset > usize::from(eoe_data_size) {
                return Err(EoEError::MailboxError);
            }
        }
    } else {
        return Err(EoEError::PacketError);
    }

    Ok(())
}

/// EoE Ethernet buffer write, blocking.
///
/// If the buffer is larger than the mailbox size, the buffer is sent in
/// several fragments. The function will split the buffer data in fragments and
/// send them to the slave one by one.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `port`: Port number on slave if applicable
/// - `buffer`: Parameter buffer
/// - `timeout`: Timeout duration, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Returns
/// `Ok(())` or Error
pub fn eoe_send(
    context: &mut Context,
    slave: u16,
    port: u8,
    buffer: &mut [u8],
    timeout: Duration,
) -> Result<(), EoEError> {
    let mut eoe = EthernetOverEthercat::default();
    eoe.mailbox_header.address = host_to_ethercat(0);
    eoe.mailbox_header.priority = 0;

    // Data section=mailbox size - 6 mailbox - 4 EoEh
    let max_data = context.slavelist[usize::from(slave)].mailbox_length - 0xA;
    let mut tx_fragment_number: u8 = 0;
    let mut tx_frame_offset: u16 = 0;

    loop {
        let frames_left = true;
        let mut tx_frame_size = buffer.len() as u16 - tx_frame_offset;
        if tx_frame_size > max_data {
            // Adjust to even 32-octet blocks
            tx_frame_size = (max_data >> 5) << 5;
        }
        let frame_info1 = if u16::from(tx_fragment_number) == buffer.len() as u16 - tx_frame_offset
        {
            eoe_header_last_fragment_set(1) | u16::from(eoe_header_frame_port_set(port))
        } else {
            eoe_header_frame_port_set(port).into()
        };
        let frame_info2 = u16::from(eoe_header_fragment_number_set(tx_fragment_number))
            | if tx_fragment_number > 0 {
                eoe_header_frame_offset_set(tx_frame_offset >> 5)
            } else {
                eoe_header_frame_offset_set(((buffer.len() + 31) as u16) >> 5)
            }
            | eoe_header_frame_number_set(u16::from(tx_fragment_number));

        // Get new mailbox count value, used as session handle
        let count = next_mailbox_count(context.slavelist[usize::from(slave)].mailbox_count);
        context.slavelist[usize::from(slave)].mailbox_count = count;

        eoe.mailbox_header.length = host_to_ethercat(4 + tx_frame_size);
        eoe.mailbox_header.mailbox_type =
            u8::from(MailboxType::EthernetOverEthercat) + mailbox_header_set_count(count);

        eoe.frame_info1 = host_to_ethercat(frame_info1);
        eoe.info_result = EthernetOverEthercatInfoResult::FrameInfo2(host_to_ethercat(frame_info2));
        eoe.data.copy_from_slice(
            &buffer[usize::from(tx_frame_offset)..usize::from(tx_frame_offset + tx_frame_size)],
        );

        // Send EoE request to slave
        let mut mailbox_out = MailboxOut::default();
        eoe.write_to(&mut mailbox_out)?;
        mailbox_out.send(context, slave, timeout)?;
        if frames_left {
            tx_frame_offset += tx_frame_size;
            tx_fragment_number += 1;
        } else {
            break Ok(());
        }
    }
}

/// Ethernet buffer read, blocking.
///
/// If the buffer is larger than the mailbox size, then the buffer is received
/// by several fragments. The function will assable the fragments into
/// a complete Ethernet buffer.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `port`: Port number on slave if applicable
/// - `size`: Size in bytes of parameter buffer
/// - `buffer`: Parameter buffer
/// - `timeout`: Timeout duration, standard is `TIMEOUT_RX_MAILBOX`
///
/// # Returns
/// `Ok(())` or error code
pub fn eoe_receive(
    context: &mut Context,
    slave: u16,
    port: u8,
    size: &mut usize,
    buffer: &mut [u8],
    timeout: Duration,
) -> Result<(), EoEError> {
    let mut mailbox_in = MailboxIn::default();
    let mut frames_left = true;
    let mut rx_fragment_number = 0;
    let mut rx_frame_number = 0xFF;
    let mut rx_frame_offset = 0;

    // Hang for a while if nothing is in
    mailbox_in.receive(context, slave, timeout)?;

    while frames_left {
        // Slave response should be Ethernet over EtherCAT
        let a_eoe = EthernetOverEthercat::try_from(mailbox_in.as_ref())?;
        if a_eoe.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::EthernetOverEthercat) {
            let eoe_data_size = ethercat_to_host(a_eoe.mailbox_header.length) - 4;
            let frame_info1 = ethercat_to_host(a_eoe.frame_info1);
            let frame_info2 = ethercat_to_host(a_eoe.info_result.inner());

            let header_fragment_number = eoe_header_fragment_number_get(frame_info2 as u8);
            if header_fragment_number != rx_fragment_number && header_fragment_number != 0 {
                return Err(EoEError::InvalidRxData);
            }

            if rx_fragment_number == 0 {
                rx_frame_number = eoe_header_frame_number_get(frame_info2);
                let rx_frame_size = eoe_header_frame_offset_get(frame_info2) << 5;
                if usize::from(rx_frame_size) > buffer.len()
                    || port != eoe_header_frame_port_get(frame_info1 as u8)
                {
                    return Err(EoEError::InvalidRxData);
                }
            } else if rx_frame_number != eoe_header_frame_number_get(frame_info2)
                || rx_frame_offset != eoe_header_frame_offset_get(frame_info2) << 5
            {
                return Err(EoEError::InvalidRxData);
            }

            if usize::from(rx_frame_offset + eoe_data_size) <= buffer.len() {
                buffer.copy_from_slice(&a_eoe.data[..usize::from(eoe_data_size)]);
                rx_frame_offset += eoe_data_size;
                rx_fragment_number += 1;
            }

            if eoe_header_last_fragment_get(frame_info1) != 0 {
                // Remove timestamp
                if eoe_header_time_append_get(frame_info1) != 0 {
                    rx_frame_offset -= 4;
                }
                frames_left = false;
                *size = rx_frame_offset.into();
            } else {
                // Hang for a while if nothing is in
                mailbox_in.receive(context, slave, timeout)?;
            }
        } else {
            return Err(EoEError::PacketError);
        }
    }

    Ok(())
}

/// EoE mailbox fragment read
///
/// Will take the data in the incoming mailbox buffer and copy it to the
/// destination Ethernet frame buffer at given offset and update the current
/// fragment variables.
///
/// # Parameters
/// `mailbox_in`: Received mailbox containing fragment data
/// `rx_fragment_number`: Fragment number
/// `rx_frame_size`: Frame size
/// `rx_frame_offset`: Frame offset
/// `rx_frame_number`: Frame number
/// `buffer_size`: Frame buffer size in bytes
/// `buffer`: Frame buffer
///
/// # Returns
/// `Ok(0)` if fragment ok, `Ok(>0)` if last fragment, Error on error
pub fn eoe_read_fragment(
    mailbox_in: &mut MailboxIn,
    rx_fragment_number: &mut u8,
    rx_frame_offset: &mut u16,
    rx_frame_size: &mut u16,
    rx_frame_number: &mut u16,
    buffer_size: &mut usize,
    buffer: &mut [u8],
) -> Result<u32, EoEError> {
    let a_eoe = EthernetOverEthercat::try_from(mailbox_in.as_ref())?;

    // Slave response should be Ethernet over Ethercat
    if a_eoe.mailbox_header.mailbox_type & 0xF == u8::from(MailboxType::EthernetOverEthercat) {
        let eoe_data_size = ethercat_to_host(a_eoe.mailbox_header.length) - 4;
        let frame_info1 = ethercat_to_host(a_eoe.frame_info1);
        let frame_info2 = ethercat_to_host(a_eoe.info_result.inner());

        // Retrieve fragment number and check whether it has the expected value
        if *rx_fragment_number != eoe_header_fragment_number_get(frame_info2 as u8) {
            // If expected fragment number isn't 0, reset working variables
            if *rx_fragment_number != 0 {
                *rx_fragment_number = 0;
                *rx_frame_size = 0;
                *rx_frame_offset = 0;
                *rx_fragment_number = 0;
            }

            // If incoming fragment number is not 0, we can't recover.
            if eoe_header_fragment_number_get(frame_info2 as u8) > 0 {
                return Err(EoEError::InvalidRxData);
            }
        }

        // Check whether it's a new frame
        // If it's a new frame, make sure it's the same
        if *rx_fragment_number == 0 {
            *rx_frame_size = eoe_header_frame_offset_get(frame_info2) << 5;
            *rx_frame_offset = 0;
            *rx_frame_number = eoe_header_frame_number_get(frame_info2);
        } else if *rx_fragment_number != eoe_header_fragment_number_get(frame_info2 as u8)
            || *rx_frame_offset != eoe_header_frame_offset_get(frame_info2) << 5
        {
            *rx_fragment_number = 0;
            *rx_frame_size = 0;
            *rx_frame_offset = 0;
            *rx_frame_number = 0;
            return Err(EoEError::InvalidRxData);
        }

        // Make sure the frame size is as expected
        if *rx_frame_offset + eoe_data_size <= *rx_frame_size
            && *rx_frame_offset + eoe_data_size <= *buffer_size as u16
        {
            buffer.copy_from_slice(&a_eoe.data[..usize::from(eoe_data_size)]);
            *rx_frame_offset += eoe_data_size;
            *rx_fragment_number += 1;
        }

        // Is it the last fragment
        if eoe_header_last_fragment_get(frame_info1) != 0 {
            // Remove timestamp
            if eoe_header_time_append_get(frame_info1) != 0 {
                *rx_frame_offset -= 4;
            }

            *buffer_size = usize::from(*rx_frame_offset);
            *rx_fragment_number = 0;
            *rx_frame_size = 0;
            *rx_frame_offset = 0;
            *rx_frame_number = 0;
            return Ok(1);
        }
    } else {
        return Err(EoEError::PacketError);
    }
    Ok(0)
}
