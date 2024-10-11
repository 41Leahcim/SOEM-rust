use std::{any::Any, net::Ipv4Addr, task::Context, time::Duration};

use num_traits::PrimInt;

use super::main::{MailboxBuffer, MailboxHeader, MAX_MAILBOX_SIZE};

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

pub const fn eoe_header_frame_type_set(x: u8) -> u8 {
    x & 0xF
}

pub const fn eoe_header_frame_type_get(x: u8) -> u8 {
    eoe_header_frame_type_set(x)
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
    GetAddressFIlterResponse,
}

pub enum EoEError {
    UnspecifiedError = 1,
    UnsupportedFrameType = 2,
    NoIpSupport = 0x201,
    NoDhcpSupport = 0x202,
    NoFilterSupport = 0x401,
}

pub struct EthernetAddress {
    pub address: [u8; ETHERNET_ADDRESS_LENGTH],
}

/// EoE IP request structure, storage only, no need to pack
pub struct EoeParameter {
    pub mac_set: u8,
    pub ip_set: u8,
    pub subnet_set: u8,
    pub default_gateway_set: u8,
    pub dns_ip_set: u8,
    pub dns_name_set: u8,
    pub mac: EthernetAddress,
    pub ip: Ipv4Addr,
    pub subnet: Ipv4Addr,
    pub default_gateway: Ipv4Addr,
    pub dns_ip: Ipv4Addr,
    pub dns_name: heapless::String<DNS_NAME_LENGTH>,
}

pub enum EthernetOverEthercatInfoResult {
    FrameInfo2(u16),
    Result(u16),
}

pub struct EthernetOverEthercat {
    pub mailbox_header: MailboxHeader,
    pub frame_info1: u16,
    pub info_result: EthernetOverEthercatInfoResult,
    pub data: [u8; MAX_EOE_DATA_LENGTH],
}

pub fn eoe_defaine_hook(context: &mut Context, hook: &mut [Box<dyn Any>]) -> i32 {
    todo!()
}

pub fn eoe_set_ip(
    context: &mut Context,
    slave: u16,
    port: u8,
    ip_parameter: &mut EoeParameter,
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn eoe_get_ip(
    context: &mut Context,
    slave: u16,
    port: u8,
    ip_parameter: &mut EoeParameter,
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn eoe_send(
    context: &mut Context,
    slave: u16,
    port: u8,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn eoe_receive(
    context: &mut Context,
    slave: u16,
    port: u8,
    size: &mut usize,
    pointer: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn eoe_read_fragment(
    mailbox_in: &mut MailboxBuffer,
    rx_fragment_number: &mut u8,
    rx_frame_offset: &mut u16,
    rx_frame_number: &mut u16,
    size: &mut usize,
    pointer: &mut [Box<dyn Any>],
) -> i32 {
    todo!()
}
