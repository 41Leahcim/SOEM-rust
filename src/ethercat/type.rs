use core::slice;
use std::{
    array,
    time::{Duration, SystemTime},
};

use num_traits::PrimInt;

use crate::oshw::htons;

/// Possible error codes returned
#[derive(Debug)]
pub enum Error {
    /// No frame returned
    NoFrame = -1,

    /// Unknown frame received
    OtherFramce = -2,

    /// General error
    Error = -3,

    /// Too many slaves
    SlaveCountExceeded = -4,

    /// Request timeout
    Timeout = -5,

    /// Library already initialized
    AlreadyInitialized = 0,

    /// Library not initialized
    NotInitialized,

    /// No slaves were found
    NoSlaves,

    /// Function failed
    FunctionFailed,
}

/// Maximum EtherCAT frame length in bytes
pub const MAX_ECAT_FRAME_LENGTH: usize = 1518;

const DATA_SIZE: usize = 2;
const DATAGRAM_HEADER_SIZE: usize = 10;
const FCS_SIZE: usize = 4;

/// Maximum EtherCAT LRW frame length in bytes
pub const MAX_LRW_DATA_LENGTH: usize = MAX_ECAT_FRAME_LENGTH
    - ETHERNET_HEADER_SIZE
    - DATA_SIZE
    - DATAGRAM_HEADER_SIZE
    - ETHERCAT_WORK_COUNTER_SIZE
    - FCS_SIZE;

/// Size of DC datagram used in first LRW frame
pub const FIRST_DC_DATAGRAM_SIZE: usize = 20;

/// Standard frame buffer size in bytes
pub const BUFSIZE: usize = MAX_ECAT_FRAME_LENGTH;

/// Datagram type EtherCAT
pub const ECATTYPE: u16 = 0x1000;

/// Number of frame buffers per channel
pub const MAX_BUF_COUNT: usize = 16;

/// Timeout value for tx frame to return to rx
pub const TIMEOUT_RETURN: Duration = Duration::from_micros(2000);

/// Timeout value for safe data transfer, max. triple retry
pub const TIMEOUT_RET3: Duration = Duration::from_micros(2000 * 3);

/// Timeout value for return "safe" variant (f.e. wireless)
pub const TIMEOUT_SAFE: Duration = Duration::from_micros(20_000);

/// Timeout value for EEPROM access
pub const TIMEOUT_EEP: Duration = Duration::from_micros(20_000);

/// Timeout value for tx mailbox cycle
pub const TIMEOUT_TX_MAILBOX: Duration = Duration::from_micros(20_000);

/// Timeout value for rx mailbox cycle
pub const TIMEOUT_RX_MAILBOX: Duration = Duration::from_micros(700_000);

/// Timeout value for check statechange
pub const TIMEOUT_STATE: Duration = Duration::from_micros(2_000_000);

/// Size of EEPROM bitmap cache
pub const MAX_EEP_BITMAP_SIZE: u16 = 128;

/// Size of EEPROM cache buffer
pub const MAX_EEP_BUF_SIZE: u16 = MAX_EEP_BITMAP_SIZE << 5;

/// Default number of retries if WKC <= 0
pub const DEFAULT_RETRIES: u8 = 3;

/// Default group size in 2^x
pub const LOG_GROUP_OFFSET: u8 = 16;

pub type Buffer = heapless::Vec<u8, BUFSIZE>;

#[derive(Debug)]
pub enum EthernetHeaderError {
    WrongSize,
}

/// Ethernet header defenition
pub struct EthernetHeader {
    /// Destination MAC
    pub destination_address: [u16; 3],

    // Source MAC
    pub source_address: [u16; 3],

    /// Ethernet type
    pub etype: u16,
}

impl EthernetHeader {
    pub fn new(mac: [u16; 3]) -> Self {
        let destination_address = array::from_fn(|_| htons(0xFFFF));
        let source_address = array::from_fn(|i| htons(mac[i]));
        let etype = htons(ETH_P_ECAT);
        Self {
            destination_address,
            source_address,
            etype,
        }
    }
}

impl TryFrom<&[u8]> for EthernetHeader {
    type Error = EthernetHeaderError;

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() < size_of::<Self>() {
            return Err(EthernetHeaderError::WrongSize);
        }
        let mut words = value
            .chunks(2)
            .map(|chunk| u16::from_le_bytes(array::from_fn(|i| chunk[i])));
        let destination_address = array::from_fn(|_| words.next().unwrap());
        let source_address = array::from_fn(|_| words.next().unwrap());
        let etype = words.next().unwrap();
        Ok(Self {
            destination_address,
            source_address,
            etype,
        })
    }
}

impl TryFrom<&[u8]> for &EthernetHeader {
    type Error = EthernetHeaderError;

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() < size_of::<Self>() {
            return Err(EthernetHeaderError::WrongSize);
        }
        Ok(unsafe { &slice::from_raw_parts((value as *const [u8]).cast(), 1)[0] })
    }
}

impl TryFrom<&mut [u8]> for &mut EthernetHeader {
    type Error = EthernetHeaderError;

    fn try_from(value: &mut [u8]) -> Result<Self, Self::Error> {
        if value.len() < size_of::<Self>() {
            return Err(EthernetHeaderError::WrongSize);
        }
        Ok(unsafe { &mut slice::from_raw_parts_mut((value as *mut [u8]).cast(), 1)[0] })
    }
}

impl AsRef<[u8]> for EthernetHeader {
    fn as_ref(&self) -> &[u8] {
        unsafe {
            slice::from_raw_parts(
                (self as *const EthernetHeader).cast(),
                size_of::<EthernetHeader>(),
            )
        }
    }
}

impl AsMut<[u8]> for EthernetHeader {
    fn as_mut(&mut self) -> &mut [u8] {
        unsafe {
            slice::from_raw_parts_mut(
                (self as *mut EthernetHeader).cast(),
                size_of::<EthernetHeader>(),
            )
        }
    }
}

/// Ethernet header size
pub const ETHERNET_HEADER_SIZE: usize = size_of::<EthernetHeader>();

#[derive(Debug)]
pub enum EthercatHeaderError {
    WrongSize,
}

/// EtherCat datagram header defenition
pub struct EthercatHeader {
    /// Length of etherCAT datagram
    pub ethercat_length: u16,

    /// EtherCAT command
    pub command: CommandType,

    /// Index used in SOEM for Tx to Rx recombination
    pub index: u8,

    /// EtherCAT address
    pub address_position: u16,
    pub address_offset: u16,

    /// Length of data position in datagram
    pub data_length: u16,

    /// Interrupt, currently unused
    pub interrupt: u16,
}

impl TryFrom<&[u8]> for &EthercatHeader {
    type Error = EthercatHeaderError;

    fn try_from(value: &[u8]) -> Result<Self, Self::Error> {
        if value.len() < size_of::<Self>() {
            return Err(EthercatHeaderError::WrongSize);
        }
        Ok(unsafe { &slice::from_raw_parts((value as *const [u8]).cast(), 1)[0] })
    }
}

impl TryFrom<&mut [u8]> for &mut EthercatHeader {
    type Error = EthercatHeaderError;

    fn try_from(value: &mut [u8]) -> Result<Self, Self::Error> {
        if value.len() < size_of::<Self>() {
            return Err(EthercatHeaderError::WrongSize);
        }
        Ok(unsafe { &mut slice::from_raw_parts_mut((value as *mut [u8]).cast(), 1)[0] })
    }
}

impl AsRef<[u8]> for EthercatHeader {
    fn as_ref(&self) -> &[u8] {
        unsafe {
            slice::from_raw_parts(
                (self as *const EthercatHeader).cast(),
                size_of::<EthercatHeader>(),
            )
        }
    }
}

impl AsMut<[u8]> for EthercatHeader {
    fn as_mut(&mut self) -> &mut [u8] {
        unsafe {
            slice::from_raw_parts_mut(
                (self as *mut EthercatHeader).cast(),
                size_of::<EthercatHeader>(),
            )
        }
    }
}

/// EtherCAT header size
pub const ETHERCAT_HEADER_SIZE: usize = size_of::<EthercatHeader>();

/// Size of `EthercatHeader.ethercat_length`
pub const ETHERCAT_LENGTH_SIZE: usize = size_of::<u16>();

/// Offset of command in EtherCAT header
pub const ETHERCAT_COMMAND_OFFET: usize = ETHERCAT_LENGTH_SIZE;

/// Size of workcounter item in EtherCAT datagram
pub const ETHERCAT_WORK_COUNTER_SIZE: usize = size_of::<u16>();

/// Definition of datagram follows bit in EthercatHeader.data_length
pub const DATAGRAM_FOLLOWS: u16 = 1 << 15;

pub enum EthercatState {
    /// No valid state
    None,

    /// Init state
    Init,

    /// Pre-operational
    PreOperational,

    /// Boot state
    Boot,

    /// Safe-operational
    SafeOperational,

    /// Operational
    Operational = 8,

    /// (ACK) error
    Error = 0x10,
}

/// Possible buffer states
#[derive(Debug, Clone, Copy)]
pub enum BufferState {
    /// Empty
    Empty,

    /// Allocated but not failed
    Alloc,

    /// Transmitted
    Tx,

    /// Received but not consumed
    Rcvd,

    /// Cycle complete
    Complete,
}

/// Ethercat data types
pub enum Datatype {
    Boolean = 1,
    Integer8,
    Integer16,
    Integer32,
    Unsigned8,
    Unsigned16,
    Unsigned32,
    Real32,
    VisibleString,
    OctetString,
    UnicodeString,
    TimeOfDay,
    TimeDifference,
    Domain = 0xF,
    Integer24,
    Real64,
    Integer64 = 0x15,
    Unsigned24,
    Unsigned64 = 0x1B,
    Bit1 = 0x30,
    Bit2,
    Bit3,
    Bit4,
    Bit5,
    Bit6,
    Bit7,
    Bit8,
}

/// Ethernet command types
#[derive(Debug, Clone, Copy)]
pub enum CommandType {
    /// No operation
    Nop,

    /// Auto increment read
    AutoPointerRead,

    /// Auto increment write
    AutoPointerWrite,

    /// Auto increment read/write
    AutoPointerReadWrite,

    /// Configured address read
    FixedPointerRead,

    /// Configured address write
    FixedPointerWrite,

    /// Configured address read/write
    FixedPointerReadWrite,

    /// Broadcast read
    BroadcastRead,

    /// Broadcast write
    BroadcastWrite,

    /// Broadcast read/write
    BroadcastReadWrite,

    /// Logical memory read
    LogicalRead,

    /// Logical memory write
    LogicalWrite,

    /// Logical memory read/write
    LogicalReadWrite,

    /// Auto increment read multiple write
    AutoReadMultipleWrite,

    /// Configured read multiple write
    FixedReadMultipleWrite,
}

impl From<CommandType> for u8 {
    fn from(value: CommandType) -> Self {
        match value {
            CommandType::Nop => 0,
            CommandType::AutoPointerRead => 1,
            CommandType::AutoPointerWrite => 2,
            CommandType::AutoPointerReadWrite => 3,
            CommandType::FixedPointerRead => 4,
            CommandType::FixedPointerWrite => 5,
            CommandType::FixedPointerReadWrite => 6,
            CommandType::BroadcastRead => 7,
            CommandType::BroadcastWrite => 8,
            CommandType::BroadcastReadWrite => 9,
            CommandType::LogicalRead => 10,
            CommandType::LogicalWrite => 11,
            CommandType::LogicalReadWrite => 12,
            CommandType::AutoReadMultipleWrite => 13,
            CommandType::FixedReadMultipleWrite => 14,
        }
    }
}

pub enum EepromCommandType {
    /// No operation
    Nop = 0x0,

    /// Read
    Read = 0x100,

    /// Write
    Write = 0x201,

    Reload = 0x300,
}

/// EEprom state machine read size
pub const EEPROM_STATE_MACHINE_READ_SIZE: u16 = 0x40;

/// EEprom state machine busy flag
pub const EEPROM_STATE_MACHINE_BUSY: u16 = 0x8000;

/// EEprom state machine error flag mask
pub const EEPROM_STATE_MACHINE_ERROR_MASK: u16 = 0x7800;

/// EEPROM state machine error acknowledge
pub const EEPROM_STATE_MACHINE_ERROR_NACK: u16 = 0x2000;

pub const SII_START: u16 = 0x40;

pub enum SiiCategory {
    /// SII category strings
    String = 10,

    /// SII category general
    General = 30,

    /// SII category Fieldbus Memory Management Unit
    FMMU = 40,

    /// SII category Sync manager
    SM = 41,

    /// SII category Process data object
    PDO = 50,
}

/// Item offsets in SII general section
pub enum SiiGeneralItem {
    /// Manufacturer
    Manufacturer = 8,

    /// Product id
    Id = 0xA,

    /// Revision
    Revision = 0xC,

    /// Boot receive mailbox
    BootRxMailbox = 0x14,

    /// Boot send mailbox
    BootTxMailbox = 0x16,

    /// Mailbox size
    MailboxSize = 0x19,

    /// Send mailbox address
    TxMailboxAddress = 0x1A,

    /// Receive mailbox address
    RxMailboxAddress = 0x18,

    MailboxProtocol = 0x1C,
}

pub enum MailboxError {
    InvalidMailboxType(u8),
    InvalidCOEMailboxType(u8),
}

/// Mailbox types
pub enum MailboxType {
    /// Error mailbox
    Error,

    /// ADS over EtherCAT
    AdsOverEthercat,

    /// Ethernet over EtherCAT
    EthernetOverEthercat,

    /// CANopen over EtherCAT
    CanopenOverEthercat,

    /// File over EtherCAT
    FileOverEthercat,

    /// Servo over EtherCAT
    ServoOverEthercat,

    /// Vendor over EtherCAT
    VendorOverEthercat = 0xF,
}

impl From<MailboxType> for u8 {
    fn from(value: MailboxType) -> Self {
        match value {
            MailboxType::Error => 0,
            MailboxType::AdsOverEthercat => 1,
            MailboxType::EthernetOverEthercat => 2,
            MailboxType::CanopenOverEthercat => 3,
            MailboxType::FileOverEthercat => 4,
            MailboxType::ServoOverEthercat => 5,
            MailboxType::VendorOverEthercat => 0xF,
        }
    }
}

impl TryFrom<u8> for MailboxType {
    type Error = MailboxError;

    fn try_from(value: u8) -> Result<Self, MailboxError> {
        match value {
            0 => Ok(MailboxType::Error),
            1 => Ok(MailboxType::AdsOverEthercat),
            2 => Ok(MailboxType::EthernetOverEthercat),
            3 => Ok(MailboxType::CanopenOverEthercat),
            4 => Ok(MailboxType::FileOverEthercat),
            5 => Ok(MailboxType::ServoOverEthercat),
            0xF => Ok(MailboxType::VendorOverEthercat),
            _ => Err(MailboxError::InvalidMailboxType(value)),
        }
    }
}

/// CANopen over EtherCat mailbox types
pub enum COEMailboxType {
    Emergency = 1,

    /// Service Data Object request
    SdoRequest,

    /// Service Data Object response
    SdoResponse,

    /// Send process data object
    TxPdo,

    /// Rceive process data object
    RxPdo,

    /// Send process data object RR
    TxPdoRR,

    /// Receive proces data object RR
    RxPdoRR,

    /// Service data object information
    SdoInfo,
}

impl TryFrom<u8> for COEMailboxType {
    type Error = MailboxError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Emergency),
            2 => Ok(Self::SdoRequest),
            3 => Ok(Self::SdoResponse),
            4 => Ok(Self::TxPdo),
            5 => Ok(Self::RxPdo),
            6 => Ok(Self::TxPdoRR),
            7 => Ok(Self::RxPdoRR),
            8 => Ok(Self::SdoInfo),
            _ => Err(MailboxError::InvalidCOEMailboxType(value)),
        }
    }
}

impl From<COEMailboxType> for u8 {
    fn from(value: COEMailboxType) -> Self {
        match value {
            COEMailboxType::Emergency => 1,
            COEMailboxType::SdoRequest => 2,
            COEMailboxType::SdoResponse => 3,
            COEMailboxType::TxPdo => 4,
            COEMailboxType::RxPdo => 5,
            COEMailboxType::TxPdoRR => 6,
            COEMailboxType::RxPdoRR => 7,
            COEMailboxType::SdoInfo => 8,
        }
    }
}

impl From<COEMailboxType> for u16 {
    fn from(value: COEMailboxType) -> Self {
        u8::from(value).into()
    }
}

/// CANopen over EtherCAT Service Data Object commands
#[derive(Debug, Clone, Copy)]
pub enum CanopenOverEthercatSdoCommand {
    /// Download initiate
    DownInit = 0x21,

    /// Download expand
    DownExp = 0x23,

    /// Download initiate calibration
    DownInitCa = 0x31,

    /// Upload request
    UpReq = 0x40,

    /// Upload request calibration
    UpReqCa = 0x50,

    /// Segment upload request
    SegUpReq = 0x60,

    /// Abort
    Abort = 0x80,
}

impl From<CanopenOverEthercatSdoCommand> for u8 {
    fn from(value: CanopenOverEthercatSdoCommand) -> Self {
        match value {
            CanopenOverEthercatSdoCommand::DownInit => 0x21,
            CanopenOverEthercatSdoCommand::DownExp => 0x23,
            CanopenOverEthercatSdoCommand::DownInitCa => 0x31,
            CanopenOverEthercatSdoCommand::UpReq => 0x40,
            CanopenOverEthercatSdoCommand::UpReqCa => 0x50,
            CanopenOverEthercatSdoCommand::SegUpReq => 0x60,
            CanopenOverEthercatSdoCommand::Abort => 0x80,
        }
    }
}

/// CANopen over EtherCAT object description command
pub enum COEObjectDescriptionCommand {
    ObjectDictionaryListRequest = 1,
    ObjectDictionaryListResponse,
    ObjectDictionaryRequest,
    ObjectDictionaryResponse,
    OeRequest,
    OeResponse,
    ServiceDataObjectInformationError,
}

pub enum FileOverEthercatOpcodes {
    Read = 1,
    Write,
    Data,
    Ack,
    Error,
    Busy,
}

pub enum ServoOverEthercatOpcodes {
    ReadRequest = 1,
    ReadResponse,
    WriteRequest,
    WriteResponse,
    Notification,
    Emergency,
}

pub enum EthercatRegisters {
    Type,
    PortDescriptor = 7,

    /// EtherCAT slave control and status unit protocol
    EscSup = 8,

    /// Standardized device profile for real-time Ethernet
    StaDr = 0x10,

    Alias = 0x12,
    DeviceLayerControl = 0x100,
    DeviceLayerPort = 0x101,
    DeviceLayerAlias = 0x103,
    DeviceLayerStatus = 0x110,
    ApplicationLayerControl = 0x120,
    ApplicationLayerStatus = 0x130,
    ApplicationLayerStatusCode = 0x134,
    ProcessDataInterfaceControl = 0x140,
    InterruptMask = 0x200,
    ReceiveError = 0x300,
    FatalReceiveError = 0x308,
    EthercatProtocolUpdateErrorCount = 0x30C,
    ProcessErrorCount = 0x30D,
    ProcessErrorCode = 0x30E,
    LinkLayerCount = 0x310,
    WatchdogCount = 0x442,
    EepromConfig = 0x500,

    /// EEPROM control and status
    EepromControlStat = 0x502,

    EepromAddress = 0x504,
    EepromData = 0x508,

    FieldbusMemoryManagementUnit0 = 0x600,
    FieldbusMemoryManagementUnit1 = 0x610,
    FieldbusMemoryManagementUnit2 = 0x620,
    FieldbusMemoryManagementUnit3 = 0x630,

    SyncManager0 = 0x800,
    SyncManager1 = 0x808,
    SyncManager2 = 0x810,
    SyncManager3 = 0x818,

    SyncManager0Status = 0x805,
    SyncManager1Status = 0x808 + 5,
    SyncManager1Act = 0x808 + 6,
    SyncManager1Control = 0x808 + 7,

    DistributedClockTime0 = 0x900,
    DistributedClockTime1 = 0x904,
    DistributedClockTime2 = 0x908,
    DistributedClockTime3 = 0x90C,

    DistributedClockSystemTime = 0x910,
    DistributedClockStartOfFrame = 0x918,
    DistributedClockSystemOffset = 0x920,
    DistributedClockSystemDelay = 0x928,
    DistributedClockSystemDifference = 0x92C,
    DistributedClockSpeedCount = 0x930,
    DistributedClockTimeFilter = 0x934,
    DistributedClockControlUnit = 0x980,
    DistributedClockSynchronizationActive = 0x981,
    DistributedClockCycle0 = 0x9A0,
    DistributedClockCycle1 = 0x9A4,
}

impl From<EthercatRegisters> for u16 {
    fn from(value: EthercatRegisters) -> Self {
        match value {
            EthercatRegisters::Type => 0,
            EthercatRegisters::PortDescriptor => 7,
            EthercatRegisters::EscSup => 8,
            EthercatRegisters::StaDr => 0x10,
            EthercatRegisters::Alias => 0x12,
            EthercatRegisters::DeviceLayerControl => 0x100,
            EthercatRegisters::DeviceLayerPort => 0x101,
            EthercatRegisters::DeviceLayerAlias => 0x103,
            EthercatRegisters::DeviceLayerStatus => 0x110,
            EthercatRegisters::ApplicationLayerControl => 0x120,
            EthercatRegisters::ApplicationLayerStatus => 0x130,
            EthercatRegisters::ApplicationLayerStatusCode => 0x134,
            EthercatRegisters::ProcessDataInterfaceControl => 0x140,
            EthercatRegisters::InterruptMask => 0x200,
            EthercatRegisters::ReceiveError => 0x300,
            EthercatRegisters::FatalReceiveError => 0x308,
            EthercatRegisters::EthercatProtocolUpdateErrorCount => 0x30C,
            EthercatRegisters::ProcessErrorCount => 0x30D,
            EthercatRegisters::ProcessErrorCode => 0x30E,
            EthercatRegisters::LinkLayerCount => 0x310,
            EthercatRegisters::WatchdogCount => 0x442,
            EthercatRegisters::EepromConfig => 0x500,
            EthercatRegisters::EepromControlStat => 0x502,
            EthercatRegisters::EepromAddress => 0x504,
            EthercatRegisters::EepromData => 0x508,
            EthercatRegisters::FieldbusMemoryManagementUnit0 => 0x600,
            EthercatRegisters::FieldbusMemoryManagementUnit1 => 0x610,
            EthercatRegisters::FieldbusMemoryManagementUnit2 => 0x620,
            EthercatRegisters::FieldbusMemoryManagementUnit3 => 0x630,
            EthercatRegisters::SyncManager0 => 0x800,
            EthercatRegisters::SyncManager1 => 0x808,
            EthercatRegisters::SyncManager2 => 0x810,
            EthercatRegisters::SyncManager3 => 0x818,
            EthercatRegisters::SyncManager0Status => 0x805,
            EthercatRegisters::SyncManager1Status => 0x808 + 5,
            EthercatRegisters::SyncManager1Act => 0x808 + 6,
            EthercatRegisters::SyncManager1Control => 0x808 + 7,
            EthercatRegisters::DistributedClockTime0 => 0x900,
            EthercatRegisters::DistributedClockTime1 => 0x904,
            EthercatRegisters::DistributedClockTime2 => 0x908,
            EthercatRegisters::DistributedClockTime3 => 0x90C,
            EthercatRegisters::DistributedClockSystemTime => 0x910,
            EthercatRegisters::DistributedClockStartOfFrame => 0x918,
            EthercatRegisters::DistributedClockSystemOffset => 0x920,
            EthercatRegisters::DistributedClockSystemDelay => 0x928,
            EthercatRegisters::DistributedClockSystemDifference => 0x92C,
            EthercatRegisters::DistributedClockSpeedCount => 0x930,
            EthercatRegisters::DistributedClockTimeFilter => 0x934,
            EthercatRegisters::DistributedClockControlUnit => 0x980,
            EthercatRegisters::DistributedClockSynchronizationActive => 0x981,
            EthercatRegisters::DistributedClockCycle0 => 0x9A0,
            EthercatRegisters::DistributedClockCycle1 => 0x9A4,
        }
    }
}

/// Service data object sync manager communication type
pub const SDO_SCOMMTYPE: u16 = 0x1C00;

/// Service Data Object - Process Data Object assignment
pub const SDO_PDO_ASSIGNMENT: u16 = 0x1C10;

/// Service Data Object - received Process Data Object assignment
pub const SDO_RX_PDO_ASSIGN: u16 = 0x1C12;

/// Service Data Object - Send Process Data Object assignment
pub const SDO_TX_PDO_ASSIGN: u16 = 0x1C13;

/// Ethercat packet type
pub const ETH_P_ECAT: u16 = 0x88A4;

pub enum ErrorType {
    ServiceDataObjectError,
    Emergency,
    PacketError = 3,

    /// Service data object information error
    SdoInfoError,

    FileOverEthercatError,

    /// File over ethercat buffer too small
    FileOverEthernetBufferTooSmall,

    PacketNumber,
    ServerOverEthercatError,
    MailboxError,
    FileOverEthernetFileNotFoundError,
    EthernetOverEthercatInvalidRxData,
}

pub enum AbortError {
    Abort(i32),
    Error {
        error_code: u16,
        error_register: u8,
        b1: u8,
        w1: u16,
        w2: u16,
    },
}

pub struct ErrorInfo {
    /// TIme at which the error was generated
    pub time: SystemTime,

    /// Signal bit, error set but not read
    pub signal: bool,

    /// Slave number that generated the error
    pub slave: u16,

    /// GoE Service Data Object index that generated the error
    pub index: u16,

    /// GoE Service Data Object subindex that generated the error
    pub sub_index: u8,

    /// Type of error
    pub error_type: ErrorType,

    pub abort_error: AbortError,
}

/// Sets the count value in the mailbox header.
/// A u16 is used to allow the maximum range of values
pub const fn mailbox_header_set_count(count: u8) -> u8 {
    count << 4
}

/// Make a word from 2 bytes
pub const fn make_word(most_significant: u8, least_significant: u8) -> u16 {
    (most_significant as u16) << 8 | least_significant as u16
}

pub const fn high_byte(word: u16) -> u8 {
    (word >> 8) as u8
}

pub const fn low_byte(word: u16) -> u8 {
    (word & 0xff) as u8
}

pub const fn low_word(dword: u32) -> u16 {
    (dword & 0xFFFF) as u16
}

pub const fn high_word(dword: u32) -> u16 {
    (dword >> 16) as u16
}

pub fn host_to_ethercat<Int: PrimInt>(value: Int) -> Int {
    value.to_le()
}

pub fn ethercat_to_host<Int: PrimInt>(value: Int) -> Int {
    if cfg!(target_endian = "big") {
        value.to_be()
    } else {
        value
    }
}
