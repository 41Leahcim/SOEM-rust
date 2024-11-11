//! General types and constants for EtherCAT.
//!
//! Constants that could need optimization for specific applications
//! are the `EC_TIMEOUTxxx`. Assumptions for the standard settings are a
//! standard Linux PC or laptop and a wired connection maximal 100 slaves.
//! For use with wireless connections or lots of slaves, the timeouts need
//! increasing. For fast systems running Xenomai and RT-net or alike the
//! timeouts need to be shorter.

use std::{
    array,
    io::{self, Read, Write},
    time::Duration,
};

use num_traits::PrimInt;

use crate::oshw::Network;

use super::{
    main::{MainError, PacketError},
    ReadFrom, WriteTo,
};

/// Maximum EtherCAT frame length in bytes
pub const MAX_ECAT_FRAME_LENGTH: usize = 1518;

const DATA_SIZE: usize = 2;
const DATAGRAM_HEADER_SIZE: usize = 10;
const FCS_SIZE: usize = 4;

/// Maximum EtherCAT LRW frame length in bytes
pub const MAX_LRW_DATA_LENGTH: usize = MAX_ECAT_FRAME_LENGTH
    - EthernetHeader::size()
    - DATA_SIZE
    - DATAGRAM_HEADER_SIZE
    - ETHERCAT_WORK_COUNTER_SIZE
    - FCS_SIZE;

/// Size of DC datagram used in first LRW frame
pub const FIRST_DC_DATAGRAM_SIZE: usize = 20;

/// Standard frame buffer size in bytes
pub const BUFFER_SIZE: usize = MAX_ECAT_FRAME_LENGTH;

/// Datagram type EtherCAT
pub const ECATTYPE: u16 = 0x1000;

/// Number of frame buffers per channel
pub const MAX_BUFFER_COUNT: usize = 16;

/// Timeout value for tx frame to return to rx
pub const TIMEOUT_RETURN: Duration = Duration::from_micros(2000);

/// Timeout value for safe data transfer, max. triple retry
pub const TIMEOUT_RETURN3: Duration = Duration::from_micros(2000 * 3);

/// Timeout value for return "safe" variant (f.e. wireless)
pub const TIMEOUT_SAFE: Duration = Duration::from_micros(20_000);

/// Timeout value for EEPROM access
pub const TIMEOUT_EEPROM: Duration = Duration::from_micros(20_000);

/// Timeout value for tx mailbox cycle
pub const TIMEOUT_TX_MAILBOX: Duration = Duration::from_micros(20_000);

/// Timeout value for rx mailbox cycle
pub const TIMEOUT_RX_MAILBOX: Duration = Duration::from_micros(700_000);

/// Timeout value for check statechange
pub const TIMEOUT_STATE: Duration = Duration::from_micros(2_000_000);

/// Size of EEPROM bitmap cache
pub const MAX_EEPROM_BITMAP_SIZE: u16 = 128;

/// Size of EEPROM cache buffer
pub const MAX_EEPROM_BUFFER_SIZE: u16 = MAX_EEPROM_BITMAP_SIZE << 5;

/// Default number of retries if WKC <= 0
pub const DEFAULT_RETRIES: u8 = 3;

/// Default group size in 2^x
pub const LOG_GROUP_OFFSET: u8 = 16;

pub type Buffer = heapless::Vec<u8, BUFFER_SIZE>;

/// Ethernet header defenition
#[derive(Debug, Clone, Copy)]
pub struct EthernetHeader {
    /// Destination MAC
    destination_address: [Network<u16>; 3],

    // Source MAC
    source_address: [Network<u16>; 3],

    /// Ethernet type
    etype: Network<u16>,
}

impl EthernetHeader {
    pub fn new(source_mac: [u16; 3]) -> Self {
        let destination_address = array::from_fn(|_| Network::from_host(0xFFFF));
        let source_address = array::from_fn(|i| Network::from_host(source_mac[i]));
        let etype = Network::from_host(ETH_P_ECAT);
        Self {
            destination_address,
            source_address,
            etype,
        }
    }

    pub const fn source_address(&self) -> &[Network<u16>; 3] {
        &self.source_address
    }

    pub fn source_address_mut(&mut self) -> &mut [Network<u16>; 3] {
        &mut self.source_address
    }

    pub const fn ethernet_type(&self) -> Network<u16> {
        self.etype
    }

    pub const fn size() -> usize {
        size_of::<u16>() * 7
    }

    /// # Errors
    /// Returns an error if the Ethernetheader couldn't be converted to bytes.
    pub fn bytes(&self) -> io::Result<[u8; Self::size()]> {
        let mut result = [0; Self::size()];
        self.write_to(&mut result.as_mut_slice())?;
        Ok(result)
    }
}

impl<R: Read> ReadFrom<R> for EthernetHeader {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let destination_address = (0..3).try_fold([Network::default(); 3], |result, index| {
            let mut result = result;
            result[index] = Network::<u16>::from_bytes(Self::read_bytes(reader)?);
            Ok::<_, io::Error>(result)
        })?;
        let source_address = (0..3).try_fold([Network::default(); 3], |result, index| {
            let mut result = result;
            result[index] = Network::<u16>::from_bytes(Self::read_bytes(reader)?);
            Ok::<_, io::Error>(result)
        })?;
        let etype = Network::<u16>::from_bytes(Self::read_bytes(reader)?);
        Ok(Self {
            destination_address,
            source_address,
            etype,
        })
    }
}

impl<W: Write> WriteTo<W> for EthernetHeader {
    fn write_to(&self, writer: &mut W) -> io::Result<()> {
        [
            self.destination_address.as_slice(),
            self.source_address.as_slice(),
            [self.etype].as_slice(),
        ]
        .into_iter()
        .flatten()
        .try_fold((), |(), word| writer.write_all(&word.to_bytes()))
    }
}

#[derive(Debug)]
pub enum EthercatHeaderError {
    Io(io::Error),
    InvalidCommand(InvalidCommand),
}

impl From<io::Error> for EthercatHeaderError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<InvalidCommand> for EthercatHeaderError {
    fn from(value: InvalidCommand) -> Self {
        Self::InvalidCommand(value)
    }
}

/// EtherCat datagram header defenition
#[derive(Debug, Clone, Copy)]
pub struct EthercatHeader {
    /// Length of etherCAT datagram
    ethercat_length: Ethercat<u16>,

    /// EtherCAT command
    command: Command,

    /// Index used in SOEM for Tx to Rx recombination
    index: u8,

    /// EtherCAT address
    address_position: Ethercat<u16>,
    address_offset: Ethercat<u16>,

    /// Length of data position in datagram
    data_length: Ethercat<u16>,

    /// Interrupt, currently unused
    interrupt: Ethercat<u16>,
}

impl<R: Read> ReadFrom<R> for EthercatHeader {
    type Err = EthercatHeaderError;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let ethercat_length = Ethercat::<u16>::from_bytes(Self::read_bytes(reader)?);
        let command = Command::try_from(Self::read_byte(reader)?)?;
        let index = Self::read_byte(reader)?;
        let address_position = Ethercat::<u16>::from_bytes(Self::read_bytes(reader)?);
        let address_offset = Ethercat::<u16>::from_bytes(Self::read_bytes(reader)?);
        let data_length = Ethercat::<u16>::from_bytes(Self::read_bytes(reader)?);
        let interrupt = Ethercat::<u16>::from_bytes(Self::read_bytes(reader)?);
        Ok(Self {
            ethercat_length,
            command,
            index,
            address_position,
            address_offset,
            data_length,
            interrupt,
        })
    }
}

impl<W: Write> WriteTo<W> for EthercatHeader {
    fn write_to(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&self.ethercat_length.to_bytes())?;
        writer.write_all(&[u8::from(self.command), self.index])?;
        writer.write_all(&self.address_position.to_bytes())?;
        writer.write_all(&self.address_offset.to_bytes())?;
        writer.write_all(&self.data_length.to_bytes())?;
        writer.write_all(&self.interrupt.to_bytes())?;
        Ok(())
    }
}

impl EthercatHeader {
    pub const fn size() -> usize {
        5 * size_of::<u16>() + 2 * size_of::<u8>()
    }

    /// # Errors
    /// Returns an error if the EtherCAT header couldn't be converted to bytes.
    pub fn bytes(&self) -> io::Result<[u8; Self::size()]> {
        let mut result = [0; Self::size()];
        self.write_to(&mut result.as_mut_slice())?;
        Ok(result)
    }

    pub const fn new(
        ethercat_length: Ethercat<u16>,
        command: Command,
        index: u8,
        address_position: Ethercat<u16>,
        address_offset: Ethercat<u16>,
        data_length: Ethercat<u16>,
        interrupt: Ethercat<u16>,
    ) -> Self {
        Self {
            ethercat_length,
            command,
            index,
            address_position,
            address_offset,
            data_length,
            interrupt,
        }
    }

    pub fn ethercat_length_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.ethercat_length
    }

    pub const fn ethercat_length(&self) -> Ethercat<u16> {
        self.ethercat_length
    }

    pub fn data_length_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.data_length
    }

    pub const fn data_length(&self) -> Ethercat<u16> {
        self.data_length
    }

    pub fn command_mut(&mut self) -> &mut Command {
        &mut self.command
    }

    pub const fn index(&self) -> u8 {
        self.index
    }

    pub fn index_mut(&mut self) -> &mut u8 {
        &mut self.index
    }

    pub fn address_position_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.address_position
    }

    pub fn address_offset_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.address_offset
    }
}

/// Size of `EthercatHeader.ethercat_length`
pub const ETHERCAT_LENGTH_SIZE: usize = size_of::<u16>();

/// Offset of command in EtherCAT header
pub const ETHERCAT_COMMAND_OFFET: usize = ETHERCAT_LENGTH_SIZE;

/// Size of workcounter item in EtherCAT datagram
pub const ETHERCAT_WORK_COUNTER_SIZE: usize = size_of::<u16>();

/// Definition of datagram follows bit in EthercatHeader.data_length
pub const DATAGRAM_FOLLOWS: u16 = 1 << 15;

#[derive(Debug, Clone, Copy)]
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

impl From<EthercatState> for u8 {
    fn from(value: EthercatState) -> Self {
        value as Self
    }
}

impl From<EthercatState> for u16 {
    fn from(value: EthercatState) -> Self {
        Self::from(u8::from(value))
    }
}

impl TryFrom<u8> for EthercatState {
    type Error = MainError;

    fn try_from(value: u8) -> Result<Self, MainError> {
        match value {
            0 => Ok(Self::None),
            1 => Ok(Self::Init),
            2 => Ok(Self::PreOperational),
            3 => Ok(Self::Boot),
            4 => Ok(Self::SafeOperational),
            8 => Ok(Self::Operational),
            0x10 => Ok(Self::Error),
            _ => Err(MainError::InvalidEthercatState(value)),
        }
    }
}

/// Possible buffer states
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

#[derive(Debug)]
#[expect(dead_code)]
pub struct InvalidDataType(u8);

/// Ethercat data types
#[derive(Debug, Default, Clone, Copy)]
pub enum Datatype {
    #[default]
    Invalid,
    Boolean,
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

impl TryFrom<u8> for Datatype {
    type Error = InvalidDataType;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::Boolean),
            2 => Ok(Self::Integer8),
            3 => Ok(Self::Integer16),
            4 => Ok(Self::Integer32),
            5 => Ok(Self::Unsigned8),
            6 => Ok(Self::Unsigned16),
            7 => Ok(Self::Unsigned32),
            8 => Ok(Self::Real32),
            9 => Ok(Self::VisibleString),
            0xA => Ok(Self::OctetString),
            0xB => Ok(Self::UnicodeString),
            0xC => Ok(Self::TimeOfDay),
            0xD => Ok(Self::TimeDifference),
            0xF => Ok(Self::Domain),
            0x10 => Ok(Self::Integer24),
            0x11 => Ok(Self::Real64),
            0x15 => Ok(Self::Integer64),
            0x16 => Ok(Self::Unsigned24),
            0x1B => Ok(Self::Unsigned64),
            0x30 => Ok(Self::Bit1),
            0x31 => Ok(Self::Bit2),
            0x32 => Ok(Self::Bit3),
            0x33 => Ok(Self::Bit4),
            0x34 => Ok(Self::Bit5),
            0x35 => Ok(Self::Bit6),
            0x36 => Ok(Self::Bit7),
            0x37 => Ok(Self::Bit8),
            _ => Err(InvalidDataType(value)),
        }
    }
}

pub struct InvalidWriteCommand(u8);

#[derive(Debug, Clone, Copy)]
pub enum WriteCommand {
    /// No operation
    Nop = 0,

    /// Auto increment write
    AutoPointerWrite = 2,

    /// Configured address write
    FixedPointerWrite = 5,

    /// Broadcast write
    BroadcastWrite = 8,

    /// Logical memory write
    LogicalWrite = 11,
}

impl From<WriteCommand> for u8 {
    fn from(value: WriteCommand) -> Self {
        value as Self
    }
}

impl TryFrom<u8> for WriteCommand {
    type Error = InvalidWriteCommand;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Nop),
            2 => Ok(Self::AutoPointerWrite),
            5 => Ok(Self::FixedPointerWrite),
            8 => Ok(Self::BroadcastWrite),
            _ => Err(InvalidWriteCommand(value)),
        }
    }
}

#[expect(dead_code)]
#[derive(Debug)]
pub struct InvalidReadCommand(u8);

/// Ethernet command types
#[derive(Debug, Clone, Copy)]
pub enum ReadCommand {
    /// No operation
    Nop = 0,

    /// Auto increment read
    AutoPointerRead = 1,

    /// Auto increment read/write
    AutoPointerReadWrite = 3,

    /// Configured address read
    FixedPointerRead = 4,

    /// Configured address read/write
    FixedPointerReadWrite = 6,

    /// Broadcast read
    BroadcastRead = 7,

    /// Broadcast read/write
    BroadcastReadWrite = 9,

    /// Logical memory read
    LogicalRead = 10,

    /// Logical memory read/write
    LogicalReadWrite = 12,

    /// Auto increment read multiple write
    AutoReadMultipleWrite = 13,

    /// Configured read multiple write
    FixedReadMultipleWrite = 14,
}

impl From<ReadCommand> for u8 {
    fn from(value: ReadCommand) -> Self {
        value as Self
    }
}

impl TryFrom<u8> for ReadCommand {
    type Error = InvalidReadCommand;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Nop),
            1 => Ok(Self::AutoPointerRead),
            3 => Ok(Self::AutoPointerReadWrite),
            4 => Ok(Self::FixedPointerRead),
            6 => Ok(Self::FixedPointerReadWrite),
            7 => Ok(Self::BroadcastRead),
            9 => Ok(Self::BroadcastReadWrite),
            10 => Ok(Self::LogicalRead),
            12 => Ok(Self::LogicalReadWrite),
            13 => Ok(Self::AutoReadMultipleWrite),
            14 => Ok(Self::FixedReadMultipleWrite),
            _ => Err(InvalidReadCommand(value)),
        }
    }
}

#[derive(Debug)]
pub struct InvalidCommand(u8);

#[derive(Debug, Clone, Copy)]
pub enum Command {
    ReadCommand(ReadCommand),
    WriteCommand(WriteCommand),
}

impl From<ReadCommand> for Command {
    fn from(value: ReadCommand) -> Self {
        Self::ReadCommand(value)
    }
}

impl From<WriteCommand> for Command {
    fn from(value: WriteCommand) -> Self {
        Self::WriteCommand(value)
    }
}

impl TryFrom<u8> for Command {
    type Error = InvalidCommand;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        if let Ok(command) = ReadCommand::try_from(value) {
            Ok(command.into())
        } else if let Ok(command) = WriteCommand::try_from(value) {
            Ok(command.into())
        } else {
            Err(InvalidCommand(value))
        }
    }
}

impl From<Command> for u8 {
    fn from(value: Command) -> Self {
        match value {
            Command::ReadCommand(read_command) => Self::from(read_command),
            Command::WriteCommand(write_command) => Self::from(write_command),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub enum EepromCommand {
    /// No operation
    Nop = 0x0,

    /// Read
    Read = 0x100,

    /// Write
    Write = 0x201,

    Reload = 0x300,
}

impl From<EepromCommand> for u16 {
    fn from(value: EepromCommand) -> Self {
        value as Self
    }
}

/// EEprom state machine 8 byte chunk read ability
pub const EEPROM_STATE_MACHINE_READ64: u16 = 0x40;

/// EEprom state machine busy flag
pub const EEPROM_STATE_MACHINE_BUSY: u16 = 0x8000;

/// EEprom state machine error flag mask
pub const EEPROM_STATE_MACHINE_ERROR_MASK: u16 = 0x7800;

/// EEPROM state machine error acknowledge
pub const EEPROM_STATE_MACHINE_ERROR_NACK: u16 = 0x2000;

pub const SII_START: u16 = 0x40;

#[derive(Debug, Clone, Copy)]
pub enum SiiCategory {
    /// SII category strings
    String = 10,

    /// SII category general
    General = 30,

    /// SII category Fieldbus Memory Management Unit
    FMMU = 40,

    /// SII category Sync manager
    SM = 41,

    /// SII category receiving Process Data Object
    PDOReceive = 50,

    /// SII category sending Process Data Object
    PDOSend = 51,
}

impl From<SiiCategory> for u8 {
    fn from(value: SiiCategory) -> Self {
        value as Self
    }
}

impl From<SiiCategory> for u16 {
    fn from(value: SiiCategory) -> Self {
        Self::from(u8::from(value))
    }
}

/// Item offsets in SII general section
#[derive(Debug, Clone, Copy)]
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

impl From<SiiGeneralItem> for u8 {
    fn from(value: SiiGeneralItem) -> Self {
        value as Self
    }
}

impl From<SiiGeneralItem> for u16 {
    fn from(value: SiiGeneralItem) -> Self {
        Self::from(u8::from(value))
    }
}

/// Mailbox types
#[derive(Debug, PartialEq, Eq)]
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
    type Error = MainError;

    fn try_from(value: u8) -> Result<Self, MainError> {
        match value {
            0 => Ok(Self::Error),
            1 => Ok(Self::AdsOverEthercat),
            2 => Ok(Self::EthernetOverEthercat),
            3 => Ok(Self::CanopenOverEthercat),
            4 => Ok(Self::FileOverEthercat),
            5 => Ok(Self::ServoOverEthercat),
            0xF => Ok(Self::VendorOverEthercat),
            _ => Err(MainError::InvalidMailboxType(value)),
        }
    }
}

/// CANopen over EtherCat mailbox types
#[derive(Debug, PartialEq, Eq)]
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
    type Error = MainError;
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
            _ => Err(MainError::InvalidCOEMailboxType(value)),
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
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum COEObjectDescriptionCommand {
    #[default]
    ObjectDesciptionListRequest = 1,
    ObjectDesciptionListResponse,
    ObjectDesciptionRequest,
    ObjectDesciptionResponse,
    ObjectEntryRequest,
    ObjectEntryResponse,
    ServiceDataObjectInformationError,
}

impl From<COEObjectDescriptionCommand> for u8 {
    fn from(value: COEObjectDescriptionCommand) -> Self {
        value as Self
    }
}

impl TryFrom<u8> for COEObjectDescriptionCommand {
    type Error = InvalidCommand;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::ObjectDesciptionListRequest),
            2 => Ok(Self::ObjectDesciptionListResponse),
            3 => Ok(Self::ObjectDesciptionRequest),
            4 => Ok(Self::ObjectDesciptionResponse),
            5 => Ok(Self::ObjectEntryRequest),
            6 => Ok(Self::ObjectEntryResponse),
            7 => Ok(Self::ServiceDataObjectInformationError),
            _ => Err(InvalidCommand(value)),
        }
    }
}

impl COEObjectDescriptionCommand {
    pub const fn size() -> usize {
        size_of::<u8>()
    }

    pub fn is_valid(&self) -> bool {
        (1..=7).contains(&(*self as u8))
    }
}

#[derive(Debug, Clone, Copy)]
#[expect(dead_code)]
pub struct InvalidFOEOpcode(u8);

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum FileOverEthercatOpcode {
    #[default]
    Read = 1,
    Write,
    Data,
    Ack,
    Error,
    Busy,
}

impl From<FileOverEthercatOpcode> for u8 {
    fn from(value: FileOverEthercatOpcode) -> Self {
        value as Self
    }
}

impl TryFrom<u8> for FileOverEthercatOpcode {
    type Error = InvalidFOEOpcode;

    fn try_from(value: u8) -> Result<Self, InvalidFOEOpcode> {
        match value {
            1 => Ok(Self::Read),
            2 => Ok(Self::Write),
            3 => Ok(Self::Data),
            4 => Ok(Self::Ack),
            5 => Ok(Self::Error),
            6 => Ok(Self::Busy),
            _ => Err(InvalidFOEOpcode(value)),
        }
    }
}

pub struct InvalidServoOverEthercatOpcode(u8);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ServoOverEthercatOpcode {
    ReadRequest = 1,
    ReadResponse,
    WriteRequest,
    WriteResponse,
    Notification,
    Emergency,
}

impl From<ServoOverEthercatOpcode> for u8 {
    fn from(value: ServoOverEthercatOpcode) -> Self {
        value as Self
    }
}

impl TryFrom<u8> for ServoOverEthercatOpcode {
    type Error = InvalidServoOverEthercatOpcode;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(Self::ReadRequest),
            2 => Ok(Self::ReadResponse),
            3 => Ok(Self::WriteRequest),
            4 => Ok(Self::WriteResponse),
            5 => Ok(Self::Notification),
            6 => Ok(Self::Emergency),
            _ => Err(InvalidServoOverEthercatOpcode(value)),
        }
    }
}

pub enum EthercatRegister {
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
    DistributedClockStart0 = 0x990,
    DistributedClockCycle0 = 0x9A0,
    DistributedClockCycle1 = 0x9A4,
}

impl From<EthercatRegister> for u16 {
    fn from(value: EthercatRegister) -> Self {
        match value {
            EthercatRegister::Type => 0,
            EthercatRegister::PortDescriptor => 7,
            EthercatRegister::EscSup => 8,
            EthercatRegister::StaDr => 0x10,
            EthercatRegister::Alias => 0x12,
            EthercatRegister::DeviceLayerControl => 0x100,
            EthercatRegister::DeviceLayerPort => 0x101,
            EthercatRegister::DeviceLayerAlias => 0x103,
            EthercatRegister::DeviceLayerStatus => 0x110,
            EthercatRegister::ApplicationLayerControl => 0x120,
            EthercatRegister::ApplicationLayerStatus => 0x130,
            EthercatRegister::ApplicationLayerStatusCode => 0x134,
            EthercatRegister::ProcessDataInterfaceControl => 0x140,
            EthercatRegister::InterruptMask => 0x200,
            EthercatRegister::ReceiveError => 0x300,
            EthercatRegister::FatalReceiveError => 0x308,
            EthercatRegister::EthercatProtocolUpdateErrorCount => 0x30C,
            EthercatRegister::ProcessErrorCount => 0x30D,
            EthercatRegister::ProcessErrorCode => 0x30E,
            EthercatRegister::LinkLayerCount => 0x310,
            EthercatRegister::WatchdogCount => 0x442,
            EthercatRegister::EepromConfig => 0x500,
            EthercatRegister::EepromControlStat => 0x502,
            EthercatRegister::EepromAddress => 0x504,
            EthercatRegister::EepromData => 0x508,
            EthercatRegister::FieldbusMemoryManagementUnit0 => 0x600,
            EthercatRegister::FieldbusMemoryManagementUnit1 => 0x610,
            EthercatRegister::FieldbusMemoryManagementUnit2 => 0x620,
            EthercatRegister::FieldbusMemoryManagementUnit3 => 0x630,
            EthercatRegister::SyncManager0 => 0x800,
            EthercatRegister::SyncManager1 => 0x808,
            EthercatRegister::SyncManager2 => 0x810,
            EthercatRegister::SyncManager3 => 0x818,
            EthercatRegister::SyncManager0Status => 0x805,
            EthercatRegister::SyncManager1Status => 0x808 + 5,
            EthercatRegister::SyncManager1Act => 0x808 + 6,
            EthercatRegister::SyncManager1Control => 0x808 + 7,
            EthercatRegister::DistributedClockTime0 => 0x900,
            EthercatRegister::DistributedClockTime1 => 0x904,
            EthercatRegister::DistributedClockTime2 => 0x908,
            EthercatRegister::DistributedClockTime3 => 0x90C,
            EthercatRegister::DistributedClockSystemTime => 0x910,
            EthercatRegister::DistributedClockStartOfFrame => 0x918,
            EthercatRegister::DistributedClockSystemOffset => 0x920,
            EthercatRegister::DistributedClockSystemDelay => 0x928,
            EthercatRegister::DistributedClockSystemDifference => 0x92C,
            EthercatRegister::DistributedClockSpeedCount => 0x930,
            EthercatRegister::DistributedClockTimeFilter => 0x934,
            EthercatRegister::DistributedClockControlUnit => 0x980,
            EthercatRegister::DistributedClockSynchronizationActive => 0x981,
            EthercatRegister::DistributedClockStart0 => 0x990,
            EthercatRegister::DistributedClockCycle0 => 0x9A0,
            EthercatRegister::DistributedClockCycle1 => 0x9A4,
        }
    }
}

/// Service data object sync manager communication type
pub const SDO_SMCOMMTYPE: u16 = 0x1C00;

/// Service Data Object - Process Data Object assignment
pub const SDO_PDO_ASSIGNMENT: u16 = 0x1C10;

/// Service Data Object - received Process Data Object assignment
pub const SDO_RX_PDO_ASSIGNMENT: u16 = 0x1C12;

/// Service Data Object - Send Process Data Object assignment
pub const SDO_TX_PDO_ASSIGNMENT: u16 = 0x1C13;

/// Ethercat packet type
pub const ETH_P_ECAT: u16 = 0x88A4;

#[derive(Debug, Clone, Copy)]
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

#[derive(Debug, Clone, Copy)]
#[repr(i32)]
pub enum AbortError {
    Abort(i32),
    ErrorCode(u16),
    PacketError(PacketError),
    EmergencyError {
        error_code: u16,
        error_register: u8,
        byte1: u8,
        word1: u16,
        word2: u16,
    },
    TooManyMasterBufferEntries = 0x0F00_0000,
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

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
/// A struct representing a numeric value in EtherCAT format.
/// This struct makes it harder or even impossible to perform some operations on values
/// converted to EtherCAT.
pub struct Ethercat<Int: PrimInt>(Int);

impl<Int: PrimInt> Ethercat<Int> {
    pub const fn from_raw(value: Int) -> Self {
        Self(value)
    }

    pub const fn into_inner(self) -> Int {
        self.0
    }

    pub fn is_zero(self) -> bool {
        self.0.is_zero()
    }

    pub fn to_host(self) -> Int {
        if cfg!(target_endian = "big") {
            self.0.to_be()
        } else {
            self.0
        }
    }

    pub fn from_host(value: Int) -> Self {
        Self(value.to_le())
    }
}

macro_rules! ethercat_bytes {
    ($data_type: ident) => {
        impl Ethercat<$data_type> {
            pub const fn to_bytes(self) -> [u8; size_of::<$data_type>()] {
                self.0.to_ne_bytes()
            }

            pub const fn from_bytes(bytes: [u8; size_of::<$data_type>()]) -> Self {
                Self::from_raw($data_type::from_ne_bytes(bytes))
            }
        }
    };
}

ethercat_bytes!(u16);
ethercat_bytes!(u32);
ethercat_bytes!(i32);
ethercat_bytes!(i64);
ethercat_bytes!(u64);
