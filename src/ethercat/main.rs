//! Main EtherCAT functions.
//!
//! Initializartion, state set and read, mailbox primitives, EEPROM primitives,
//! SII reading and processdata exchange.
//!
//! Defines ec_slave[]. All slave information is put in this structure.
//! Needed for most user interaction with slaves.

use std::{
    array,
    io::{self, Read, Write},
    str::Utf8Error,
    thread,
    time::{Duration, SystemTime},
};

use heapless::String as HeaplessString;
use oshw::nicdrv::Port;

use super::{
    base::{aprd, brd, bwr, fpwr, fpwrw},
    config::ConfigError,
    dc::{config_dc, DistributedClock},
    eoe::EthernetOverEthercat,
    r#type::{
        AbortError, ErrorType, Ethercat, EthercatHeader, EthercatState, ReadCommand, SiiCategory,
        SiiGeneralItem, WriteCommand, DEFAULT_RETRIES, EEPROM_STATE_MACHINE_BUSY,
        ETHERCAT_COMMAND_OFFET, MAX_BUFFER_COUNT, TIMEOUT_RETURN,
    },
    ReadFrom, WriteTo,
};
use crate::{
    ethercat::{
        base::{add_datagram, apwr, fprd, setup_datagram},
        eoe::{header_frame_type_get, EoEFrameType},
        r#type::{
            high_byte, high_word, low_word, BufferState, EepromCommand, EthercatRegister,
            EthernetHeader, MailboxType, BUFFER_SIZE, EEPROM_STATE_MACHINE_ERROR_MASK,
            EEPROM_STATE_MACHINE_ERROR_NACK, EEPROM_STATE_MACHINE_READ64,
            ETHERCAT_WORK_COUNTER_SIZE, MAX_EEPROM_BUFFER_SIZE, SII_START, TIMEOUT_EEPROM,
            TIMEOUT_RETURN3,
        },
    },
    osal::OsalTimer,
    oshw::{
        self,
        nicdrv::{NicdrvError, RedundancyMode, SECONDARY_MAC},
        Network,
    },
};

/// Max. entries in EtherCAT error list
pub const MAX_ERROR_LIST_ENTRIES: usize = 64;

/// Max length of readable name in slavelist and Object Description List
pub const MAX_NAME_LENGTH: u16 = 40;

/// Maximum number of slaves in array
pub const MAX_SLAVES: usize = 200;

/// Maximum number of groups
pub const MAX_GROUPS: usize = 2;

/// Maximum number of IO segments per group
pub const MAX_IO_SEGMENTS: usize = 64;

/// Max mailbox size
pub const MAX_MAILBOX_SIZE: usize = 1486;

/// Max EEProm Process Data Object entries
pub const MAX_EE_PDO: usize = 0x200;

/// Max Sync Manager used
pub const MAX_SM: u8 = 8;

/// Max Fieldbus Memory Management Units used
pub const MAX_FMMU: usize = 4;

/// Max adapter name length
pub const MAX_ADAPTER_NAME_LENGTH: usize = 128;

/// Maximum number of concurrent threads in mapping
pub const MAX_MAPT: usize = 1;

/// Delay in microseconds for eeprom ready loop
const LOCAL_DELAY: Duration = Duration::from_micros(200);

const MAX_FIXED_POINTER_READ_MULTI: usize = 64;

#[derive(Debug)]
pub enum MainError {
    Io(io::Error),
    Nicdrv(NicdrvError),
    Utf8(Utf8Error),
    InvalidMailboxType(u8),
    InvalidCOEMailboxType(u8),
    InvalidEthercatState(u8),
    SlaveIndexOutOfBounds(u16),
    InvalidEepromAddress(u16),
    SiiCategoryNotFound(SiiCategory),
    InvalidSyncManagerType(u8),
    MailboxFull,
    NoMailboxFound,
    SyncManagerIndexOutOfBounds,
    MissedTooManyAcks,
    EepromBusy,
    NoIOFound,
    NoFrame,
}

impl From<io::Error> for MainError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

impl From<NicdrvError> for MainError {
    fn from(value: NicdrvError) -> Self {
        Self::Nicdrv(value)
    }
}

impl From<Utf8Error> for MainError {
    fn from(value: Utf8Error) -> Self {
        Self::Utf8(value)
    }
}

/// EtherCAT Adapter
pub struct Adapter {
    name: String,
    desc: String,
}

/// Create list of available network adapters
///
/// # Returns
/// List of available network adapters
impl Adapter {
    /// Create list over available network adapters.
    ///
    /// # Panics
    /// Panics if the name of an interface is too long
    ///
    /// # Returns
    /// First element in linked list of adapters
    pub fn find_adapters() -> Vec<Self> {
        use crate::safe_c::if_nameindex;
        // Iterate all devices and create a local copy holding the name and description
        if_nameindex()
            .take_while(|id| id.index != 0)
            .map(|id| {
                // Fetch description and name, in Linux they are the same.
                let desc = id.name.clone();
                Self {
                    name: id.name,
                    desc,
                }
            })
            .collect()
    }
}

/// Fieldbus Memory Management Unit
#[derive(Debug, Default, Clone, Copy)]
pub struct Fmmu {
    log_start: Ethercat<u32>,
    log_length: Ethercat<u16>,
    log_start_bit: u8,
    log_end_bit: u8,
    physical_start: Ethercat<u16>,
    physical_start_bit: u8,
    fmmu_type: u8,
    fmmu_active: bool,

    /// 1-byte
    unused: (),

    /// 2-bytes
    unused2: (),
}

impl<W: Write> WriteTo<W> for Fmmu {
    fn write_to(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&self.log_start.to_bytes())?;
        writer.write_all(&self.log_length.to_bytes())?;
        writer.write_all(&[self.log_start_bit, self.log_end_bit])?;
        writer.write_all(&self.physical_start.to_bytes())?;
        writer.write_all(&[
            self.physical_start_bit,
            self.fmmu_type,
            u8::from(self.fmmu_active),
        ])?;
        writer.write_all(&[0; 3])?;
        Ok(())
    }
}

impl<R: Read> ReadFrom<R> for Fmmu {
    type Err = MainError;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let log_start = Ethercat::<u32>::from_bytes(<[u8; 4]>::read_from(reader)?);
        let log_length = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let log_start_bit = u8::read_from(reader)?;
        let log_end_bit = u8::read_from(reader)?;
        let physical_start = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let physical_start_bit = u8::read_from(reader)?;
        let fmmu_type = u8::read_from(reader)?;
        let fmmu_active = u8::read_from(reader)? != 0;
        let _unused = u8::read_from(reader)?;
        let _unused2 = u8::read_from(reader)?;
        Ok(Self {
            log_start,
            log_length,
            log_start_bit,
            log_end_bit,
            physical_start,
            physical_start_bit,
            fmmu_type,
            fmmu_active,
            unused: (),
            unused2: (),
        })
    }
}

impl Fmmu {
    pub const fn size() -> usize {
        size_of::<u32>() + 2 * size_of::<u16>() + 5 * size_of::<u8>() + 3
    }

    /// # Errors
    /// Returns an error if the Fmmu couldn't be converted to bytes
    #[expect(
        clippy::missing_panics_doc,
        reason = "Won't panic if Self::size is written correctly"
    )]
    pub fn bytes(&self) -> [u8; Self::size()] {
        let mut result = [0; Self::size()];
        self.write_to(&mut result.as_mut_slice()).unwrap();
        result
    }

    pub const fn log_start(&self) -> Ethercat<u32> {
        self.log_start
    }

    pub fn log_start_mut(&mut self) -> &mut Ethercat<u32> {
        &mut self.log_start
    }

    pub const fn log_length(&self) -> Ethercat<u16> {
        self.log_length
    }

    pub fn log_length_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.log_length
    }

    pub const fn log_start_bit(&self) -> u8 {
        self.log_start_bit
    }

    pub fn log_start_bit_mut(&mut self) -> &mut u8 {
        &mut self.log_start_bit
    }

    pub fn log_end_bit_mut(&mut self) -> &mut u8 {
        &mut self.log_end_bit
    }

    pub fn physical_start_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.physical_start
    }

    pub fn physical_start_bit_mut(&mut self) -> &mut u8 {
        &mut self.physical_start_bit
    }

    pub fn fmmu_type_mut(&mut self) -> &mut u8 {
        &mut self.fmmu_type
    }

    pub fn fmmu_active_mut(&mut self) -> &mut bool {
        &mut self.fmmu_active
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SyncManager {
    start_address: Ethercat<u16>,
    sm_length: Ethercat<u16>,
    sm_flags: Ethercat<u32>,
}

impl SyncManager {
    pub const fn size() -> usize {
        size_of::<u16>() * 2 + size_of::<u32>()
    }

    /// # Errors
    /// Returns an error if the SyncManager couldn't be converted to bytes
    #[expect(
        clippy::missing_panics_doc,
        reason = "Won't panic if Self::size is written correctly"
    )]
    pub fn bytes(&self) -> [u8; Self::size()] {
        let mut result = [0; Self::size()];
        self.write_to(&mut result.as_mut_slice()).unwrap();
        result
    }

    pub const fn start_address(&self) -> Ethercat<u16> {
        self.start_address
    }

    pub fn start_address_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.start_address
    }

    pub fn sm_flags_mut(&mut self) -> &mut Ethercat<u32> {
        &mut self.sm_flags
    }

    pub const fn sm_flags(&self) -> Ethercat<u32> {
        self.sm_flags
    }

    pub fn sm_length_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.sm_length
    }

    pub const fn sm_length(&self) -> Ethercat<u16> {
        self.sm_length
    }
}

impl<R: Read> ReadFrom<R> for SyncManager {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> io::Result<Self> {
        let start_address = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let sm_length = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let sm_flags = Ethercat::<u32>::from_bytes(<[u8; 4]>::read_from(reader)?);
        Ok(Self {
            start_address,
            sm_length,
            sm_flags,
        })
    }
}

impl<W: Write> WriteTo<W> for SyncManager {
    fn write_to(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&self.start_address.to_bytes())?;
        writer.write_all(&self.sm_length.to_bytes())?;
        writer.write_all(&self.sm_flags.to_bytes())
    }
}

struct StateStatus {
    state: u16,
    _unused: u16,
    al_status_code: u16,
}

pub enum MailboxProtocol {
    AdsOverEthercat = 1,
    EthernetOverEthercat,
    CanopenOverEthercat = 4,
    FileOverEthercat = 8,
    ServoOverEthercat = 0x10,
    VendorOverEthercat = 0x20,
}

impl From<MailboxProtocol> for u8 {
    fn from(value: MailboxProtocol) -> Self {
        value as Self
    }
}

impl From<MailboxProtocol> for u16 {
    fn from(value: MailboxProtocol) -> Self {
        value as Self
    }
}

/// CANopen over Ethercat Device Profile
pub enum Coedet {
    ServiceDataObject = 1,
    ServiceDataObjectInfo = 2,
    ProcessDataObjectAssign = 4,
    ProcessDataObjectConfig = 8,
    Upload = 0x10,

    /// Service data object communication over EtherCAT
    Sdoca = 0x20,
}

impl From<Coedet> for u8 {
    fn from(value: Coedet) -> Self {
        value as Self
    }
}

pub const SYNC_MANAGER_ENABLE_MASK: u32 = 0xFFFE_FFFF;

/// Sync manager type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SyncManagerType {
    #[default]
    Unused,
    MailboxWrite,
    MailboxRead,
    Outputs,
    Inputs,
}

impl From<SyncManagerType> for u8 {
    fn from(value: SyncManagerType) -> Self {
        match value {
            SyncManagerType::Unused => 0,
            SyncManagerType::MailboxWrite => 1,
            SyncManagerType::MailboxRead => 2,
            SyncManagerType::Outputs => 3,
            SyncManagerType::Inputs => 4,
        }
    }
}

impl TryFrom<u8> for SyncManagerType {
    type Error = MainError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Unused),
            1 => Ok(Self::MailboxWrite),
            2 => Ok(Self::MailboxRead),
            3 => Ok(Self::Outputs),
            4 => Ok(Self::Inputs),
            _ => Err(MainError::InvalidSyncManagerType(value)),
        }
    }
}

/// Amount of data per read from EEProm
#[derive(Debug, PartialEq, Eq)]
pub enum EepReadSize {
    Bytes4,
    Bytes8,
}

pub enum SlaveResponding {
    SlaveResponds,
    SlaveLost,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EepromControl {
    Master,
    Pdi,
}

pub struct SlaveEeprom {
    manufacturer: u32,
    id: u32,
    revision: u32,

    /// The amount of data per read from EEPROM
    read_size: EepReadSize,

    control: EepromControl,
}

impl SlaveEeprom {
    pub const fn new(
        manufacturer: u32,
        id: u32,
        revision: u32,
        read_size: EepReadSize,
        control: EepromControl,
    ) -> Self {
        Self {
            manufacturer,
            id,
            revision,
            read_size,
            control,
        }
    }

    pub const fn manufacturer(&self) -> u32 {
        self.manufacturer
    }

    pub fn manufacturer_mut(&mut self) -> &mut u32 {
        &mut self.manufacturer
    }

    pub const fn id(&self) -> u32 {
        self.id
    }

    pub fn id_mut(&mut self) -> &mut u32 {
        &mut self.id
    }

    pub const fn revision(&self) -> u32 {
        self.revision
    }

    pub fn revision_mut(&mut self) -> &mut u32 {
        &mut self.revision
    }

    pub fn read_size_mut(&mut self) -> &mut EepReadSize {
        &mut self.read_size
    }

    pub const fn control(&self) -> EepromControl {
        self.control
    }

    pub fn control_mut(&mut self) -> &mut EepromControl {
        &mut self.control
    }
}

#[derive(Debug, Default)]
pub struct SlaveMailbox {
    /// Length of write mailbox in bytes, 0 if no mailbox
    length: u16,

    write_offset: u16,

    // Length of read mailbox in bytes
    read_length: u16,

    read_offset: u16,

    /// Supported mailbox protocols
    protocols: u16,

    /// Counter value of mailbox link layer protocol 1..7
    count: u8,
}

impl SlaveMailbox {
    pub const fn length(&self) -> u16 {
        self.length
    }

    pub fn length_mut(&mut self) -> &mut u16 {
        &mut self.length
    }

    pub const fn write_offset(&self) -> u16 {
        self.write_offset
    }

    pub fn write_offset_mut(&mut self) -> &mut u16 {
        &mut self.write_offset
    }

    pub fn read_length_mut(&mut self) -> &mut u16 {
        &mut self.read_length
    }

    pub const fn read_length(&self) -> u16 {
        self.read_length
    }

    pub fn read_offset_mut(&mut self) -> &mut u16 {
        &mut self.read_offset
    }

    pub const fn read_offset(&self) -> u16 {
        self.read_offset
    }

    pub fn protocols_mut(&mut self) -> &mut u16 {
        &mut self.protocols
    }

    pub const fn protocols(&self) -> u16 {
        self.protocols
    }

    pub const fn mailbox_count(&self) -> u8 {
        self.count
    }

    /// Sets and returns index of next mailbox counter value.
    /// Used for Mailbox Link Layer.
    ///
    /// # Returns next mailbox counter value
    pub fn next_count(&mut self) -> u8 {
        // Wraps around to 1, not 0 for a range of 1..=7
        self.count = self.count % 7 + 1;
        self.count
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ProtocolDetails {
    canopen_over_ethercat: u8,
    file_over_ethercat: u8,
    ethernet_over_ethercat: u8,
    servo_over_ethercat: u8,
}

impl ProtocolDetails {
    pub const fn canopen_over_ethercat(&self) -> u8 {
        self.canopen_over_ethercat
    }

    pub fn canopen_over_ethercat_mut(&mut self) -> &mut u8 {
        &mut self.canopen_over_ethercat
    }

    pub const fn file_over_ethercat(&self) -> u8 {
        self.file_over_ethercat
    }

    pub fn file_over_ethercat_mut(&mut self) -> &mut u8 {
        &mut self.file_over_ethercat
    }

    pub const fn ethernet_over_ethercat(&self) -> u8 {
        self.ethernet_over_ethercat
    }

    pub fn ethernet_over_ethercat_mut(&mut self) -> &mut u8 {
        &mut self.ethernet_over_ethercat
    }

    pub const fn servo_over_ethercat(&self) -> u8 {
        self.servo_over_ethercat
    }

    pub fn servo_over_ethercat_mut(&mut self) -> &mut u8 {
        &mut self.servo_over_ethercat
    }
}

/// Detected EtherCAT slave
pub struct Slave<'slave> {
    /// State of slave
    state: EthercatState,

    /// Application layer status code
    al_status_code: u16,

    /// Configured address
    config_address: u16,

    /// Alias address
    alias_address: u16,

    /// Manufacturer from EEprom
    eep: SlaveEeprom,

    interface_type: u16,
    output_bits: u16,

    /// Output bytes, if output_bits < 8, output_bytes = 0
    output_bytes: u16,

    /// IOmap buffer
    io_map: &'slave [u8],
    input_offset: u32,
    output_offset: u32,

    /// Startbit in first output byte
    output_startbit: u8,

    input_bits: u16,

    /// Input bytes, if input_bits < 8, output_bytes = 0
    input_bytes: u16,

    /// Startbit in IOmap buffer
    input_startbit: u8,

    sync_manager: [SyncManager; MAX_SM as usize],
    sync_manager_type: [SyncManagerType; MAX_SM as usize],

    /// Fieldbus Memory Management Units
    fmmu: [Fmmu; MAX_FMMU],

    /// Fieldbus Memory Management Unit 0 function
    fmmu0_function: u8,

    /// Fieldbus Memory Management Unit 1 function
    fmmu1_function: u8,

    /// Fieldbus Memory Management Unit 2 function
    fmmu2_function: u8,

    /// Fieldbus Memory Management Unit 3 function
    fmmu3_function: u8,

    mailbox: SlaveMailbox,

    /// Distributed clock if slave has DC capability
    distributed_clock: Option<DistributedClock>,

    /// Physical type: Ebus Ethernet combinations
    physical_type: u8,

    /// Topology: 1 to 3 links
    topology: u8,

    /// Active ports bitmap: ....3210, set if respective port is active
    active_ports: u8,

    /// Consumed ports bitmap: ...3210, used for internal delay measurement
    consumed_ports: u8,

    /// Slave number for parent, 0=master
    slave_number_for_parent: u8,

    /// Port number on parent this slave is connected to
    parent_port: u8,

    // 0 = master
    parent: u8,

    /// Port number on this slave the parent is connected to
    entry_port: u8,

    propagation_delay: Duration,

    /// Link to config table
    config_index: u16,

    /// Link to SII config
    sii_index: u16,

    protocol_details: ProtocolDetails,

    /// E-bus current
    ebus_current: u16,

    /// If > 0 block use of LRW in processdata
    block_logical_read_write: u8,

    group: u8,

    /// First unused Fieldbus Memory Management Unit
    fmmu_unused: u8,

    /// Whether the slave is responding, not used by the SOEM library
    is_lost: SlaveResponding,

    /// Registered configuration function PO -> SO (DEPRECATED)
    po2_so_config: Option<fn(slave: u16) -> i32>,

    /// Registered configuration function PO->SO
    po2_so_configx: Option<fn(context: &mut Context, slave: u16) -> i32>,

    name: HeaplessString<{ MAX_NAME_LENGTH as usize + 1 }>,
}

impl<'slave> Slave<'slave> {
    pub const fn propagation_delay(&self) -> Duration {
        self.propagation_delay
    }

    pub fn propagation_delay_mut(&mut self) -> &mut Duration {
        &mut self.propagation_delay
    }

    pub const fn active_ports(&self) -> u8 {
        self.active_ports
    }

    pub const fn consumed_ports(&self) -> u8 {
        self.consumed_ports
    }

    pub fn consumed_ports_mut(&mut self) -> &mut u8 {
        &mut self.consumed_ports
    }

    pub fn entry_port_mut(&mut self) -> &mut u8 {
        &mut self.entry_port
    }

    pub const fn entry_port(&self) -> u8 {
        self.entry_port
    }

    pub const fn distributed_clock(&self) -> Option<&DistributedClock> {
        self.distributed_clock.as_ref()
    }

    pub fn distributed_clock_mut(&mut self) -> Option<&mut DistributedClock> {
        self.distributed_clock.as_mut()
    }

    pub fn set_distributed_clock(&mut self, clock: DistributedClock) {
        self.distributed_clock = Some(clock);
    }

    pub fn remove_distributed_clock(&mut self) {
        self.distributed_clock = None;
    }

    pub const fn alias_address(&self) -> u16 {
        self.alias_address
    }

    pub fn input_startbit_mut(&mut self) -> &mut u8 {
        &mut self.input_startbit
    }

    pub const fn input_startbit(&self) -> u8 {
        self.input_startbit
    }

    pub fn input_bytes_mut(&mut self) -> &mut u16 {
        &mut self.input_bytes
    }

    pub const fn output_bytes(&self) -> u16 {
        self.output_bytes
    }

    pub fn output_bytes_mut(&mut self) -> &mut u16 {
        &mut self.output_bytes
    }

    pub const fn state(&self) -> EthercatState {
        self.state
    }

    pub const fn eeprom(&self) -> &SlaveEeprom {
        &self.eep
    }

    pub fn eeprom_mut(&mut self) -> &mut SlaveEeprom {
        &mut self.eep
    }

    pub const fn mailbox(&self) -> &SlaveMailbox {
        &self.mailbox
    }

    pub fn mailbox_mut(&mut self) -> &mut SlaveMailbox {
        &mut self.mailbox
    }

    pub fn sync_manager_type_mut(&mut self) -> &mut [SyncManagerType; MAX_SM as usize] {
        &mut self.sync_manager_type
    }

    pub fn get_sync_manager_type_mut(&mut self, sync_manager_index: u8) -> &mut SyncManagerType {
        &mut self.sync_manager_type[usize::from(sync_manager_index)]
    }

    pub fn get_sync_manager_type(&self, sync_manager_index: u8) -> SyncManagerType {
        self.sync_manager_type[usize::from(sync_manager_index)]
    }

    pub fn get_sync_manager(&self, sync_manager_index: u8) -> &SyncManager {
        &self.sync_manager[usize::from(sync_manager_index)]
    }

    pub fn get_sync_manager_mut(&mut self, sync_manager_index: u8) -> &mut SyncManager {
        &mut self.sync_manager[usize::from(sync_manager_index)]
    }

    pub const fn protocol_details(&self) -> ProtocolDetails {
        self.protocol_details
    }

    pub fn protocol_details_mut(&mut self) -> &mut ProtocolDetails {
        &mut self.protocol_details
    }

    pub const fn block_logical_read_write(&self) -> u8 {
        self.block_logical_read_write
    }

    pub fn block_logical_read_write_mut(&mut self) -> &mut u8 {
        &mut self.block_logical_read_write
    }

    pub const fn ebus_current(&self) -> u16 {
        self.ebus_current
    }

    pub fn ebus_current_mut(&mut self) -> &mut u16 {
        &mut self.ebus_current
    }

    pub const fn name(&self) -> &HeaplessString<{ MAX_NAME_LENGTH as usize + 1 }> {
        &self.name
    }

    pub fn name_mut(&mut self) -> &mut HeaplessString<{ MAX_NAME_LENGTH as usize + 1 }> {
        &mut self.name
    }

    pub const fn fmmu0_function(&self) -> u8 {
        self.fmmu0_function
    }

    pub fn fmmu0_function_mut(&mut self) -> &mut u8 {
        &mut self.fmmu0_function
    }

    pub const fn fmmu1_function(&self) -> u8 {
        self.fmmu1_function
    }

    pub fn fmmu1_function_mut(&mut self) -> &mut u8 {
        &mut self.fmmu1_function
    }

    pub const fn fmmu2_function(&self) -> u8 {
        self.fmmu2_function
    }

    pub fn fmmu2_function_mut(&mut self) -> &mut u8 {
        &mut self.fmmu2_function
    }

    pub const fn fmmu3_function(&self) -> u8 {
        self.fmmu3_function
    }

    pub fn fmmu3_function_mut(&mut self) -> &mut u8 {
        &mut self.fmmu3_function
    }

    pub fn interface_type_mut(&mut self) -> &mut u16 {
        &mut self.interface_type
    }

    pub fn config_address_mut(&mut self) -> &mut u16 {
        &mut self.config_address
    }

    pub const fn config_address(&self) -> u16 {
        self.config_address
    }

    pub fn alias_address_mut(&mut self) -> &mut u16 {
        &mut self.alias_address
    }

    pub fn physical_type_mut(&mut self) -> &mut u8 {
        &mut self.physical_type
    }

    pub const fn topology(&self) -> u8 {
        self.topology
    }

    pub fn topology_mut(&mut self) -> &mut u8 {
        &mut self.topology
    }

    pub fn active_ports_mut(&mut self) -> &mut u8 {
        &mut self.active_ports
    }

    pub const fn parent_port(&self) -> u8 {
        self.parent_port
    }

    pub fn parent_port_mut(&mut self) -> &mut u8 {
        &mut self.parent_port
    }

    pub const fn parent(&self) -> u8 {
        self.parent
    }

    pub fn parent_mut(&mut self) -> &mut u8 {
        &mut self.parent
    }

    pub const fn output_bits(&self) -> u16 {
        self.output_bits
    }

    pub fn output_bits_mut(&mut self) -> &mut u16 {
        &mut self.output_bits
    }

    pub const fn input_bits(&self) -> u16 {
        self.input_bits
    }

    pub fn input_bits_mut(&mut self) -> &mut u16 {
        &mut self.input_bits
    }

    pub const fn po2_so_config(&self) -> Option<fn(u16) -> i32> {
        self.po2_so_config
    }

    pub const fn po2_so_configx(&self) -> Option<fn(context: &mut Context, u16) -> i32> {
        self.po2_so_configx
    }

    pub const fn config_index(&self) -> u16 {
        self.config_index
    }

    pub const fn group(&self) -> u8 {
        self.group
    }

    pub const fn fmmu(&self) -> &[Fmmu; MAX_FMMU] {
        &self.fmmu
    }

    pub fn get_fmmu(&self, index: u8) -> Fmmu {
        self.fmmu[usize::from(index)]
    }

    pub fn get_fmmu_mut(&mut self, index: u8) -> &mut Fmmu {
        &mut self.fmmu[usize::from(index)]
    }

    pub const fn fmmu_unused(&self) -> u8 {
        self.fmmu_unused
    }

    pub fn fmmu_unused_mut(&mut self) -> &mut u8 {
        &mut self.fmmu_unused
    }

    pub const fn input_bytes(&self) -> u16 {
        self.input_bytes
    }

    pub fn input(&self) -> Option<&[u8]> {
        let start = self.output_offset as usize
            + usize::from(self.output_bytes)
            + self.input_offset as usize;
        self.io_map
            .get(start..start + usize::from(self.input_bytes))
    }

    pub fn input_offset_mut(&mut self) -> &mut u32 {
        &mut self.input_offset
    }

    pub fn outputs(&self) -> Option<&'slave [u8]> {
        let start = self.output_offset as usize;
        self.io_map
            .get(start..start + usize::from(self.output_bytes))
    }

    pub fn output_offset_mut(&mut self) -> &mut u32 {
        &mut self.output_offset
    }

    pub const fn output_startbit(&self) -> u8 {
        self.output_startbit
    }

    pub fn output_startbit_mut(&mut self) -> &mut u8 {
        &mut self.output_startbit
    }

    pub fn io_map_mut(&mut self) -> &mut &'slave [u8] {
        &mut self.io_map
    }
}

impl Default for Slave<'_> {
    fn default() -> Self {
        const DEFAULT_SLICE: &[u8] = &[];
        Self {
            state: EthercatState::None,
            al_status_code: 0,
            config_address: 0,
            alias_address: 0,
            eep: SlaveEeprom::new(0, 0, 0, EepReadSize::Bytes4, EepromControl::Pdi),
            interface_type: 0,
            output_bits: 0,
            output_bytes: 0,
            io_map: DEFAULT_SLICE,
            input_offset: 0,
            output_offset: 0,
            output_startbit: 0,
            input_bits: 0,
            input_bytes: 0,
            input_startbit: 0,
            sync_manager: [SyncManager::default(); MAX_SM as usize],
            sync_manager_type: [SyncManagerType::default(); MAX_SM as usize],
            fmmu: [Fmmu::default(); MAX_FMMU],
            fmmu0_function: 0,
            fmmu1_function: 0,
            fmmu2_function: 0,
            fmmu3_function: 0,
            mailbox: SlaveMailbox::default(),
            distributed_clock: None,
            physical_type: 0,
            topology: 0,
            active_ports: 0,
            consumed_ports: 0,
            slave_number_for_parent: 0,
            parent_port: 0,
            parent: 0,
            entry_port: 0,
            propagation_delay: Duration::default(),
            config_index: 0,
            sii_index: 0,
            protocol_details: ProtocolDetails::default(),
            ebus_current: 0,
            block_logical_read_write: 0,
            group: 0,
            fmmu_unused: 0,
            is_lost: SlaveResponding::SlaveLost,
            po2_so_config: None,
            po2_so_configx: None,
            name: HeaplessString::new(),
        }
    }
}

///EtherCAT slave group
#[derive(Debug)]
pub struct SlaveGroup<'io_map> {
    /// Logical start address for this group
    logical_start_address: u32,

    /// Output bytes, 0 if output bits < 0
    output_bytes: u32,

    /// IOmap buffer
    io_map: &'io_map [u8],

    /// Input bytes, 0 if input bits < 8
    input_bytes: u32,

    /// Next DC slave
    dc_next: Option<u16>,

    /// E-bus current
    ebus_current: u16,

    /// If >0 block use of logical read write in process data
    block_logical_read_write: u8,

    /// Number of used IO segments
    used_segment_count: u16,

    first_input_segment: u16,

    /// Offset in input segment
    input_offset: u16,

    /// Expected workcounter outputs
    work_counter_outputs: u16,

    /// Expected workcounter inputs
    work_counter_inputs: u16,

    check_slaves_states: bool,

    /// OI segmentation list. Datagrams must not break SM in two.
    io_segments: [u32; MAX_IO_SEGMENTS],
}

impl<'io_map> SlaveGroup<'io_map> {
    pub fn with_logical_start_address(logical_start_address: u32) -> Self {
        Self {
            logical_start_address,
            ..Default::default()
        }
    }

    pub const fn logical_start_address(&self) -> u32 {
        self.logical_start_address
    }

    pub const fn output_bytes(&self) -> u32 {
        self.output_bytes
    }

    pub fn output_bytes_mut(&mut self) -> &mut u32 {
        &mut self.output_bytes
    }

    pub fn io_map_mut(&mut self) -> &mut &'io_map [u8] {
        &mut self.io_map
    }

    pub const fn io_map(&self) -> &'io_map [u8] {
        self.io_map
    }

    pub const fn input_bytes(&self) -> u32 {
        self.input_bytes
    }

    pub fn input_bytes_mut(&mut self) -> &mut u32 {
        &mut self.input_bytes
    }

    pub fn inputs(&mut self) -> &'io_map [u8] {
        &self.io_map[self.output_bytes as usize..]
    }

    pub fn outputs(&self) -> &'io_map [u8] {
        &self.io_map[..self.output_bytes as usize]
    }

    pub fn remove_distributed_clock(&mut self) {
        self.dc_next = None;
    }

    pub fn set_distributed_clock_next(&mut self, next: u16) {
        self.dc_next = Some(next);
    }

    pub const fn has_distributed_clock(&self) -> bool {
        self.dc_next.is_some()
    }

    pub fn work_counter_inputs_mut(&mut self) -> &mut u16 {
        &mut self.work_counter_inputs
    }

    pub fn ebus_current_mut(&mut self) -> &mut u16 {
        &mut self.ebus_current
    }

    pub fn block_logical_read_write_mut(&mut self) -> &mut u8 {
        &mut self.block_logical_read_write
    }

    pub const fn used_segment_count(&self) -> u16 {
        self.used_segment_count
    }

    pub fn used_segment_count_mut(&mut self) -> &mut u16 {
        &mut self.used_segment_count
    }

    pub fn first_input_segment_mut(&mut self) -> &mut u16 {
        &mut self.first_input_segment
    }

    pub fn input_offset_mut(&mut self) -> &mut u16 {
        &mut self.input_offset
    }

    pub fn work_counter_outputs_mut(&mut self) -> &mut u16 {
        &mut self.work_counter_outputs
    }

    pub fn get_io_segment_mut(&mut self, segment: u16) -> &mut u32 {
        &mut self.io_segments[usize::from(segment)]
    }
}

impl<'io_map> Default for SlaveGroup<'io_map> {
    fn default() -> Self {
        const STATIC_SLICE: &[u8] = &[];
        Self {
            logical_start_address: 0,
            output_bytes: 0,
            io_map: STATIC_SLICE,
            input_bytes: 0,
            dc_next: None,
            ebus_current: 0,
            block_logical_read_write: 0,
            used_segment_count: 0,
            first_input_segment: 0,
            input_offset: 0,
            work_counter_outputs: 0,
            work_counter_inputs: 0,
            check_slaves_states: false,
            io_segments: [0; MAX_IO_SEGMENTS],
        }
    }
}

/// Record for Ethercat EEPROM communications
struct Eeprom {
    command: EepromCommand,
    address: u16,
    d2: u16,
}

impl<W: Write> WriteTo<W> for Eeprom {
    /// # Errors
    /// Returns an error if the writer doesn't have enough capacity for the eeprom
    fn write_to(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&Ethercat::from_host(self.command as u16).to_bytes())?;
        writer.write_all(&Ethercat::from_host(self.address).to_bytes())?;
        writer.write_all(&Ethercat::from_host(self.d2).to_bytes())?;
        Ok(())
    }
}

/// Eeprom Fieldbus Memory Management Unit
#[derive(Debug, Default, Clone, Copy)]
pub struct EepromFmmu {
    start_position: u16,
    number_fmmu: u8,
    fmmu: [u8; 4],
}

impl EepromFmmu {
    pub const fn fmmu_count(&self) -> u8 {
        self.number_fmmu
    }

    pub const fn fmmu(&self) -> &[u8; 4] {
        &self.fmmu
    }

    /// Get FMMU data from SII FMMU section in slave EEPROM
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave number
    ///
    /// # Errors
    /// Returns an error if:
    /// - A byte couldn't be read from the EEPROM
    /// - Control couldn't be returned to the PDI
    ///
    /// # Returns
    /// FMMU struct from SII (maximum is 4 FMMU's)
    pub fn sii_fmmu(context: &mut Context, slave: u16) -> Result<Self, MainError> {
        let slave_usize = usize::from(slave);
        let eeprom_control = context.slavelist[slave_usize].eeprom().control();

        let mut number_fmmu = 0;
        let mut fmmu = [0; 4];
        let start_position = context
            .sii_find(slave, SiiCategory::FMMU)
            .unwrap_or_default();

        if start_position > 0 {
            let address = start_position;
            number_fmmu = context.sii_get_byte(slave, address)? * 2;
            fmmu[0] = context.sii_get_byte(slave, address + 2)?;
            fmmu[1] = context.sii_get_byte(slave, address + 3)?;
            if number_fmmu > 2 {
                fmmu[2] = context.sii_get_byte(slave, address + 4)?;
                fmmu[3] = context.sii_get_byte(slave, address + 5)?;
            }
        }

        // If eeprom was previously pdi, then restore
        if eeprom_control == EepromControl::Pdi {
            context.eeprom_to_pdi(slave)?;
        }
        Ok(Self {
            start_position,
            number_fmmu,
            fmmu,
        })
    }
}

#[derive(Debug, Clone, Default)]
pub struct EepromSyncManager {
    start_position: u16,
    sync_manager_count: u8,
    phase_start: u16,
    phase_length: u16,
    control_register: u8,

    /// Not used by SOEM
    slave_register: u8,

    activate: u8,

    /// Not used by SOEM
    process_data_interface_control: u8,
}

impl EepromSyncManager {
    pub const fn sync_manager_count(&self) -> u8 {
        self.sync_manager_count
    }

    pub const fn phase_start(&self) -> u16 {
        self.phase_start
    }

    pub const fn phase_length(&self) -> u16 {
        self.phase_length
    }

    pub const fn control_register(&self) -> u8 {
        self.control_register
    }

    pub const fn activate(&self) -> u8 {
        self.activate
    }
}

impl EepromSyncManager {
    /// Get SyncManaer from SII SyncManager section in slave EEPROM
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave number
    ///
    /// # Errors
    /// Returns an error if:
    /// - The SyncManager SII category couldn't be found
    /// - A byte couldn't be read from the eeprom
    /// - Eeprom couldn't be returned to the PDI
    ///
    /// # Returns
    /// First SyncManager struct from SII
    pub fn sii_sm(context: &mut Context, slave: u16) -> Result<Self, MainError> {
        let eeprom_control = context.slavelist[usize::from(slave)].eeprom().control();

        let start_position = context.sii_find(slave, SiiCategory::SM)?;
        let sync_manager = if start_position > 0 {
            let address = start_position;
            let w = u16::from(context.sii_get_byte(slave, address)?)
                + (u16::from(context.sii_get_byte(slave, address + 1)?) << 8);
            Self {
                sync_manager_count: (w / 4) as u8,
                phase_start: u16::from(context.sii_get_byte(slave, address + 2)?)
                    + (u16::from(context.sii_get_byte(slave, address + 3)?) << 8),
                phase_length: u16::from(context.sii_get_byte(slave, address + 4)?)
                    + (u16::from(context.sii_get_byte(slave, address + 5)?) << 8),
                control_register: context.sii_get_byte(slave, address + 6)?,
                start_position,
                slave_register: context.sii_get_byte(slave, address + 7)?,
                activate: context.sii_get_byte(slave, address + 8)?,
                process_data_interface_control: context.sii_get_byte(slave, address + 9)?,
            }
        } else {
            Self::default()
        };

        // If eeprom control was previously pdi, restore it
        if eeprom_control == EepromControl::Pdi {
            context.eeprom_to_pdi(slave)?;
        }
        Ok(sync_manager)
    }

    /// Get next SyncManager data from SII SyncManager section in slave EEPROM.
    ///
    /// # Parameters
    /// `self`: First sync manager struct from SII
    /// `context`: Context struct
    /// `slave`: Slave number
    /// `number`: Sync manager number
    ///
    /// # Errors
    /// Returns an error if:
    /// - A byte couldn't be read from the EEPROM
    /// - The requested sync manager index is too high
    /// - Control over the EEPROM couldn't be returned to the PDI
    ///
    /// # Returns
    /// Requested Eeprom SyncManager if available
    pub fn sii_sm_next(
        &self,
        context: &mut Context,
        slave: u16,
        index: u8,
    ) -> Result<Self, MainError> {
        let eeprom_control = context.slavelist[usize::from(slave)].eeprom().control();
        if index >= self.sync_manager_count {
            return Err(MainError::SyncManagerIndexOutOfBounds);
        }
        let address = self.start_position + 2 + u16::from(index) * 8;
        let sync_manager = Self {
            phase_start: u16::from(context.sii_get_byte(slave, address)?)
                + (u16::from(context.sii_get_byte(slave, address + 1)?) << 8),
            phase_length: u16::from(context.sii_get_byte(slave, address + 2)?)
                + (u16::from(context.sii_get_byte(slave, address + 3)?) << 8),
            control_register: context.sii_get_byte(slave, address + 4)?,
            slave_register: context.sii_get_byte(slave, address + 5)?,
            activate: context.sii_get_byte(slave, address + 6)?,
            process_data_interface_control: context.sii_get_byte(slave, address + 7)?,
            start_position: self.start_position,
            sync_manager_count: self.sync_manager_count,
        };

        // If eeprom control was previously pdi then restore
        if eeprom_control == EepromControl::Pdi {
            context.eeprom_to_pdi(slave)?;
        }
        Ok(sync_manager)
    }
}

/// Eeprom Process Data Object
#[derive(Debug)]
pub struct EepromPdo {
    start_position: u16,
    length: u16,
    pdo_count: u16,
    index: [u16; MAX_EE_PDO],
    sync_manager: [u16; MAX_EE_PDO],
    bit_size: [u16; MAX_EE_PDO],
    sync_manager_bit_size: [u16; MAX_SM as usize],
}

impl Default for EepromPdo {
    fn default() -> Self {
        Self {
            start_position: 0,
            length: 0,
            pdo_count: 0,
            index: [0; MAX_EE_PDO],
            sync_manager: [0; MAX_EE_PDO],
            bit_size: [0; MAX_EE_PDO],
            sync_manager_bit_size: [0; MAX_SM as usize],
        }
    }
}

impl EepromPdo {
    pub fn get_sync_manager_bit_size(&self, sync_manager_index: u8) -> u16 {
        self.sync_manager_bit_size[usize::from(sync_manager_index)]
    }

    /// Get PDO data from SII section in slave EEPROM.
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave number
    /// `transmitting`: false=RXPDO, true=TXPDO
    ///
    /// # Errors
    /// Returns an error if:
    /// - The PDO send or PDO receive EEPROM category couldn't be found
    /// - A byte from the eeprom couldn't be read
    /// - EEPROM control couldn't be returned to PDI
    ///
    /// # Returns
    /// Mapping size in bits of PDO and the Pdo struct from SII
    pub fn sii_pdo(
        context: &mut Context,
        slave: u16,
        transmitting: bool,
    ) -> Result<(u32, Self), MainError> {
        let eeprom_control = context.slavelist[usize::from(slave)].eeprom().control();
        let mut size = 0;
        let mut pdo_count = 0;
        let mut length = 0;
        let mut index = [0; MAX_EE_PDO];
        let mut sync_manager_bit_size = [0; MAX_SM as usize];
        let mut bit_size = [0; MAX_EE_PDO];
        let mut sync_manager = [0; MAX_EE_PDO];
        let pdo = if let Ok(start_position) = context.sii_find(
            slave,
            if transmitting {
                SiiCategory::PDOSend
            } else {
                SiiCategory::PDOReceive
            },
        ) {
            let mut address = start_position;
            length = u16::from(context.sii_get_byte(slave, address)?)
                + (u16::from(context.sii_get_byte(slave, address + 1)?) << 8);
            let mut current = 1;
            address += 2;

            // Traverse through all PDOs
            loop {
                pdo_count += 1;
                index[usize::from(pdo_count)] = u16::from(context.sii_get_byte(slave, address)?)
                    + (u16::from(context.sii_get_byte(slave, address + 1)?) << 8);
                current += 1;
                let e = u16::from(context.sii_get_byte(slave, address + 2)?);
                sync_manager[usize::from(pdo_count)] =
                    u16::from(context.sii_get_byte(slave, address + 3)?);
                address += 8;
                current += 2;

                // Check whether the sync manager is active and in range
                if sync_manager[usize::from(pdo_count)] < u16::from(MAX_SM) {
                    // Read all entries defined in PDO
                    for _err in 1..=e {
                        current += 4;
                        address += 5;
                        bit_size[usize::from(pdo_count)] +=
                            u16::from(context.sii_get_byte(slave, address)?);
                        address += 3;
                    }
                    sync_manager_bit_size[usize::from(sync_manager[usize::from(pdo_count)])] +=
                        bit_size[usize::from(pdo_count)];
                    size += u32::from(bit_size[usize::from(pdo_count)]);
                    current += 1;
                } else {
                    // PDO deactivated, because SM is 0xFF or > MAXSM
                    current += 4 * e + 1;
                    address += 8 * e;
                }

                // Limit number of PDO entries in buffer
                if usize::from(pdo_count) >= MAX_EE_PDO - 1 {
                    current = length;
                }

                if current >= length {
                    break;
                }
            }
            Self {
                start_position,
                length,
                pdo_count,
                index,
                sync_manager,
                bit_size,
                sync_manager_bit_size,
            }
        } else {
            Self {
                start_position: 0,
                length,
                pdo_count,
                index,
                sync_manager,
                bit_size,
                sync_manager_bit_size,
            }
        };

        // If eeprom control wa previously pdi, then restore
        if eeprom_control == EepromControl::Pdi {
            context.eeprom_to_pdi(slave)?;
        }
        Ok((size, pdo))
    }
}

/// Mailbox error structure
struct MailboxError {
    mailbox_header: MailboxHeader,
    error_type: Ethercat<u16>,
    detail: Ethercat<u16>,
}

impl MailboxError {
    pub fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
        let mailbox_header = MailboxHeader::read_from(reader)?;
        let mut word = [0; 2];
        reader.read_exact(&mut word)?;
        let error_type = Ethercat::<u16>::from_bytes(word);
        reader.read_exact(&mut word)?;
        let detail = Ethercat::<u16>::from_bytes(word);
        Ok(Self {
            mailbox_header,
            error_type,
            detail,
        })
    }
}

/// Mailbox buffer array
#[derive(Debug)]
pub struct MailboxIn {
    data: [u8; MAX_MAILBOX_SIZE],
    length: usize,
}

impl Read for MailboxIn {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.length == 0 {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "Mailbox is empty",
            ));
        }
        buf.copy_from_slice(&self.data[..self.length]);
        let written = buf.len().min(self.length);
        self.length -= written;
        self.data.rotate_left(written);
        Ok(written)
    }
}

impl MailboxIn {
    /// Clear mailbox buffer (sets size to 0)
    ///
    /// # Parameter
    /// `self`: Mailbox to clear
    pub fn clear(&mut self) {
        self.length = 0;
    }

    /// Read OUT mailbox of slave.
    /// Supports mailbox link layer with repeat requests.
    ///
    /// # Parameters
    /// `self`: Mailbox data
    /// `context`: Context struct
    /// `slave`: Slave number
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - The slave doesn't have a mailbox
    /// - Data couldn't be received from the slave.
    /// - Received data couldn't be parsed into a `MailboxHeader`
    /// - Received data couldn't be parsed into a `MailboxError`
    /// - Received data couldn't be parsed into an `EmergencyRequest`
    /// - Received data couldn't be parsed into a `EthernetOverEthercat` struct
    ///
    /// # Returns
    /// Work counter (>0 if success)
    pub fn receive(
        &mut self,
        context: &mut Context,
        slave: u16,
        timeout: Duration,
    ) -> Result<u16, MainError> {
        let slave_usize = usize::from(slave);
        let mailbox_length = context.slavelist[slave_usize].mailbox().read_length();

        if !(0..=MAX_MAILBOX_SIZE).contains(&usize::from(mailbox_length)) {
            return Err(MainError::NoMailboxFound);
        }

        let config_address = context.slavelist[slave_usize].config_address;
        let timer = OsalTimer::new(timeout);

        // Wait for read mailbox available
        let (work_counter, sync_manager_status) = loop {
            let mut word = [0; 2];
            let work_counter = fprd(
                &mut context.port,
                config_address,
                EthercatRegister::SyncManager1Status.into(),
                &mut word,
                TIMEOUT_RETURN,
            )?;
            let sync_manager_status = Ethercat::<u16>::from_bytes(word).to_host();

            if sync_manager_status & 8 == 0 && timeout > LOCAL_DELAY {
                thread::sleep(LOCAL_DELAY);
            }
            if (work_counter == 0 && sync_manager_status & 8 == 0) || timer.is_expired() {
                break (work_counter, sync_manager_status);
            }
        };

        // If read mailbox is available
        if work_counter > 0 && sync_manager_status & 8 > 0 {
            let mailbox_read_offset = context.slavelist[slave_usize].mailbox().read_offset();
            let mailbox_header = MailboxHeader::read_from(&mut self.data.as_slice())?;

            loop {
                // Get mailbox
                let mut work_counter = fprd(
                    &mut context.port,
                    config_address,
                    mailbox_read_offset,
                    &mut self.data,
                    TIMEOUT_RETURN,
                )?;

                // Mailbox error response
                if work_counter > 0 && mailbox_header.mailbox_type.trailing_zeros() >= 4 {
                    let mailbox_error = MailboxError::read_from(&mut self.data.as_slice())?;
                    context.mailbox_error(slave, mailbox_error.detail.to_host());

                    // Prevent emergency to cascade up, it's already handled
                    work_counter = 0;
                } else if work_counter > 0
                    && mailbox_header.mailbox_type & 0xF == MailboxType::CanopenOverEthercat.into()
                {
                    // If a CanOpen over EtherCAT response was received
                    let emergency_request = EmergencyRequest::read_from(&mut self.data.as_slice())?;
                    // Is it an emergency request
                    if emergency_request.can_open.to_host() >> 12 == 1 {
                        context.mailbox_emergency_error(
                            slave,
                            emergency_request.error_code.to_host(),
                            emergency_request.error_register,
                            emergency_request.byte_data,
                            emergency_request.word1.to_host(),
                            emergency_request.word2.to_host(),
                        );
                        // Prevent emergency to cascade up, it's already handled
                        work_counter = 0;
                    }
                } else if work_counter > 0
                    && mailbox_header.mailbox_type & 0xF == MailboxType::EthernetOverEthercat.into()
                {
                    // Ethernet over EtherCAT response
                    let eoe_mailbox = EthernetOverEthercat::read_from(&mut self.data.as_slice())?;
                    let frame_info1 = eoe_mailbox.frame_info1().to_host();

                    // All non fragment data frame types are expected to be handled by slave
                    // send/receive API if the Ethernet over EtherCAT hook is set.
                    if header_frame_type_get(frame_info1 as u8)
                        .is_ok_and(|frame_type| frame_type == EoEFrameType::FragmentData)
                        && context.ethernet_over_ethercat_hook.is_some_and(|eoe_hook| {
                            eoe_hook(context, slave, self.data.as_mut_slice()) != 0
                        })
                    {
                        // Fragment handled by Ethernet over EtherCAT hook
                        work_counter = 0;
                    }
                } else if work_counter == 0 {
                    // Read mailbox lost
                    let sync_manager_status = Ethercat::from_host(sync_manager_status ^ 0x200);
                    let word = sync_manager_status.to_bytes();
                    fpwr(
                        &mut context.port,
                        config_address,
                        EthercatRegister::SyncManager1Status.into(),
                        &word,
                        TIMEOUT_RETURN,
                    )?;
                    let sync_manager_status = Ethercat::<u16>::from_bytes(word);

                    // Wait for toggle ack
                    loop {
                        let mut sync_manager_control = [0];
                        let work_counter2 = fprd(
                            &mut context.port,
                            config_address,
                            EthercatRegister::SyncManager1Control.into(),
                            &mut sync_manager_control,
                            TIMEOUT_RETURN,
                        )?;
                        if (work_counter2 > 0
                            && sync_manager_control[0] & 2
                                == high_byte(sync_manager_status.to_host()) & 2)
                            || timer.is_expired()
                        {
                            break;
                        }
                    }

                    // Wait for read mailbox available
                    loop {
                        let mut word = sync_manager_status.to_bytes();
                        let work_counter2 = fprd(
                            &mut context.port,
                            config_address,
                            EthercatRegister::SyncManager1Status.into(),
                            &mut word,
                            TIMEOUT_RETURN,
                        )?;
                        let sync_manager_status = Ethercat::<u16>::from_bytes(word).to_host();
                        if sync_manager_status & 8 == 0 && timeout == LOCAL_DELAY {
                            thread::sleep(LOCAL_DELAY);
                        }
                        if (work_counter2 > 0 && sync_manager_status & 8 == 0) || timer.is_expired()
                        {
                            break;
                        }
                    }
                }

                if work_counter > 0 || timer.is_expired() {
                    break;
                }
            }
        } else if work_counter > 0 {
            // No read mailbox available
            return Err(MainError::NoMailboxFound);
        }
        Ok(work_counter)
    }
}

impl AsRef<[u8]> for MailboxIn {
    fn as_ref(&self) -> &[u8] {
        &self.data
    }
}

impl AsMut<[u8]> for MailboxIn {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }
}

impl Default for MailboxIn {
    fn default() -> Self {
        Self {
            data: [0; MAX_MAILBOX_SIZE],
            length: 0,
        }
    }
}

#[derive(Debug)]
pub struct MailboxOut {
    data: [u8; MAX_MAILBOX_SIZE],
    write_index: usize,
}

impl Write for MailboxOut {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        let written = (&mut self.data[self.write_index..]).write(buf)?;
        self.write_index += written;
        Ok(written)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.write_index = 0;
        Ok(())
    }
}

impl MailboxOut {
    /// Write to IN mailbox of slave.
    ///
    /// # Parameters
    /// `self`: Mailbox data
    /// `context`: Context struct
    /// `slave`: Slave number
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - No mailbox was found for the slave
    /// - The mailbox of the slave is already full
    /// - Data couldn't be send
    ///
    /// # Returns
    /// Work counter (> 0 if success)
    pub fn send(
        &mut self,
        context: &mut Context,
        slave: u16,
        timeout: Duration,
    ) -> Result<u16, MainError> {
        let slave_usize = usize::from(slave);
        let mailbox_length = context.slavelist[slave_usize].mailbox().length();
        if mailbox_length == 0 || usize::from(mailbox_length) > MAX_MAILBOX_SIZE {
            return Err(MainError::NoMailboxFound);
        }
        let config_address = context.slavelist[slave_usize].config_address;
        if !context.mailbox_empty(slave, timeout)? {
            return Err(MainError::MailboxFull);
        }

        let mailbox_write_offset = context.slavelist[slave_usize].mailbox().write_offset();

        // Write to slave in mailbox.
        fpwr(
            &mut context.port,
            config_address,
            mailbox_write_offset,
            &self.data[..self.write_index],
            TIMEOUT_RETURN3,
        )
        .map_err(MainError::from)
    }
}

impl AsRef<[u8]> for MailboxOut {
    fn as_ref(&self) -> &[u8] {
        &self.data
    }
}

impl AsMut<[u8]> for MailboxOut {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.data
    }
}

impl Default for MailboxOut {
    fn default() -> Self {
        Self {
            data: [0; MAX_MAILBOX_SIZE],
            write_index: 0,
        }
    }
}

/// Standard ethercat mailbox header
#[derive(Debug, Default)]
pub struct MailboxHeader {
    length: Ethercat<u16>,
    address: Ethercat<u16>,
    priority: u8,
    mailbox_type: u8,
}

impl MailboxHeader {
    pub const fn new(
        length: Ethercat<u16>,
        address: Ethercat<u16>,
        priority: u8,
        mailbox_type: u8,
    ) -> Self {
        Self {
            length,
            address,
            priority,
            mailbox_type,
        }
    }
    pub const fn size() -> usize {
        2 * size_of::<u16>() + 2 * size_of::<u8>()
    }

    pub fn length_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.length
    }

    pub const fn length(&self) -> Ethercat<u16> {
        self.length
    }

    pub fn address_mut(&mut self) -> &mut Ethercat<u16> {
        &mut self.address
    }

    pub fn priority_mut(&mut self) -> &mut u8 {
        &mut self.priority
    }

    pub fn mailbox_type_mut(&mut self) -> &mut u8 {
        &mut self.mailbox_type
    }

    pub const fn mailbox_type(&self) -> u8 {
        self.mailbox_type
    }

    /// # Errors
    /// Returns an error if:
    /// - The reader doesn't have enough data to create a `MailboxHeader`
    pub fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
        let mut word = [0; 2];
        reader.read_exact(&mut word)?;
        let length = Ethercat::<u16>::from_bytes(word);
        reader.read_exact(&mut word)?;
        let address = Ethercat::<u16>::from_bytes(word);
        reader.read_exact(&mut word)?;
        Ok(Self {
            length,
            address,
            priority: word[0],
            mailbox_type: word[1],
        })
    }
}

impl<W: Write> WriteTo<W> for MailboxHeader {
    fn write_to(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&self.length.to_bytes())?;
        writer.write_all(&self.address.to_bytes())?;
        writer.write_all(&[self.priority, self.mailbox_type])
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ApplicationLayerStatus {
    status: Ethercat<u16>,

    /// 2-bytes
    unused: (),

    code: Ethercat<u16>,
}

impl<R: Read> ReadFrom<R> for ApplicationLayerStatus {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let status = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let _unused = <[u8; 2]>::read_from(reader)?;
        let code = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        Ok(Self {
            status,
            unused: (),
            code,
        })
    }
}

impl<W: Write> WriteTo<W> for ApplicationLayerStatus {
    fn write_to(&self, writer: &mut W) -> io::Result<()> {
        writer.write_all(&self.status.to_bytes())?;
        writer.write_all(&[0; 2])?;
        writer.write_all(&self.code.to_bytes())
    }
}

impl ApplicationLayerStatus {
    pub const fn size() -> usize {
        size_of::<u16>() * 2 + 2
    }

    /// # Errors
    /// Returns an error if the application layer status couldn't be converted to bytes
    #[expect(
        clippy::missing_panics_doc,
        reason = "Won't panic if Self::size is written correctly"
    )]
    pub fn bytes(&self) -> [u8; Self::size()] {
        let mut result = [0; Self::size()];
        self.write_to(&mut result.as_mut_slice()).unwrap();
        result
    }
}

#[derive(Debug, Default)]
struct IndexBuffer {
    index: u8,
    data: Vec<u8>,
    dc_offset: u16,
}

/// Stack structure to store segmented logical read, logical write, logical read/write constructs
#[derive(Debug, Default)]
struct IndexStack {
    pushed: u8,
    pulled: u8,
    buffers: [IndexBuffer; MAX_BUFFER_COUNT],
}

impl IndexStack {
    /// Push index of segmented logical read, logical write, logical read/write combination
    ///
    /// # Parameters
    /// `self`: Index stack
    /// `index`: Used datagram index
    /// `data`: Process data segment
    /// `dc_offset`: Offset position of DC frame
    fn push_index(&mut self, index: u8, data: &[u8], dc_offset: u16) {
        if usize::from(self.pushed) >= MAX_BUFFER_COUNT {
            return;
        }
        let pushed_usize = usize::from(self.pushed);
        self.buffers[pushed_usize].index = index;
        self.buffers[pushed_usize].data = data.to_vec();
        self.buffers[pushed_usize].dc_offset = dc_offset;
        self.pushed += 1;
    }

    /// Pull index of segmented logical read, logical write, logical read/write combination
    /// `self`: Index stack
    ///
    /// # Returns stack location, None if empty
    fn pull_index(&mut self) -> Option<u8> {
        if self.pulled >= self.pushed {
            return None;
        }
        let return_value = self.pulled;
        self.pulled += 1;
        Some(return_value)
    }

    /// Clear the index stack
    ///
    /// # Parameters
    /// `self`: Index stack
    fn clear_index(&mut self) {
        self.pushed = 0;
        self.pulled = 0;
    }
}

/// Ringbuffer for error storage
#[derive(Debug, Default)]
struct ErrorRing {
    head: u16,
    tail: u16,
    error: heapless::Vec<ErrorInfo, { MAX_ERROR_LIST_ENTRIES + 1 }>,
}

/// Sync manager communication type structure for communication access
#[derive(Debug, Clone)]
pub struct SyncManagerCommunicationType {
    number: u8,
    null: u8,
    sync_manager_type: [SyncManagerType; MAX_SM as usize],
}

impl SyncManagerCommunicationType {
    pub fn number_mut(&mut self) -> &mut u8 {
        &mut self.number
    }

    pub const fn number(&self) -> u8 {
        self.number
    }

    pub fn get_sync_manager_type(&self, sync_manager_index: u8) -> SyncManagerType {
        self.sync_manager_type[usize::from(sync_manager_index)]
    }
}

impl From<SyncManagerCommunicationType> for [u8; 2 + MAX_SM as usize] {
    fn from(value: SyncManagerCommunicationType) -> Self {
        array::from_fn(|i| match i {
            0 => value.number,
            1 => value.null,
            2.. => u8::from(value.sync_manager_type[i - 2]),
        })
    }
}

impl TryFrom<[u8; 2 + MAX_SM as usize]> for SyncManagerCommunicationType {
    type Error = MainError;

    fn try_from(value: [u8; 2 + MAX_SM as usize]) -> Result<Self, Self::Error> {
        let mut sync_manager_type = [SyncManagerType::default(); MAX_SM as usize];
        for i in 0..sync_manager_type.len() {
            sync_manager_type[i] = SyncManagerType::try_from(value[i + 2])?;
        }
        Ok(Self {
            number: value[0],
            null: value[1],
            sync_manager_type,
        })
    }
}

/// Service data object assign structure for communication access
#[derive(Debug, Clone, Copy)]
pub struct PdoAssign {
    number: u8,
    null: u8,
    index: [Ethercat<u16>; 256],
}

impl PdoAssign {
    pub const fn size() -> usize {
        2 * size_of::<u8>() + size_of::<[u16; 256]>()
    }

    pub fn number_mut(&mut self) -> &mut u8 {
        &mut self.number
    }

    pub const fn number(&self) -> u8 {
        self.number
    }

    pub const fn index(&self) -> &[Ethercat<u16>; 256] {
        &self.index
    }
}

impl<R: Read> ReadFrom<R> for PdoAssign {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> io::Result<Self> {
        let number = u8::read_from(reader)?;
        let null = u8::read_from(reader)?;
        let index = (0..256).try_fold([Ethercat::default(); 256], |result, index| {
            let mut result = result;
            result[index] = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
            Ok::<_, io::Error>(result)
        })?;
        Ok(Self {
            number,
            null,
            index,
        })
    }
}

/// Service data object assign structure for communication access
#[derive(Debug, Clone, Copy)]
pub struct PdoDescription {
    number: u8,
    null: u8,
    pdo: [Ethercat<u32>; 256],
}

impl PdoDescription {
    pub fn number_mut(&mut self) -> &mut u8 {
        &mut self.number
    }

    pub const fn number(&self) -> u8 {
        self.number
    }

    pub const fn pdo(&self) -> &[Ethercat<u32>; 256] {
        &self.pdo
    }

    pub const fn size() -> usize {
        size_of::<u8>() * 2 + size_of::<[u32; 256]>()
    }
}

impl<R: Read> ReadFrom<R> for PdoDescription {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> io::Result<Self> {
        let number = u8::read_from(reader)?;
        let null = u8::read_from(reader)?;
        let pdo = (0..256).try_fold([Ethercat::default(); 256], |result, index| {
            let mut result = result;
            result[index] = Ethercat::<u32>::from_bytes(<[u8; 4]>::read_from(reader)?);
            Ok::<_, io::Error>(result)
        })?;
        Ok(Self { number, null, pdo })
    }
}

#[derive(Debug, Clone, Copy)]
pub enum PacketError {
    UnexpectedFrameReturned = 1,
    DataContainerTooSmallForType = 3,
    NoResponse = 4,
    TooManySyncManagers = 10,
}

/// Struct to retrieve errors
#[derive(Debug, Clone, Copy)]
struct ErrorInfo {
    /// TIme at which the error was generated
    time: SystemTime,

    /// Signal bit, error set but not read
    signal: bool,

    /// Slave number that generated the error
    slave: u16,

    /// GoE Service Data Object index that generated the error
    index: u16,

    /// GoE Service Data Object subindex that generated the error
    sub_index: u8,

    /// Type of error
    error_type: ErrorType,

    abort_error: AbortError,
}

/// Context structure referenced by all Ethernet eXtended functions
pub struct Context<'context> {
    /// Port reference may include red port
    port: Port,

    slavelist: Vec<Slave<'context>>,

    /// Number of slaves found in configuration
    slave_count: u16,

    /// Maximum nummber of slaves allowed in slavelist
    max_slaves: u16,

    grouplist: Vec<SlaveGroup<'context>>,

    /// Maximum number of groups allowed in grouplist
    max_group: u32,

    /// Internal reference to eeprom cache buffer
    esibuf: Vec<u8>,

    /// internal reference to eeprom cache map
    esimap: Vec<u8>,

    /// Internal current slave for eeprom cache
    esislave: u16,

    /// Internal reference to error list
    elist: ErrorRing,

    /// Internal reference to processdata stack buffer info
    index_stack: IndexStack,

    /// Reference to ecaterror state
    ecaterror: bool,

    /// Reference to last DC time from slaves
    dc_time: i64,

    /// Internal Sync manager buffer
    sync_manager_communication_type: Vec<SyncManagerCommunicationType>,

    /// Internal pdo assign list
    pdo_assign: Vec<PdoAssign>,

    /// Internal pdo description list
    pdo_description: Vec<PdoDescription>,

    /// Internal eeprom sync manager list
    eep_sync_manager: EepromSyncManager,

    // Internal eeprom FMMU list
    eep_fmmu: EepromFmmu,

    /// Registered file over ethercat hook
    file_over_ethercat_hook: Option<fn(slave: u16, packetnumber: i32, datasize: i32) -> i32>,

    /// Registered Ethernet over ethercat hook
    #[expect(clippy::type_complexity)]
    ethernet_over_ethercat_hook:
        Option<fn(context: &mut Context, slave: u16, ecembx: &mut [u8]) -> i32>,

    /// Flag to control legacy automatic state change or manual state change
    manual_state_change: bool,

    /// Userdata promotes application configuration especially in EC_VER2 with multiple ec_context
    /// instances.
    userdata: Vec<u8>,
}

impl<'context> Context<'context> {
    /// Ethernet over Ethercat progress hook
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `hook`: Pointer to hook function
    pub fn set_ethernet_over_ethercat_hook(
        &mut self,
        hook: fn(context: &mut Context, slave: u16, ece_mailbox: &mut [u8]) -> i32,
    ) {
        self.ethernet_over_ethercat_hook = Some(hook);
    }

    /// File over Ethercat progress hook
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `hook`: Pointer to hook function
    pub fn set_file_over_ethercat_hook(
        &mut self,
        hook: fn(slave: u16, packetnumber: i32, datasize: i32) -> i32,
    ) {
        self.file_over_ethercat_hook = Some(hook);
    }

    pub const fn file_over_ethercat_hook(
        &self,
    ) -> Option<fn(slave: u16, packetnumber: i32, datasize: i32) -> i32> {
        self.file_over_ethercat_hook
    }

    pub const fn manual_state_change(&self) -> bool {
        self.manual_state_change
    }

    pub fn get_slave_mut(&mut self, slave: u16) -> &mut Slave<'context> {
        &mut self.slavelist[usize::from(slave)]
    }

    pub fn get_slave(&self, slave: u16) -> &Slave<'context> {
        &self.slavelist[usize::from(slave)]
    }

    pub fn get_pdo_assign(&self, thread_number: usize) -> PdoAssign {
        self.pdo_assign[thread_number]
    }

    pub fn get_pdo_assign_mut(&mut self, thread_number: usize) -> &mut PdoAssign {
        &mut self.pdo_assign[thread_number]
    }

    pub fn get_pdo_description(&self, thread_number: usize) -> PdoDescription {
        self.pdo_description[thread_number]
    }

    pub fn get_pdo_description_mut(&mut self, thread_number: usize) -> &mut PdoDescription {
        &mut self.pdo_description[thread_number]
    }

    pub fn get_sync_manager_communication_type_mut(
        &mut self,
        thread_number: usize,
    ) -> &mut SyncManagerCommunicationType {
        &mut self.sync_manager_communication_type[thread_number]
    }

    pub fn port_mut(&mut self) -> &mut Port {
        &mut self.port
    }

    pub fn slave_count_mut(&mut self) -> &mut u16 {
        &mut self.slave_count
    }

    pub const fn slave_count(&self) -> u16 {
        self.slave_count
    }

    pub const fn max_slaves(&self) -> u16 {
        self.max_slaves
    }

    pub fn reset_slave_list(&mut self) {
        self.slavelist.clear();
        if self.slavelist.capacity() < usize::from(self.max_slaves) {
            self.slavelist
                .reserve_exact(usize::from(self.max_slaves) - self.slavelist.capacity());
        }
    }

    pub fn slavelist(&self) -> &[Slave<'context>] {
        &self.slavelist
    }

    pub fn slavelist_mut(&mut self) -> &mut [Slave<'context>] {
        &mut self.slavelist
    }

    pub fn reset_group_list(&mut self) {
        self.grouplist.clear();
        if self.grouplist.capacity() < self.max_group as usize {
            self.grouplist
                .reserve_exact(self.max_group as usize - self.grouplist.capacity());
        }
    }

    pub fn push_group(&mut self, slave_group: SlaveGroup<'context>) {
        self.grouplist.push(slave_group);
    }

    pub fn get_group(&self, group: u8) -> &SlaveGroup {
        &self.grouplist[usize::from(group)]
    }

    pub fn get_group_mut(&mut self, group: u8) -> &mut SlaveGroup<'context> {
        &mut self.grouplist[usize::from(group)]
    }

    pub const fn max_group(&self) -> u32 {
        self.max_group
    }

    pub fn eep_sync_manager_mut(&mut self) -> &mut EepromSyncManager {
        &mut self.eep_sync_manager
    }

    pub const fn eep_sync_manager(&self) -> &EepromSyncManager {
        &self.eep_sync_manager
    }

    pub fn eep_fmmu_mut(&mut self) -> &mut EepromFmmu {
        &mut self.eep_fmmu
    }

    pub const fn eep_fmmu(&self) -> EepromFmmu {
        self.eep_fmmu
    }

    /// Report Service Data Object error
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `slave`: slave number
    /// - `index`: index that generated the error
    /// - `sub_index`: Subindex that generated the error
    /// - `abort_error`: Abort code or error, see EtherCAT documentation for list
    pub fn sdo_error(&mut self, slave: u16, index: u16, sub_index: u8, abort_error: AbortError) {
        self.ecaterror = true;
        self.push_error(ErrorInfo {
            time: SystemTime::now(),
            slave,
            index,
            sub_index,
            error_type: ErrorType::ServiceDataObjectError,
            abort_error,
            signal: false,
        });
    }

    /// Report Service Data Object info error
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `slave`: Slave number
    /// - `index`: Index that generated the error
    /// - `sub_index`: Subindex that generated the error
    /// - `abort_error`: Abort code or error, see EtherCAT documentation for list
    pub(super) fn sdo_info_error(
        &mut self,
        slave: u16,
        index: u16,
        sub_index: u8,
        abort_error: AbortError,
    ) {
        self.ecaterror = true;
        self.push_error(ErrorInfo {
            time: SystemTime::now(),
            signal: false,
            slave,
            index,
            sub_index,
            error_type: ErrorType::SdoInfoError,
            abort_error,
        });
    }

    /// Initialize library in single NIC mode
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `interface_name`: Device name, f.e. "eth0"
    ///
    /// # Errors
    /// Returns an error if the NIC couldn't be setup
    ///
    /// # Returns
    /// `Ok(())` or error
    pub fn init(interface_name: &str) -> Result<Self, ConfigError> {
        let mut result = Self {
            port: Port::setup_nic(interface_name, RedundancyMode::None)?,
            slavelist: Vec::new(),
            slave_count: 0,
            max_slaves: MAX_SLAVES as u16,
            grouplist: Vec::new(),
            max_group: MAX_GROUPS as u32,
            esibuf: Vec::with_capacity(usize::from(MAX_EEPROM_BUFFER_SIZE)),
            esimap: Vec::with_capacity(usize::from(MAX_EEPROM_BUFFER_SIZE)),
            esislave: 0,
            elist: ErrorRing::default(),
            index_stack: IndexStack::default(),
            ecaterror: false,
            dc_time: 0,
            sync_manager_communication_type: Vec::new(),
            pdo_assign: Vec::new(),
            pdo_description: Vec::new(),
            eep_sync_manager: EepromSyncManager::default(),
            eep_fmmu: EepromFmmu::default(),
            file_over_ethercat_hook: None,
            ethernet_over_ethercat_hook: None,
            manual_state_change: false,
            userdata: Vec::new(),
        };
        super::config::config_init(&mut result, false)?;
        config_dc(&mut result)?;
        Ok(result)
    }

    /// Initialize library in redundant NIC mode
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `redport`: Mutable reference to redundant port data
    /// - `interface_name`: Primary device name f.e. "etho"
    /// - `interface_name2`: Secondary device name, f.e. "eth1"
    ///
    /// # Errors
    /// Returns an error if:
    /// - The NIC couldn't be setup
    /// - The `temp_tx_buffer` doesn't contain enoguh data to create an `EthernetHeader`
    ///
    /// # Returns
    /// `Ok(())` or error
    #[expect(
        clippy::missing_panics_doc,
        reason = "New buffer size is checked at compile time"
    )]
    pub fn init_redundant(
        interface_name: &str,
        interface_name2: &str,
    ) -> Result<Self, ConfigError> {
        const _: () = assert!(
            EthernetHeader::size() + EthernetHeader::size() + ETHERCAT_WORK_COUNTER_SIZE + 2
                < BUFFER_SIZE
        );
        let mut port = Port::setup_nic(interface_name, RedundancyMode::Redundant(interface_name2))?;

        // Prepare "dummy" broadcat read tx frame for redundant operation
        let mut ethernet_header =
            EthernetHeader::read_from(&mut port.stack().temp_tx_buf().as_slice())?;
        ethernet_header.source_address_mut()[1] = Network::from_host(SECONDARY_MAC[0]);
        ethernet_header.write_to(&mut port.stack_mut().temp_tx_buf_mut().as_mut_slice())?;
        let zbuf = [0; 2];
        setup_datagram(
            port.stack_mut().temp_tx_buf_mut(),
            ReadCommand::BroadcastRead.into(),
            0,
            0,
            2,
            &zbuf,
        )?;
        port.stack_mut()
            .temp_tx_buf_mut()
            .resize(
                EthernetHeader::size() + EthernetHeader::size() + ETHERCAT_WORK_COUNTER_SIZE + 2,
                0,
            )
            .unwrap();
        let mut result = Self {
            port: Port::setup_nic(interface_name, RedundancyMode::None)?,
            slavelist: Vec::new(),
            slave_count: 0,
            max_slaves: MAX_SLAVES as u16,
            grouplist: Vec::new(),
            max_group: MAX_GROUPS as u32,
            esibuf: Vec::with_capacity(usize::from(MAX_EEPROM_BUFFER_SIZE)),
            esimap: Vec::with_capacity(usize::from(MAX_EEPROM_BUFFER_SIZE)),
            esislave: 0,
            elist: ErrorRing::default(),
            index_stack: IndexStack::default(),
            ecaterror: false,
            dc_time: 0,
            sync_manager_communication_type: Vec::new(),
            pdo_assign: Vec::new(),
            pdo_description: Vec::new(),
            eep_sync_manager: EepromSyncManager::default(),
            eep_fmmu: EepromFmmu::default(),
            file_over_ethercat_hook: None,
            ethernet_over_ethercat_hook: None,
            manual_state_change: false,
            userdata: Vec::new(),
        };
        super::config::config_init(&mut result, false)?;
        config_dc(&mut result)?;
        Ok(result)
    }

    /// Pushes error on the error list
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `error`: Error describtion
    fn push_error(&mut self, mut error: ErrorInfo) {
        error.signal = true;
        self.elist.error[usize::from(self.elist.head)] = error;
        self.elist.head += 1;
        if usize::from(self.elist.head) >= MAX_ERROR_LIST_ENTRIES {
            self.elist.head = 0;
        }
        if self.elist.head == self.elist.tail {
            self.elist.tail += 1;
        }
        if usize::from(self.elist.tail) >= MAX_ERROR_LIST_ENTRIES {
            self.elist.tail = 0;
        }
        self.ecaterror = true;
    }

    /// Pops an error from the list
    ///
    /// # Parameters
    /// - `self`: Context struct
    ///
    /// # Returns
    /// `Some(ErrorInfo)` if an error is present, None if empty
    fn pop_error(&mut self) -> Option<ErrorInfo> {
        self.elist.error[usize::from(self.elist.tail)].signal = false;
        if self.elist.head == self.elist.tail {
            self.ecaterror = false;
            None
        } else {
            self.elist.tail += 1;
            if usize::from(self.elist.tail) >= MAX_ERROR_LIST_ENTRIES {
                self.elist.tail = 0;
            }
            Some(self.elist.error[usize::from(self.elist.tail)])
        }
    }

    /// Checks if error list has entries
    ///
    /// # Parameters
    /// - `self`: Context struct
    ///
    /// # Returns
    /// true if error list contains entries
    pub const fn is_error(&self) -> bool {
        self.elist.head != self.elist.tail
    }

    /// Report packet error
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `slave`: Slave number
    /// - `index`: Index that generated the error
    /// - `sub_index`: Subindex that generated the error
    /// - `error_code`: Error code
    pub fn packet_error(&mut self, slave: u16, index: u16, sub_index: u8, error_code: PacketError) {
        self.ecaterror = true;
        self.push_error(ErrorInfo {
            time: SystemTime::now(),
            slave,
            index,
            sub_index,
            error_type: ErrorType::PacketError,
            abort_error: AbortError::PacketError(error_code),
            signal: false,
        });
    }

    /// Report Servo over EtherCAT error.
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `slave`: Slave number
    /// - `idn`: IDN that generated the error
    /// - `error`: Error code, see EtherCAT documentation for list
    pub(super) fn servo_over_ethercat_error(&mut self, slave: u16, idn: u16, error: u16) {
        self.push_error(ErrorInfo {
            time: SystemTime::now(),
            signal: false,
            slave,
            index: idn,
            sub_index: 0,
            error_type: ErrorType::ServerOverEthercatError,
            abort_error: AbortError::ErrorCode(error),
        });
        self.ecaterror = true;
    }

    /// Report mailbox error
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `slave`: Slave number
    /// - `detail`: Following EtherCAT specification
    fn mailbox_error(&mut self, slave: u16, detail: u16) {
        self.push_error(ErrorInfo {
            time: SystemTime::now(),
            signal: false,
            slave,
            index: 0,
            sub_index: 0,
            error_type: ErrorType::MailboxError,
            abort_error: AbortError::ErrorCode(detail),
        });
    }

    /// Report Mailbox emergency error.
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `slave`: Slave number
    /// - `error_code`: Following EtherCAT specification
    /// - `error_register`
    /// - `byte1`
    /// - `word1`
    /// - `word2`
    fn mailbox_emergency_error(
        &mut self,
        slave: u16,
        error_code: u16,
        error_register: u8,
        byte1: u8,
        word1: u16,
        word2: u16,
    ) {
        self.push_error(ErrorInfo {
            time: SystemTime::now(),
            signal: false,
            slave,
            index: 0,
            sub_index: 0,
            error_type: ErrorType::Emergency,
            abort_error: AbortError::EmergencyError {
                error_code,
                error_register,
                byte1,
                word1,
                word2,
            },
        });
    }

    /// Read one byte from slave EEPROM via cache.
    /// If the cache location is empty, a read request is made to the slave.
    /// Depending on the slave capabilities, the request is 4 or 8 bytes.
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `slave`: Slave number
    /// - `address`: Eeprom address in bytes (slave uses words)
    ///
    /// # Errors
    /// Returns an error if:
    /// - The requested EEPROM address is invalid
    /// - Control over the EEPROM couldn't be set to master
    ///
    /// # Returns
    /// Requested byte or None
    pub fn sii_get_byte(&mut self, slave: u16, address: u16) -> Result<u8, MainError> {
        // Make sure the selected slave is cached
        if slave != self.esislave {
            self.esislave = slave;
        }

        if address >= MAX_EEPROM_BUFFER_SIZE {
            return Err(MainError::InvalidEepromAddress(address));
        }
        if usize::from(address) >= self.esibuf.len() {
            self.esibuf.resize(usize::from(address) + 1, 0);
        }
        let mut mapword = address >> 5;
        let mut mapbyte = address - (mapword << 5);
        if self.esimap.len() <= usize::from(mapword) {
            self.esimap.resize(usize::from(mapword) + 1, 0);
        }
        if self.esimap[usize::from(mapword)] & (1 << mapbyte) != 0 {
            // Byte is already in buffer
            return Ok(self.esibuf[usize::from(address)]);
        }
        // Byte is not in buffer, put it there
        let config_address = self.slavelist[usize::from(slave)].config_address;

        // Set eeprom cotrol to master
        self.eeprom_to_master(slave)?;
        let eeprom_address = address >> 1;
        let eeprom_data64 = self.read_eeprom_fp(config_address, eeprom_address, TIMEOUT_EEPROM)?;

        if self.esibuf.len() <= (usize::from(eeprom_address) << 1) + 8 {
            self.esibuf
                .resize((usize::from(eeprom_address) << 1) + 9, 0);
        }

        let count = if self.slavelist[usize::from(slave)].eeprom().read_size == EepReadSize::Bytes8
        {
            // 8 byte response
            self.esibuf[usize::from(eeprom_address) << 1..]
                .copy_from_slice(&eeprom_data64.to_bytes());
            8
        } else {
            // 4 byte response
            self.esibuf[usize::from(eeprom_address) << 1..]
                .copy_from_slice(&Ethercat::from_host(eeprom_data64.to_host() as u32).to_bytes());
            4
        };

        // Find bitmap location
        mapword = eeprom_address >> 4;
        mapbyte = (eeprom_address << 1) - (mapword << 5);
        for _ in 0..count {
            // Set bitmap for each byte that is read
            self.esimap[usize::from(mapword)] |= 1 << mapbyte;
            mapbyte += 1;
            if mapbyte > 31 {
                mapbyte = 0;
                mapword += 1;
            }
        }
        Ok(self.esibuf[usize::from(address)])
    }

    /// Find SII section header in slave EEPROM
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `slave`: Slave number
    /// - `category`: Section category
    ///
    /// # Errors
    /// Returns an error if:
    /// - A byte couldn't be read from the EEPROM
    /// - The requested SII category couldn't be found
    ///
    /// # Returns
    /// byte address of section at section length entry, None if not available
    pub fn sii_find(&mut self, slave: u16, category: SiiCategory) -> Result<u16, MainError> {
        let slave_usize = usize::from(slave);
        let eeprom_control = self.slavelist[slave_usize].eeprom().control();

        let mut address = SII_START << 1;

        // Read first SII section category
        let mut p = u16::from(self.sii_get_byte(slave, address)?);
        p += u16::from(self.sii_get_byte(slave, address + 1)?) << 8;
        address += 2;

        // Traverse SII while category is not found and not End Of File
        while p != u16::from(category) && p != 0xFFFF {
            // Read section length
            p = match self.sii_get_byte(slave, address) {
                Ok(byte) => u16::from(byte),
                Err(_) => break,
            };
            p += match self.sii_get_byte(slave, address + 1) {
                Ok(byte) => u16::from(byte) << 8,
                Err(_) => break,
            };

            // Locate next section category
            address += p << 1;

            // Read section category
            p = match self.sii_get_byte(slave, address) {
                Ok(byte) => u16::from(byte),
                Err(_) => break,
            };
            p += match self.sii_get_byte(slave, address + 1) {
                Ok(byte) => u16::from(byte) << 8,
                Err(_) => break,
            }
        }
        if eeprom_control == EepromControl::Pdi {
            self.eeprom_to_pdi(slave)?;
        }
        (p == u16::from(category))
            .then_some(p)
            .ok_or(MainError::SiiCategoryNotFound(category))
    }

    /// Get string from SII string section in slave EEPROM.
    ///
    /// # Parameters
    ///`self`: Context struct
    /// `string`: Requested string
    /// `slave`: Slave number
    /// `string_number`: String number
    ///
    /// # Panics
    /// Panics if a byte couldn't be added to the string
    ///
    /// # Errors
    /// Returns an error if:
    /// - A byte of the string couldn't be read
    ///
    /// # Returns
    /// The resulting string or error
    pub fn sii_string<const SIZE: usize>(
        &mut self,
        slave: u16,
        string_number: u16,
    ) -> Result<heapless::String<SIZE>, MainError> {
        let slave_usize = usize::from(slave);
        let eeprom_control = self.slavelist[slave_usize].eeprom().control();

        let mut result = heapless::Vec::<u8, SIZE>::new();
        if let Ok(address) = self.sii_find(slave, SiiCategory::String) {
            // Skip SII section header
            let mut address = address + 2;

            // Read number of strings in section
            let number = self.sii_get_byte(slave, address);
            address += 1;

            // Check whether the requested string is available
            if number.is_ok_and(|number| string_number <= u16::from(number)) {
                for i in 1..=string_number {
                    let Ok(length) = self.sii_get_byte(slave, address) else {
                        continue;
                    };
                    if i < string_number {
                        address += u16::from(length);
                        continue;
                    }
                    for j in 1..=length {
                        if u16::from(j) > MAX_NAME_LENGTH {
                            address += 1;
                            continue;
                        }
                        result.push(self.sii_get_byte(slave, address)?).unwrap();
                        address += 1;
                    }
                }
            }
        }

        // If eeprom control was previously pdi, then restore
        if eeprom_control == EepromControl::Pdi {
            self.eeprom_to_pdi(slave)?;
        }
        Ok(HeaplessString::from_utf8(result)?)
    }

    /// Read all slave states in `ec_slave`
    ///
    /// # Warning
    /// The BOOT state is actually higher than INIT and PRE_OP (see state representation)
    ///
    /// # Parameters
    /// `context`: Context struct
    ///
    /// # Errors
    /// Returns an error if:
    /// - Failed to read the state of the slaves
    /// - Failed to read application layer status of slaves
    /// - Received state isn't a valid ethercat state
    ///
    /// # Returns
    /// lowest state found.
    pub fn read_state(&mut self) -> Result<EthercatState, MainError> {
        // Try to establish the state of all slaves, sending only one broadcast datagram.
        // This way, a number of datagrams equal to the number of slaves will be send when needed.
        let mut byte = [0];
        let wkc = brd(
            &mut self.port,
            0,
            EthercatRegister::ApplicationLayerStatus,
            &mut byte,
            TIMEOUT_RETURN,
        )?;

        let all_slaves_present = wkc >= self.slave_count;

        let received_value = byte[0];

        let error_flag = if received_value & u8::from(EthercatState::Error) != 0 {
            false
        } else {
            self.slavelist[0].al_status_code = 0;
            true
        };

        let bitwise_state = received_value & 0xF;
        let all_slaves_same_state = match EthercatState::try_from(bitwise_state) {
            Ok(
                state @ (EthercatState::Init
                | EthercatState::PreOperational
                | EthercatState::SafeOperational
                | EthercatState::Operational),
            ) => {
                self.slavelist[0].state = state;
                true
            }
            _ => false,
        };

        let lowest = if !error_flag && all_slaves_same_state && all_slaves_present {
            // No slave has toggled the error flag, so the al_status_code
            // (even if different from 0) should be ignored and the slaves
            // have reached the same state. So the internal state can be
            // updated without sending any datagram.
            let state = self.slavelist[0].state;
            for slave in &mut self.slavelist[1..=usize::from(self.slave_count)] {
                slave.al_status_code = 0;
                slave.state = state;
            }
            state
        } else {
            // Not all slaves have the same state or at least one is in error, so one datagram per
            // slave is needed.
            self.slavelist[0].al_status_code = 0;
            let mut lowest = 0xFF;
            let mut fslave = 1;
            let mut slca = [0; MAX_FIXED_POINTER_READ_MULTI];
            let mut sl = [ApplicationLayerStatus::default(); MAX_FIXED_POINTER_READ_MULTI];
            loop {
                let lslave =
                    if usize::from(self.slave_count - fslave) >= MAX_FIXED_POINTER_READ_MULTI {
                        fslave + MAX_FIXED_POINTER_READ_MULTI as u16 - 1
                    } else {
                        self.slave_count
                    };

                for slave in fslave..=lslave {
                    const ZERO: ApplicationLayerStatus = ApplicationLayerStatus {
                        status: Ethercat::from_raw(0),
                        unused: (),
                        code: Ethercat::from_raw(0),
                    };
                    let config_address = self.slavelist[usize::from(slave)].config_address;
                    slca[usize::from(slave - fslave)] = config_address;
                    sl[usize::from(slave - fslave)] = ZERO;
                }

                fixed_pointer_read_multi(
                    self,
                    (lslave - fslave) + 1,
                    &mut slca[..1],
                    &mut sl[..1],
                    TIMEOUT_RETURN3,
                )?;

                for slave in fslave..=lslave {
                    let received_value = sl[usize::from(slave - fslave)].status.to_host();
                    self.slavelist[usize::from(slave)].al_status_code =
                        sl[usize::from(slave - fslave)].code.to_host();
                    lowest = lowest.min(received_value & 0xF);
                    self.slavelist[usize::from(slave)].state =
                        EthercatState::try_from(received_value as u8)?;
                    self.slavelist[0].al_status_code |=
                        self.slavelist[usize::from(slave)].al_status_code;
                }
                fslave = lslave + 1;

                if lslave >= self.slave_count {
                    break;
                }
            }
            let lowest = EthercatState::try_from(lowest as u8)?;
            self.slavelist[0].state = lowest;
            lowest
        };

        Ok(lowest)
    }

    /// Write slave state, if slave = 0, write to all slaves.
    /// The function doesn't check if the actual state is changed.
    ///
    /// # Parameters
    /// `self`: Context struct
    /// `slave`: Slave number, 0 = master
    ///
    /// # Errors
    /// Returns an error if the state couldn't be send.
    ///
    /// # Returns
    /// Workcounter or error
    pub fn write_state(&mut self, slave: u16) -> Result<u16, NicdrvError> {
        if slave == 0 {
            let sl_state = Ethercat::from_host(u16::from(self.slavelist[usize::from(slave)].state));
            bwr(
                &mut self.port,
                0,
                EthercatRegister::ApplicationLayerControl,
                &sl_state.to_bytes(),
                TIMEOUT_RETURN3,
            )
        } else {
            let config_address = self.slavelist[usize::from(slave)].config_address;
            fpwrw(
                &mut self.port,
                config_address,
                EthercatRegister::ApplicationLayerControl,
                Ethercat::from_host(u16::from(self.slavelist[usize::from(slave)].state)),
                TIMEOUT_RETURN3,
            )
        }
    }

    /// Check actual slave state.
    /// This is a blocking function.
    /// to refresh the state of all slaves `context::read_state()` should be called.
    ///
    /// # Warning
    /// If this is used for slave 0 (=all slaves), the state of all slaves is read by a bitwise OR
    /// operation.
    /// The returned value is also the bitwise OR state of all slaves.
    /// This has some implications for the boot state. The boot state representation collides with
    /// INIT | PRE_OP so this function can't be used for slave= 0 and reqstate = STATE_BOOT and if
    /// the returned state is boot, some slaves might actualy be in INIT and PRE_OP and not in BOOT.
    ///
    /// # Parameters
    /// `self`: Context struct
    /// `slave`: Slave number, 0 = all slaves (only the `slavelist[0].state` is refreshed)
    /// `request_state`: Requested state
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - THe application layer status couldn't be read
    /// - `ApplicationLayerStatus` couldn't be created from received data
    ///
    /// # Returns
    /// Requested state or found state after timeout
    pub fn check_state(
        &mut self,
        slave: u16,
        request_state: EthercatState,
        timeout: Duration,
    ) -> Result<u16, MainError> {
        if slave > self.slave_count {
            return Err(MainError::SlaveIndexOutOfBounds(slave));
        }
        let timer = OsalTimer::new(timeout);

        let slave_usize = usize::from(slave);
        let config_address = self.slavelist[slave_usize].config_address;
        let state;
        (self.slavelist[slave_usize].state, state) = loop {
            let rval = if slave < 1 {
                let mut word = [0; 2];
                brd(
                    &mut self.port,
                    0,
                    EthercatRegister::ApplicationLayerStatus,
                    &mut word,
                    TIMEOUT_RETURN,
                )?;
                Ethercat::<u16>::from_bytes(word).to_host()
            } else {
                let mut bytes = [0; 6];
                fprd(
                    &mut self.port,
                    config_address,
                    EthercatRegister::ApplicationLayerStatus.into(),
                    &mut bytes,
                    TIMEOUT_RETURN,
                )?;
                let slstat = ApplicationLayerStatus::read_from(&mut bytes.as_slice())?;
                self.slavelist[slave_usize].al_status_code = slstat.code.to_host();
                slstat.status.to_host()
            };

            // Read slave status
            let state = rval & 0xF;

            if state != request_state.into() {
                thread::sleep(Duration::from_micros(1000));
            }
            if state == request_state.into() || timer.is_expired() {
                break (EthercatState::try_from(rval as u8)?, state);
            }
        };

        Ok(state)
    }

    /// Check if IN mailbox of slave is empty.
    ///
    /// # Parameters
    /// `context`: Context struct
    ///
    /// # Errors
    /// Returns an error if:
    /// - The status of sync manager 0 couldn't be read
    ///
    /// # Returns
    /// Whether the mailbox was empty or error
    pub fn mailbox_empty(&mut self, slave: u16, timeout: Duration) -> Result<bool, NicdrvError> {
        let timer = OsalTimer::new(timeout);
        let config_address = self.slavelist[usize::from(slave)].config_address;

        let (wkc, sync_manager_status) = loop {
            let mut byte = [0];
            let wkc = fprd(
                &mut self.port,
                config_address,
                EthercatRegister::SyncManager0Status.into(),
                &mut byte,
                TIMEOUT_RETURN,
            )?;
            let sync_manager_status = byte[0];
            if sync_manager_status & 8 != 0 && timeout > LOCAL_DELAY {
                thread::sleep(LOCAL_DELAY);
            }
            if (wkc != 0 && sync_manager_status & 8 != 0) || timer.is_expired() {
                break (wkc, sync_manager_status);
            }
        };

        Ok(wkc > 0 && sync_manager_status & 8 == 0)
    }

    /// Dump complete EEPROM data from slave in buffer.
    ///
    /// # Parameter
    /// `self`: Context struct
    /// `slave`: Slave number
    /// `esibuf`: EEPROM data buffer, will stop reading early if not big enough
    ///
    /// # Errors
    /// Returns an error if:
    /// - Eeprom control couldn't be taken by master
    /// - Eeprom control couldn't be returned to PDI
    ///
    /// # Returns
    /// Amount of words (u16) read from eeprom
    pub fn esi_dump(&mut self, slave: u16, esibuf: &mut [u8]) -> Result<usize, MainError> {
        let slave_usize = usize::from(slave);
        let eeprom_control = self.slavelist[slave_usize].eeprom().control();

        // Set eeprom control to master
        self.eeprom_to_master(slave)?;
        let config_address = self.slavelist[slave_usize].config_address;
        let increment = if self.slavelist[slave_usize].eeprom().read_size == EepReadSize::Bytes8 {
            4
        } else {
            8
        };

        let mut last_p16 = 0;
        for (p16, address) in (0..=(u32::MAX as usize)
            .min(esibuf.len())
            .saturating_sub(increment))
            .zip(SII_START..MAX_EEPROM_BUFFER_SIZE >> 1)
            .step_by(increment)
        {
            let edat = self.read_eeprom_fp(config_address, address, TIMEOUT_EEPROM)?;
            esibuf[p16..p16 + increment].copy_from_slice(&edat.to_bytes());
            last_p16 = p16;
        }

        // If eeprom control was previously PDI, then restore
        if eeprom_control == EepromControl::Pdi {
            self.eeprom_to_pdi(slave)?;
        }
        Ok(last_p16 + increment)
    }

    /// Read eeprom from skave bypassing cache.
    ///
    /// # Parameters
    /// `self`: Context struct
    /// `slave`: Slave number
    /// `eeprom_address`: Address in the eeprom
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if the master couldn't take control over the EEPROM.
    ///
    /// # Returns
    /// EEPROM data 32-bit
    pub fn read_eeprom(
        &mut self,
        slave: u16,
        eeprom_address: u16,
        timeout: Duration,
    ) -> Result<Ethercat<u32>, MainError> {
        // Set eeprom control to master
        self.eeprom_to_master(slave)?;
        let config_address = self.slavelist[usize::from(slave)].config_address;
        Ok(Ethercat::from_host(
            self.read_eeprom_fp(config_address, eeprom_address, timeout)?
                .to_host() as u32,
        ))
    }

    /// Write EEPROM to slave bypassing cache.
    ///
    /// # Parameters
    /// `self`: Context struct
    /// `slave`: Slave number
    /// `eeprom_address`: Address in the eeprom
    /// `data`: 16-bit data
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if the master couldn't take control over the EEPROM.
    ///
    /// # Returns
    /// > 0 if OK
    pub fn write_eeprom(
        &mut self,
        slave: u16,
        eeproma: u16,
        data: Ethercat<u16>,
        timeout: Duration,
    ) -> Result<(), MainError> {
        // Sets eeprom control to master
        self.eeprom_to_master(slave)?;
        let config_address = self.slavelist[usize::from(slave)].config_address;
        self.write_eeprom_fp(config_address, eeproma, data, timeout)
    }

    /// Set eeprom control to master. Only if set to PDI.
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave numner
    ///
    /// # Errors
    /// Returns an error if:
    /// - The EEPROM control couldn't be forced from PDI
    /// - EEPROM control couldn't be set to master
    ///
    /// # Returns
    /// Ok(()) or error
    pub fn eeprom_to_master(&mut self, slave: u16) -> Result<(), NicdrvError> {
        let slave_usize = usize::from(slave);
        if self.slavelist[slave_usize].eeprom().control() == EepromControl::Master {
            return Ok(());
        }

        let config_address = self.slavelist[slave_usize].config_address;

        let eeprom_control = [2];
        let mut error_counter = 0;

        // Force eeprom from PDI
        loop {
            let Err(error) = fpwr(
                &mut self.port,
                config_address,
                EthercatRegister::EepromConfig.into(),
                &eeprom_control,
                TIMEOUT_RETURN,
            ) else {
                break;
            };
            if error_counter < DEFAULT_RETRIES {
                error_counter += 1;
            } else {
                return Err(error);
            }
        }

        let eeprom_control = [0];
        error_counter = 0;

        // Set eeprom to master
        loop {
            let Err(error) = fpwr(
                &mut self.port,
                config_address,
                EthercatRegister::EepromConfig.into(),
                &eeprom_control,
                TIMEOUT_RETURN,
            ) else {
                break;
            };
            if error_counter < DEFAULT_RETRIES {
                error_counter += 1;
            } else {
                return Err(error);
            }
        }
        *self.slavelist[slave_usize].eeprom_mut().control_mut() = EepromControl::Master;

        Ok(())
    }

    /// Set eeprom control to PDI. Only if set to master.
    ///
    /// # Parameter
    /// `context`: context struct
    /// `slave`: Slave number
    ///
    /// # Errors
    /// Returns an error if:
    /// - Eeprom control couldn't be returned to the PDI
    ///
    /// # Returns
    /// `Ok(())` or error
    pub fn eeprom_to_pdi(&mut self, slave: u16) -> Result<(), NicdrvError> {
        if self.slavelist[usize::from(slave)].eeprom().control() == EepromControl::Pdi {
            return Ok(());
        }

        let config_address = self.slavelist[usize::from(slave)].config_address;
        let eep_control = [1];
        let mut error_count = 0;
        loop {
            let Err(error) = fpwr(
                &mut self.port,
                config_address,
                EthercatRegister::EepromConfig.into(),
                &eep_control,
                TIMEOUT_RETURN,
            ) else {
                break;
            };
            if error_count < DEFAULT_RETRIES {
                error_count += 1;
            } else {
                return Err(error);
            }
        }
        *self.slavelist[usize::from(slave)]
            .eeprom_mut()
            .control_mut() = EepromControl::Pdi;
        Ok(())
    }

    fn eeprom_wait_not_busy_ap(
        &mut self,
        auto_increment_address: u16,
        eeprom_state: &mut u16,
        timeout: Duration,
    ) -> Result<bool, NicdrvError> {
        let timer = OsalTimer::new(timeout);
        let mut counter = 0;
        *eeprom_state = 0;
        loop {
            if counter != 0 {
                counter += 1;
            }
            let mut word = [0; 2];
            let Err(err) = aprd(
                &mut self.port,
                auto_increment_address,
                EthercatRegister::EepromControlStat.into(),
                &mut word,
                TIMEOUT_RETURN,
            ) else {
                *eeprom_state = Ethercat::<u16>::from_bytes(word).to_host();
                if *eeprom_state & EEPROM_STATE_MACHINE_BUSY != 0 || timer.is_expired() {
                    return Ok(*eeprom_state & EEPROM_STATE_MACHINE_BUSY == 0);
                }
                continue;
            };
            if timer.is_expired() {
                return Err(err);
            }
        }
    }

    /// Read eeprom from slave bypassing cache. APRD method.
    ///
    /// # Parameters
    /// `self`: Context struct
    /// `auto_increment_address`: Auto increment address of slave
    /// `eeprom_address`: Address in the eeprom
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - The eeprom is too busy
    /// - Error bits couldn't be cleared
    /// - Eeprom configuration couldn't be send
    /// - Eeprom data couldn't be read
    /// - No acknowledges were received from slave
    ///
    /// # Returns
    /// EEPROM data 64-bit or 32-bit
    pub fn read_eeprom_ap(
        &mut self,
        auto_inrecment_address: u16,
        eeprom_address: u16,
        timeout: Duration,
    ) -> Result<Ethercat<u64>, MainError> {
        let mut eeprom_state = 0;
        if !self.eeprom_wait_not_busy_ap(auto_inrecment_address, &mut eeprom_state, timeout)? {
            return Err(MainError::EepromBusy);
        }
        // If error bits are set
        if eeprom_state & EEPROM_STATE_MACHINE_ERROR_MASK != 0 {
            // Clear error bits
            eeprom_state = EepromCommand::Nop.into();
            apwr(
                &mut self.port,
                auto_inrecment_address,
                EthercatRegister::EepromControlStat.into(),
                &Ethercat::from_host(eeprom_state).to_bytes(),
                TIMEOUT_RETURN3,
            )?;
        }

        let mut nack_count = 0;
        loop {
            let eeprom = Eeprom {
                command: EepromCommand::Read,
                address: eeprom_address,
                d2: 0,
            };
            let mut eeprom_bytes = [0; 6];
            eeprom.write_to(&mut eeprom_bytes.as_mut_slice())?;
            let mut retry_count = 0;

            loop {
                let Err(err) = apwr(
                    &mut self.port,
                    auto_inrecment_address,
                    EthercatRegister::EepromControlStat.into(),
                    &eeprom_bytes,
                    TIMEOUT_RETURN,
                ) else {
                    break;
                };
                if retry_count < DEFAULT_RETRIES {
                    retry_count += 1;
                } else {
                    return Err(err.into());
                }
            }

            thread::sleep(LOCAL_DELAY);
            eeprom_state = 0;

            if !self.eeprom_wait_not_busy_ap(auto_inrecment_address, &mut eeprom_state, timeout)? {
                continue;
            }
            if eeprom_state & EEPROM_STATE_MACHINE_ERROR_NACK != 0 {
                nack_count += 1;
                thread::sleep(LOCAL_DELAY * 5);
            } else {
                retry_count = 0;
                if eeprom_state & EEPROM_STATE_MACHINE_READ64 != 0 {
                    let mut bytes = [0; 8];
                    loop {
                        let Err(error) = aprd(
                            &mut self.port,
                            auto_inrecment_address,
                            EthercatRegister::EepromData.into(),
                            &mut bytes,
                            TIMEOUT_RETURN,
                        ) else {
                            return Ok(Ethercat::<u64>::from_bytes(bytes));
                        };
                        if retry_count < DEFAULT_RETRIES {
                            retry_count += 1;
                        } else {
                            return Err(error.into());
                        }
                    }
                } else {
                    let mut bytes = [0; 4];
                    loop {
                        let Err(error) = aprd(
                            &mut self.port,
                            auto_inrecment_address,
                            EthercatRegister::EepromData.into(),
                            &mut bytes,
                            TIMEOUT_RETURN,
                        ) else {
                            return Ok(Ethercat::from_host(u64::from(
                                Ethercat::<u32>::from_bytes(bytes).to_host(),
                            )));
                        };
                        if retry_count < DEFAULT_RETRIES {
                            retry_count += 1;
                        } else {
                            return Err(error.into());
                        }
                    }
                }
            }
            if nack_count >= 3 {
                return Err(MainError::MissedTooManyAcks);
            }
        }
    }

    /// Write EEPROM to slave bypassing cache. APWR method.
    ///
    /// # Parameters
    /// `self`: Context struct
    /// `auto_indement_address`: Configured address of slave
    /// `eeprom_address`: Address in EEPROM
    /// `data`: Data
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - The eeprom is too busy
    /// - Error bits couldn't be cleared
    /// - Eeprom data couldn't be send
    /// - Eeprom command couldn't be send
    ///
    /// # Returns
    /// Error or ok
    pub fn write_eeprom_ap(
        &mut self,
        auto_inrecment_address: u16,
        eeprom_address: u16,
        data: Ethercat<u16>,
        timeout: Duration,
    ) -> Result<(), MainError> {
        let mut eeprom_state = 0;
        if !self.eeprom_wait_not_busy_ap(auto_inrecment_address, &mut eeprom_state, timeout)? {
            return Err(MainError::EepromBusy);
        }

        // Clear error bits, if they are set
        if eeprom_state & EEPROM_STATE_MACHINE_ERROR_MASK != 0 {
            apwr(
                &mut self.port,
                auto_inrecment_address,
                EthercatRegister::EepromControlStat.into(),
                &Ethercat::from_raw(u16::from(EepromCommand::Nop)).to_bytes(),
                TIMEOUT_RETURN3,
            )?;
        }

        let mut no_ack = 0;
        loop {
            let mut retry_counter = 0;
            loop {
                let Err(err) = apwr(
                    &mut self.port,
                    auto_inrecment_address,
                    EthercatRegister::EepromData.into(),
                    &data.to_bytes(),
                    TIMEOUT_RETURN,
                ) else {
                    break;
                };
                if retry_counter < DEFAULT_RETRIES {
                    retry_counter += 1;
                } else {
                    return Err(err.into());
                }
            }

            let eeprom = Eeprom {
                command: EepromCommand::Write,
                address: eeprom_address,
                d2: 0,
            };
            let mut eeprom_bytes = [0; size_of::<Eeprom>()];
            eeprom.write_to(&mut eeprom_bytes.as_mut_slice())?;
            retry_counter = 0;
            loop {
                let Err(err) = apwr(
                    &mut self.port,
                    auto_inrecment_address,
                    EthercatRegister::EepromControlStat.into(),
                    &eeprom_bytes,
                    TIMEOUT_RETURN,
                ) else {
                    break;
                };
                if retry_counter < DEFAULT_RETRIES {
                    retry_counter += 1;
                } else {
                    return Err(err.into());
                }
            }
            thread::sleep(2 * LOCAL_DELAY);
            eeprom_state = 0;

            if !self.eeprom_wait_not_busy_ap(auto_inrecment_address, &mut eeprom_state, timeout)? {
                continue;
            }
            if eeprom_state & EEPROM_STATE_MACHINE_ERROR_NACK != 0 {
                no_ack += 1;
                thread::sleep(LOCAL_DELAY * 5);
            } else {
                return Ok(());
            }
            if no_ack >= 3 {
                return Err(MainError::MissedTooManyAcks);
            }
        }
    }

    fn eeprom_wait_not_busy_fp(
        &mut self,
        config_address: u16,
        eeprom_state: &mut u16,
        timeout: Duration,
    ) -> Result<bool, NicdrvError> {
        let timer = OsalTimer::new(timeout);

        let mut retry_count = 0;
        loop {
            if retry_count != 0 {
                thread::sleep(LOCAL_DELAY);
            }
            retry_count += 1;

            let mut word = [0; 2];
            let Err(err) = fprd(
                &mut self.port,
                config_address,
                EthercatRegister::EepromControlStat.into(),
                &mut word,
                TIMEOUT_RETURN,
            ) else {
                *eeprom_state = Ethercat::<u16>::from_bytes(word).to_host();
                if *eeprom_state & EEPROM_STATE_MACHINE_BUSY == 0 || timer.is_expired() {
                    return Ok(*eeprom_state & EEPROM_STATE_MACHINE_BUSY == 0);
                }
                continue;
            };
            if timer.is_expired() {
                return Err(err);
            }
        }
    }

    /// Read EEPROM from slave bypassing cache. Fixed Pointer Read method.
    ///
    /// # Parameters
    /// `self`: Context struct
    /// `config_address`: Configured address of slave
    /// `eeprom_address`: Address in the EEPROM
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - Failed to check whether the eeprom is busy
    /// - Eeprom is busy
    /// - Failed to clear error bits
    /// - Failed to send EEPROM command
    /// - Failed to read data
    ///
    /// # Returns
    /// EEPROM data 64-bit or 32-bit
    pub fn read_eeprom_fp(
        &mut self,
        config_address: u16,
        eeprom_address: u16,
        timeout: Duration,
    ) -> Result<Ethercat<u64>, MainError> {
        let mut eeprom_state = 0;
        if !self.eeprom_wait_not_busy_fp(config_address, &mut eeprom_state, timeout)? {
            return Err(MainError::EepromBusy);
        }

        // Clear error bits if set
        if eeprom_address & EEPROM_STATE_MACHINE_ERROR_MASK != 0 {
            fpwr(
                &mut self.port,
                config_address,
                EthercatRegister::EepromControlStat.into(),
                &Ethercat::from_raw(u16::from(EepromCommand::Nop)).to_bytes(),
                TIMEOUT_RETURN3,
            )?;
        }

        let mut no_ack_counter = 0;
        loop {
            let eeprom = Eeprom {
                command: EepromCommand::Read,
                address: eeprom_address,
                d2: 0,
            };
            let mut eeprom_bytes = [0; size_of::<Eeprom>()];
            eeprom.write_to(&mut eeprom_bytes.as_mut_slice())?;

            let mut retry_count = 0;
            loop {
                let Err(err) = fpwr(
                    &mut self.port,
                    config_address,
                    EthercatRegister::EepromControlStat.into(),
                    &eeprom_bytes,
                    TIMEOUT_RETURN,
                ) else {
                    break;
                };

                if retry_count >= DEFAULT_RETRIES {
                    return Err(err.into());
                }
                retry_count += 1;
            }

            thread::sleep(LOCAL_DELAY);
            eeprom_state = 0;
            if !self.eeprom_wait_not_busy_fp(config_address, &mut eeprom_state, timeout)? {
                continue;
            }

            // If no ACK was received
            if eeprom_state & EEPROM_STATE_MACHINE_ERROR_NACK == 0 {
                no_ack_counter += 1;
                thread::sleep(LOCAL_DELAY * 5);
                if no_ack_counter >= 3 {
                    return Err(MainError::MissedTooManyAcks);
                }
            } else if eeprom_state & EEPROM_STATE_MACHINE_READ64 != 0 {
                retry_count = 0;
                loop {
                    let mut bytes = [0; 8];
                    let Err(error) = fprd(
                        &mut self.port,
                        config_address,
                        EthercatRegister::EepromData.into(),
                        &mut bytes,
                        TIMEOUT_RETURN,
                    ) else {
                        return Ok(Ethercat::<u64>::from_bytes(bytes));
                    };
                    if retry_count >= DEFAULT_RETRIES {
                        return Err(error.into());
                    }
                    retry_count += 1;
                }
            } else {
                retry_count = 0;
                loop {
                    let mut bytes = [0; 4];
                    let Err(error) = fprd(
                        &mut self.port,
                        config_address,
                        EthercatRegister::EepromData.into(),
                        &mut bytes,
                        TIMEOUT_RETURN,
                    ) else {
                        return Ok(Ethercat::from_host(u64::from(
                            Ethercat::<u32>::from_bytes(bytes).to_host(),
                        )));
                    };
                    if retry_count >= DEFAULT_RETRIES {
                        return Err(error.into());
                    }
                    retry_count += 1;
                }
            }
        }
    }

    /// Write EEPROM to slave bypassing cache. Fixed pointer write method.
    ///
    /// # Parameter
    /// `context`: Context struct
    /// `config_address`: Configured address of slave
    /// `eeprom_address`: Address in the EEPROM
    /// `data`: 16-bit data
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - Failed to check whether the EEPROM is busy
    /// - EEPROM is busy
    /// - Failed to clear error bits
    /// - Failed to send EEPROM command
    /// - Failed to send data
    ///
    /// # Returns
    /// `Ok(())` or error
    pub fn write_eeprom_fp(
        &mut self,
        config_address: u16,
        eeprom_address: u16,
        data: Ethercat<u16>,
        timeout: Duration,
    ) -> Result<(), MainError> {
        let mut eeprom_state = 0;
        if !self.eeprom_wait_not_busy_fp(config_address, &mut eeprom_state, timeout)? {
            return Err(MainError::EepromBusy);
        }

        // Clear error bits if set
        if eeprom_state & EEPROM_STATE_MACHINE_ERROR_MASK != 0 {
            fpwr(
                &mut self.port,
                config_address,
                EthercatRegister::EepromControlStat.into(),
                &Ethercat::from_raw(u16::from(EepromCommand::Nop)).to_bytes(),
                TIMEOUT_RETURN3,
            )?;
        }

        let mut no_ack_count = 0;
        loop {
            let mut retry_count = 0;
            loop {
                let Err(error) = fpwr(
                    &mut self.port,
                    config_address,
                    EthercatRegister::EepromData.into(),
                    &data.to_bytes(),
                    TIMEOUT_RETURN,
                ) else {
                    break;
                };
                if retry_count >= DEFAULT_RETRIES {
                    return Err(error.into());
                }
                retry_count += 1;
            }

            let eeprom = Eeprom {
                command: EepromCommand::Write,
                address: eeprom_address,
                d2: 0,
            };
            let mut eeprom_bytes = [0; size_of::<Eeprom>()];
            eeprom.write_to(&mut eeprom_bytes.as_mut_slice())?;

            retry_count = 0;
            loop {
                let Err(error) = fpwr(
                    &mut self.port,
                    config_address,
                    EthercatRegister::EepromControlStat.into(),
                    &eeprom_bytes,
                    TIMEOUT_RETURN,
                ) else {
                    break;
                };
                if retry_count >= DEFAULT_RETRIES {
                    return Err(error.into());
                }
                retry_count += 1;
            }

            thread::sleep(LOCAL_DELAY * 2);
            eeprom_state = 0;
            if !self.eeprom_wait_not_busy_fp(config_address, &mut eeprom_state, timeout)? {
                continue;
            }
            if eeprom_state & EEPROM_STATE_MACHINE_ERROR_NACK != 0 {
                no_ack_count += 1;
                thread::sleep(LOCAL_DELAY * 5);
                if no_ack_count >= 3 {
                    return Err(MainError::MissedTooManyAcks);
                }
            } else {
                return Ok(());
            }
        }
    }
}

/// An eeprom request struct, containing information required to perform EEPROM requests in
/// parallel.
pub struct EepromRequest {
    slave: u16,
}

impl EepromRequest {
    pub const fn slave(&self) -> u16 {
        self.slave
    }

    /// parallel read EEPROM from slave bypassing cache.
    /// This function sends the request to the slave.
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave number
    /// `eeprom_address`: Address in EEPROM
    ///
    /// # Errors
    /// Returns an error if:
    /// - Eeprom control can't be taken by the master
    /// - Failed to check whether the eeprom is busy
    /// - Eeprom is busy
    /// - Failed to clear error bits
    /// - Failed to send Eeprom command
    ///
    /// # Returns
    /// `EepromRequest` which can later be used to retrieve the result.
    pub fn request_eeprom_data(
        context: &mut Context,
        slave: u16,
        eeprom_address: SiiGeneralItem,
    ) -> Result<Self, MainError> {
        // Set eeprom control to master
        context.eeprom_to_master(slave)?;
        let config_address = context.slavelist[usize::from(slave)].config_address;

        let mut eeprom_state = 0;
        if !context.eeprom_wait_not_busy_fp(config_address, &mut eeprom_state, TIMEOUT_EEPROM)? {
            return Err(MainError::EepromBusy);
        }

        // Clear error bits, if set
        if eeprom_state & EEPROM_STATE_MACHINE_ERROR_MASK != 0 {
            fpwr(
                &mut context.port,
                config_address,
                EthercatRegister::EepromControlStat.into(),
                &Ethercat::from_host(u16::from(EepromCommand::Nop)).to_bytes(),
                TIMEOUT_RETURN3,
            )?;
        }

        let eeprom = Eeprom {
            command: EepromCommand::Read,
            address: eeprom_address.into(),
            d2: 0,
        };
        let mut eeprom_bytes = [0; size_of::<Eeprom>()];
        eeprom.write_to(&mut eeprom_bytes.as_mut_slice())?;

        let mut retry_counter = 0;
        loop {
            let Err(error) = fpwr(
                &mut context.port,
                config_address,
                EthercatRegister::EepromControlStat.into(),
                &eeprom_bytes,
                TIMEOUT_RETURN,
            ) else {
                return Ok(Self { slave });
            };

            if retry_counter >= DEFAULT_RETRIES {
                return Err(error.into());
            }
            retry_counter += 1;
        }
    }

    /// Parallel read EEPROM from slave bypassing cache.
    /// Executes the actual read from the slave.
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave number
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - Failed to check if the eeprom is busy
    /// - Eeprom is busy
    /// - Failed to read data
    ///
    /// # Returns
    /// 32-bit EEPROM data
    pub fn read_eeprom_data(
        self,
        context: &mut Context,
        timeout: Duration,
    ) -> Result<Ethercat<u32>, MainError> {
        let config_address = context.slavelist[usize::from(self.slave)].config_address;
        let mut eeprom_state = 0;
        if !context.eeprom_wait_not_busy_fp(config_address, &mut eeprom_state, timeout)? {
            return Err(MainError::EepromBusy);
        }

        let mut retries = 0;
        loop {
            let mut data = [0; 4];
            let Err(error) = fprd(
                &mut context.port,
                config_address,
                EthercatRegister::EepromData.into(),
                &mut data,
                TIMEOUT_RETURN,
            ) else {
                return Ok(Ethercat::<u32>::from_bytes(data));
            };
            if retries >= DEFAULT_RETRIES {
                return Err(error.into());
            }
            retries += 1;
        }
    }
}

/// Emergency request structure
struct EmergencyRequest {
    mailbox_header: MailboxHeader,
    can_open: Ethercat<u16>,
    error_code: Ethercat<u16>,
    error_register: u8,
    byte_data: u8,
    word1: Ethercat<u16>,
    word2: Ethercat<u16>,
}

impl<R: Read> ReadFrom<R> for EmergencyRequest {
    type Err = io::Error;

    fn read_from(reader: &mut R) -> Result<Self, Self::Err> {
        let mailbox_header = MailboxHeader::read_from(reader)?;
        let can_open = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let error_code = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let error_register = u8::read_from(reader)?;
        let byte_data = u8::read_from(reader)?;
        let word1 = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        let word2 = Ethercat::<u16>::from_bytes(<[u8; 2]>::read_from(reader)?);
        Ok(Self {
            mailbox_header,
            can_open,
            error_code,
            error_register,
            byte_data,
            word1,
            word2,
        })
    }
}

pub struct ProcessDataRequest;

impl ProcessDataRequest {
    /// Transmit processdata to slaves.
    /// Uses Logical read/write, or logical read followed by logical write if logical read/write is not allowed (blockLRW).
    /// Both the input and output processdata are transmitted.
    /// The outputs with the actual data, the inputs have a plcaeholder.
    /// The inputs are gathered with the receive processdata function.
    /// In contrast to the base lrw function, this function is non-blocking.
    /// If the processdata doesn't fit in one datagram, multiple are used.
    /// In order to recombine thee slave response, a stack is used.
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `group`: Group number
    /// `use_overlap_io`: Flag if overlapped iomap is used
    ///
    /// # Errors
    /// Returns an error if:
    /// - No inputs and no outputs were found in slave group
    /// - Failed to transmit the read IO request
    ///
    /// # Returns
    /// Ok if processdata is transmitted without error, error otherwise
    fn main_send_processdata(
        context: &mut Context,
        group: u8,
        use_overlap_io: bool,
    ) -> Result<Self, MainError> {
        let group_usize = usize::from(group);
        let mut first = context.grouplist[group_usize].has_distributed_clock();

        // For overlapping IO map, use the biggest
        let (mut length, mut io_map_input_offset) = if use_overlap_io {
            // For overlap IOmap make the frame EQ big to the biggest part
            (
                context.grouplist[group_usize]
                    .output_bytes
                    .max(context.grouplist[group_usize].input_bytes),
                // Save the offset used to compensate where to save inputs when frame returns
                context.grouplist[group_usize].output_bytes,
            )
        } else {
            (
                context.grouplist[group_usize].output_bytes
                    + context.grouplist[group_usize].input_bytes,
                0,
            )
        };

        if length == 0 {
            return Err(MainError::NoIOFound);
        }

        let mut logical_address = context.grouplist[group_usize].logical_start_address;
        // Check if logical read/write is blocked by one or more slaves.
        if context.grouplist[group_usize].block_logical_read_write > 0 {
            // If inputs available, generate logical read
            if context.grouplist[group_usize].input_bytes > 0 {
                let mut current_segment = context.grouplist[group_usize].first_input_segment;
                let mut data = context.grouplist[group_usize].inputs();
                let mut length = context.grouplist[group_usize].input_bytes;
                logical_address += context.grouplist[group_usize].output_bytes;

                // Segment transfer if needed
                loop {
                    let sublength =
                        if current_segment == context.grouplist[group_usize].first_input_segment {
                            context.grouplist[group_usize].io_segments[usize::from(current_segment)]
                                - u32::from(context.grouplist[group_usize].input_offset)
                        } else {
                            context.grouplist[group_usize].io_segments[usize::from(current_segment)]
                        };
                    current_segment += 1;

                    // Get new index
                    let index = context.port.get_index();
                    let word1 = low_word(logical_address);
                    let word2 = high_word(logical_address);
                    setup_datagram(
                        &mut context.port.stack_mut().tx_buffers_mut()[usize::from(index)],
                        ReadCommand::LogicalRead.into(),
                        index,
                        word1,
                        word2,
                        &data[..sublength as usize],
                    )?;
                    let dc_offset = if first {
                        first = false;
                        // Fixed Pointer Read Multiple Write in second datagram
                        let config_address = context
                            .get_slave(context.grouplist[group_usize].dc_next.unwrap())
                            .config_address;
                        add_datagram(
                            &mut context.port.stack_mut().tx_buffers_mut()[usize::from(index)],
                            ReadCommand::LogicalRead.into(),
                            index,
                            false,
                            config_address,
                            EthercatRegister::DistributedClockSystemTime.into(),
                            &Ethercat::from_host(context.dc_time).to_bytes(),
                        ) as u16
                    } else {
                        0
                    };

                    // Send frame
                    context.port.out_frame_red(index)?;

                    // Push index and data pointer on stack
                    context.index_stack.push_index(index, data, dc_offset);
                    length -= sublength;
                    logical_address += sublength;
                    data = &data[sublength as usize..];

                    if length == 0
                        || current_segment >= context.grouplist[group_usize].used_segment_count
                    {
                        break;
                    }
                }
            }

            // If outputs available generate logical write
            if context.grouplist[group_usize].output_bytes != 0 {
                let mut data = context.grouplist[group_usize].io_map();
                let mut length = context.grouplist[group_usize].output_bytes;
                let mut logical_address = context.grouplist[group_usize].logical_start_address;
                let mut current_segment = 0;

                // Segment transfer if needed
                loop {
                    let sub_length =
                        context.grouplist[group_usize].io_segments[current_segment].min(length);
                    current_segment += 1;

                    // Get new index
                    let index = context.port.get_index();
                    let word1 = low_word(logical_address);
                    let word2 = high_word(logical_address);

                    setup_datagram(
                        &mut context.port.stack_mut().tx_buffers_mut()[usize::from(index)],
                        ReadCommand::FixedReadMultipleWrite.into(),
                        index,
                        word1,
                        word2,
                        &data[..sub_length as usize],
                    )?;
                    let dc_offset = if first {
                        first = false;

                        // Fixed Pointer Read Multiple Write in second datagram
                        add_datagram(
                            &mut context.port.stack_mut().tx_buffers_mut()[usize::from(index)],
                            ReadCommand::FixedReadMultipleWrite.into(),
                            index,
                            false,
                            context.slavelist[usize::from(
                                context.grouplist[usize::from(group)].dc_next.unwrap(),
                            )]
                            .config_address,
                            EthercatRegister::DistributedClockSystemTime.into(),
                            &Ethercat::from_host(context.dc_time).to_bytes(),
                        ) as u16
                    } else {
                        0
                    };

                    // Send frame
                    context.port.out_frame_red(index)?;

                    // Push index and data pointer on stack
                    context.index_stack.push_index(index, data, dc_offset);

                    length -= sub_length;
                    logical_address += sub_length;
                    data = &data[sub_length as usize..];

                    if length == 0
                        || current_segment
                            >= usize::from(context.grouplist[group_usize].used_segment_count)
                    {
                        break;
                    }
                }
            }
        } else {
            // Logical read/write can be used
            let mut data = if context.grouplist[group_usize].output_bytes == 0 {
                // Clear offset, don't compensate for overlapping IOmap if there only are inputs
                io_map_input_offset = 0;

                context.grouplist[group_usize].inputs()
            } else {
                context.grouplist[group_usize].outputs()
            };

            // Segment transfer if needed
            let mut current_segment = 0;
            loop {
                let sub_length = context.grouplist[group_usize].io_segments[current_segment];
                current_segment += 1;

                // Get new index
                let index = context.port.get_index();
                let word1 = low_word(logical_address);
                let word2 = high_word(logical_address);

                setup_datagram(
                    &mut context.port.stack_mut().tx_buffers_mut()[usize::from(index)],
                    ReadCommand::LogicalReadWrite.into(),
                    index,
                    word1,
                    word2,
                    &data[..sub_length as usize],
                )?;

                let dc_offset = if first {
                    first = false;

                    //Fixed Pointer Read Multiple Write in second datagram
                    add_datagram(
                        &mut context.port.stack_mut().tx_buffers_mut()[usize::from(index)],
                        ReadCommand::FixedReadMultipleWrite.into(),
                        index,
                        false,
                        context.slavelist
                            [usize::from(context.grouplist[group_usize].dc_next.unwrap())]
                        .config_address,
                        EthercatRegister::DistributedClockSystemTime.into(),
                        &Ethercat::from_host(context.dc_time).to_bytes(),
                    ) as u16
                } else {
                    0
                };

                // Send frame
                context.port.out_frame_red(index)?;

                // Push index and pointer on stack.
                // The `iomap_input_offset` compensate for where the inputs are stored
                // in the IOmap if we use an overlapping IOmap. If a regular IOmap is
                // used, it should always be 0.
                context.index_stack.push_index(
                    index,
                    &data[io_map_input_offset as usize..],
                    dc_offset,
                );
                length -= sub_length;
                logical_address += sub_length;
                data = &data[sub_length as usize..];

                if length == 0
                    || current_segment
                        >= usize::from(context.grouplist[group_usize].used_segment_count)
                {
                    break;
                }
            }
        }
        Ok(Self)
    }

    /// Transmit processdata to slaves.
    /// Uses logical read/write or logical read + logical write if logical read/write is not
    /// allowed (block logical read write).
    /// Both the input and output processdata are transmitted in the overlapped IOmap.
    /// The outputs with the actual data, the inputs replace the output data in the returning
    /// frame. The inputs are gathered with the receive processdata function.
    /// In contrast to the base lrw function, this function is non-blocking.
    /// If the processdata doesn't fit in one datagram, multiple are used.
    /// In order to recombine the slave response, a stack is used.
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `group`: Group number
    ///
    /// # Errors
    /// Returns an error if:
    /// - No inputs and no outputs were found in slave group
    /// - Failed to transmit read IO request
    ///
    /// # Returns
    /// `ProcessDataRequest` if processdata is transmitted, error otherwise
    pub fn send_overlap_processdata_group(
        context: &mut Context,
        group: u8,
    ) -> Result<Self, MainError> {
        Self::main_send_processdata(context, group, true)
    }

    /// Transmit processdata to slaves.
    /// Uses logical read/write or logical read + logical write if logical read/write is not
    /// allowed (block logical read/write).
    /// Both the input and output processdata are transmitted.
    /// The outputs with the actual data, the inputs have a placeholder.
    /// The inputs are gathered with the receive processdata function.
    /// In contrast to the logical read/write function, this function is non-blocking.
    /// If the processdata doesn't fit in one datagram, multiple are used.
    /// In order to recombine the slave response, a stack is used.
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `group`: Group number
    ///
    /// # Errors
    /// Returns an error if:
    /// - No inputs and no outputs were found in slave group
    /// - Failed to transmit read IO request
    ///
    /// # Returns
    /// `ProcessDataRequest` if processdata is transmitted, error otherwise
    pub fn send_processdata_group(context: &mut Context, group: u8) -> Result<Self, MainError> {
        Self::main_send_processdata(context, group, false)
    }

    /// # Errors
    /// Returns an error if:
    /// - No inputs and no outputs were found
    /// - Failed to transmit read IO request
    pub fn send_processdata(context: &mut Context) -> Result<Self, MainError> {
        Self::send_processdata_group(context, 0)
    }

    /// # Errors
    /// Returns an error if:
    /// - No inputs and no outputs were found
    /// - Failed to transmit read IO request
    pub fn send_overlap_processdata(context: &mut Context) -> Result<Self, MainError> {
        Self::send_overlap_processdata_group(context, 0)
    }

    /// Receive processdata from slaves.
    /// Second part of `Self::send_processdata`.
    /// Received datagrams are recombined with the processdata with help from the stack.
    /// If a datagram contains input processdata, it copies it to the processdata structure.
    ///
    /// # Parameters
    /// `self`: ProcessDataRequest struct containing the group number
    /// `context`: Context struct
    /// `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - No frame was received
    ///
    /// # Returns workcounter
    pub fn receive_processdata_group(
        self,
        context: &mut Context,
        timeout: Duration,
    ) -> Result<u16, MainError> {
        let mut wkc2;
        let mut wkc = 0;
        let mut valid_work_counter = false;

        while let Some(position) = context.index_stack.pull_index() {
            let position_usize = usize::from(position);
            let index = context.index_stack.buffers[position_usize].index;
            wkc2 = context.port.wait_in_frame(index, timeout);

            // Check if the frame contains input data
            let Ok(work_counter2_value) = wkc2 else {
                continue;
            };

            let command_type = ReadCommand::try_from(
                context.port.stack_mut().rx_buffers_mut()[usize::from(index)].data()
                    [ETHERCAT_COMMAND_OFFET],
            );

            let index_usize = usize::from(index);
            if matches!(
                command_type,
                Ok(ReadCommand::LogicalRead | ReadCommand::LogicalReadWrite)
            ) {
                if context.index_stack.buffers[position_usize].dc_offset != 0 {
                    context.index_stack.buffers[position_usize].data.clear();
                    context.index_stack.buffers[position_usize]
                        .data
                        .clone_from_slice(
                            &context.port.stack().rx_buffers()[index_usize].data()
                                [EthercatHeader::size()..],
                        );
                    let mut word = [0; 2];
                    word.copy_from_slice(
                        &context.port.stack().rx_buffers()[index_usize].data()[EthercatHeader::size(
                        )
                            + context.index_stack.buffers[position_usize].data.len()..],
                    );
                    wkc = Ethercat::<u16>::from_bytes(word).to_host();
                    let mut long_word = [0; 8];
                    long_word.copy_from_slice(
                        &context.port.stack().rx_buffers()[index_usize].data()
                            [usize::from(context.index_stack.buffers[position_usize].dc_offset)..],
                    );
                    context.dc_time = Ethercat::<i64>::from_bytes(long_word).to_host();
                } else {
                    // Copy input data back to process data buffer
                    context.index_stack.buffers[position_usize].data.clear();
                    context.index_stack.buffers[position_usize]
                        .data
                        .clone_from_slice(
                            &context.port.stack().rx_buffers()[index_usize].data()
                                [EthercatHeader::size()..],
                        );
                    wkc += work_counter2_value;
                }
                valid_work_counter = true;
            } else if context.port.stack().rx_buffers()[usize::from(index)].data()
                [ETHERCAT_COMMAND_OFFET]
                == u8::from(WriteCommand::LogicalWrite)
            {
                if context.index_stack.buffers[usize::from(position)].dc_offset != 0 {
                    let mut word = [0; ETHERCAT_WORK_COUNTER_SIZE];
                    word.copy_from_slice(
                        &context.port.stack().rx_buffers()[usize::from(index)].data()
                            [EthercatHeader::size()
                                + context.index_stack.buffers[usize::from(position)]
                                    .data
                                    .len()..],
                    );

                    // Output WKC counts 2 times when using logical read write, emulate the same
                    // for logical write.
                    wkc = Ethercat::<u16>::from_bytes(word).to_host() * 2;
                    let mut long = [0; 8];
                    long.copy_from_slice(
                        &context.port.stack().rx_buffers()[usize::from(index)].data()
                            [usize::from(
                                context.index_stack.buffers[usize::from(position)].dc_offset,
                            )..],
                    );
                    context.dc_time = Ethercat::<i64>::from_bytes(long).to_host();
                } else {
                    // Output WKC counts 2 times when using Logical Read/Write, emulate the same
                    // for Logical Write.
                    wkc += wkc2? * 2;
                }
                valid_work_counter = true;
            }

            // Release buffer
            context
                .port
                .set_buf_stat(usize::from(index), BufferState::Empty);
        }

        context.index_stack.clear_index();

        // If no frame has arrived
        if valid_work_counter {
            Ok(wkc)
        } else {
            Err(MainError::NoFrame)
        }
    }

    /// # Errors
    /// Returns an error if:
    /// - No frame was received
    pub fn receive_processdata(
        self,
        context: &mut Context,
        timeout: Duration,
    ) -> Result<u16, MainError> {
        self.receive_processdata_group(context, timeout)
    }
}

fn fixed_pointer_read_multi(
    context: &mut Context,
    number: u16,
    config_list: &mut [u16],
    sl_status_list: &mut [ApplicationLayerStatus],
    timeout: Duration,
) -> Result<(), MainError> {
    let port = &mut context.port;
    let index = port.get_index();
    let mut sl_count: u16 = 0;
    setup_datagram(
        &mut port.stack_mut().tx_buffers_mut()[usize::from(index)],
        ReadCommand::FixedPointerRead.into(),
        index,
        config_list[usize::from(sl_count)],
        EthercatRegister::ApplicationLayerStatus.into(),
        &sl_status_list[usize::from(sl_count)].bytes(),
    )?;
    let mut sl_data_position = [0; MAX_FIXED_POINTER_READ_MULTI];
    sl_data_position[usize::from(sl_count)] = EthercatHeader::size();
    sl_count += 1;
    for sl_count in sl_count..number - 1 {
        sl_data_position[usize::from(sl_count)] = add_datagram(
            &mut port.stack_mut().tx_buffers_mut()[usize::from(index)],
            ReadCommand::FixedPointerRead.into(),
            index,
            true,
            config_list[usize::from(sl_count)],
            EthercatRegister::ApplicationLayerStatus.into(),
            &sl_status_list[usize::from(sl_count)].bytes(),
        );
    }
    sl_count = sl_count.max(number - 1);
    if sl_count < number {
        sl_data_position[usize::from(sl_count)] = add_datagram(
            &mut port.stack_mut().tx_buffers_mut()[usize::from(index)],
            ReadCommand::FixedPointerRead.into(),
            index,
            false,
            config_list[usize::from(sl_count)],
            EthercatRegister::ApplicationLayerStatus.into(),
            &sl_status_list[usize::from(sl_count)].bytes(),
        );
    }
    port.src_confirm(index, timeout)?;
    for sl_count in 0..number {
        sl_status_list[usize::from(sl_count)] = ApplicationLayerStatus::read_from(
            &mut &port.stack().rx_buffers()[usize::from(index)]
                .data()
                .as_slice()[sl_data_position[usize::from(sl_count)]..],
        )?;
    }

    port.set_buf_stat(usize::from(index), BufferState::Empty);
    Ok(())
}
