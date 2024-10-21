use std::{
    any::Any,
    array,
    io::{self, Read, Write},
    sync::{Arc, Mutex},
    time::Duration,
};

use bytemuck::{AnyBitPattern, NoUninit, Zeroable};
use heapless::String as HeaplessString;
use oshw::nicdrv::{Port, RedPort};

use super::r#type::{
    Error, ErrorInfo, EthercatState, MailboxError, SiiCategory, SiiGeneralItem, MAX_BUF_COUNT,
};
use crate::oshw;

/// Max. entries in EtherCAT error list
pub const MAX_E_LIST_ENTRIES: usize = 64;

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

/// EtherCAT Adapter
pub struct EcAdapter {
    pub name: HeaplessString<MAX_ADAPTER_NAME_LENGTH>,
    pub desc: HeaplessString<MAX_ADAPTER_NAME_LENGTH>,
}

/// Fieldbus Memory Management Unit
#[derive(Debug, Clone, Copy)]
pub struct Fmmu {
    pub log_start: u32,
    pub log_length: u16,
    pub log_start_bit: u8,
    pub log_end_bit: u8,
    pub physical_start: u16,
    pub physical_start_bit: u8,
    pub fmmu_type: u8,
    pub fmmu_active: u8,
    pub unused: u8,
    pub unused2: u16,
}

#[allow(unsafe_code)]
unsafe impl NoUninit for Fmmu {}

#[allow(unsafe_code)]
unsafe impl Zeroable for Fmmu {}

#[allow(unsafe_code)]
unsafe impl AnyBitPattern for Fmmu {}

impl<'fmmu> From<&'fmmu mut Fmmu> for &'fmmu mut [u8] {
    fn from(value: &'fmmu mut Fmmu) -> Self {
        bytemuck::bytes_of_mut(value)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SyncManager {
    pub start_address: u16,
    pub sm_length: u16,
    pub sm_flags: u32,
}

#[allow(unsafe_code)]
unsafe impl NoUninit for SyncManager {}

#[allow(unsafe_code)]
unsafe impl Zeroable for SyncManager {}

#[allow(unsafe_code)]
unsafe impl AnyBitPattern for SyncManager {}

impl<'sm> From<&'sm mut SyncManager> for &'sm mut [u8] {
    fn from(value: &'sm mut SyncManager) -> Self {
        bytemuck::bytes_of_mut(value)
    }
}

pub struct StateStatus {
    pub state: u16,
    pub unused: u16,
    pub al_status_code: u16,
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
        value as u8
    }
}

impl From<MailboxProtocol> for u16 {
    fn from(value: MailboxProtocol) -> Self {
        value as u16
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
        value as u8
    }
}

pub const SYNC_MANAGER_ENABLE_MASK: u32 = 0xFFFE_FFFF;

#[derive(Debug, Clone, Copy)]
pub struct InvalidSyncManagerType(u8);

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
    type Error = InvalidSyncManagerType;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Unused),
            1 => Ok(Self::MailboxWrite),
            2 => Ok(Self::MailboxRead),
            3 => Ok(Self::Outputs),
            4 => Ok(Self::Inputs),
            _ => Err(InvalidSyncManagerType(value)),
        }
    }
}

/// Amount of data per read from EEProm
pub enum EepReadSize {
    Bytes4,
    Bytes8,
}

/// Detected EtherCAT slave
pub struct Slave<'slave> {
    /// State of slave
    pub state: u16,

    /// Application layer status code
    pub al_status_code: u16,

    /// Configured address
    pub config_address: u16,

    /// Alias address
    pub alias_address: u16,

    /// Manufacturer from EEprom
    pub eep_manufacturer: u32,

    /// EEProm ID
    pub eep_id: u32,

    /// EEProm revision
    pub eep_revision: u32,

    pub interface_type: u16,
    pub output_bits: u16,

    /// Output bytes, if output_bits < 8, output_bytes = 0
    pub output_bytes: u16,

    /// Output IOmap buffer
    pub outputs: &'slave [u8],

    /// Startbit in first output byte
    pub output_startbit: u8,

    pub input_bits: u16,

    /// Input bytes, if input_bits < 8, output_bytes = 0
    pub input_bytes: u16,

    /// Input IOmap buffer
    pub input: Option<&'slave [u8]>,

    /// Startbit in IOmap buffer
    pub input_startbit: u8,

    pub sync_manager: [SyncManager; MAX_SM as usize],
    pub sync_manager_type: [SyncManagerType; MAX_SM as usize],

    /// Fieldbus Memory Management Units
    pub fmmu: [Fmmu; MAX_FMMU],

    /// Fieldbus Memory Management Unit 0 function
    pub fmmu0_function: u8,

    /// Fieldbus Memory Management Unit 1 function
    pub fmmu1_function: u8,

    /// Fieldbus Memory Management Unit 2 function
    pub fmmu2_function: u8,

    /// Fieldbus Memory Management Unit 3 function
    pub fmmu3_function: u8,

    /// Length of write mailbox in bytes, 0 if no mailbox
    pub mailbox_length: u16,

    pub mailbox_write_offset: u16,

    // Length of read mailbox in bytes
    pub mailbox_read_length: u16,
    pub mailbox_read_offset: u16,

    /// Supported mailbox protocols
    pub mailbox_protocols: u16,

    /// Counter value of mailbox link layer protocol 1..7
    pub mailbox_count: u8,

    /// Has DC capability
    pub has_dc: bool,

    /// Physical type: Ebus Ethernet combinations
    pub physical_type: u8,

    /// Topology: 1 to 3 links
    pub topology: u8,

    /// Active ports bitmap: ....3210, set if respective port is active
    pub active_ports: u8,

    /// Consumed ports bitmap: ...3210, used for internal delay measurement
    pub consumed_ports: u8,

    /// Slave number for parent, 0=master
    pub slave_number_for_parent: u8,

    /// Port number on parent this slave is connected to
    pub parent_port: u8,

    // 0 = master
    pub parent: u8,

    /// Port number on this slave the parent is connected to
    pub entry_port: u8,

    /// DC receivetimes on port A
    pub dc_rt_a: Duration,

    /// DC receivetimes on port B
    pub dc_rt_b: Duration,

    /// DC receivetimes on port C
    pub dc_rt_c: Duration,

    /// DC receivetimes on port D
    pub dc_rt_d: Duration,

    pub propagation_delay: Duration,

    /// Next DC slave
    pub dc_next: u16,

    /// Previous DC slave
    pub dc_previous: u16,

    /// DC cycle received in nanoseconds
    pub dc_cycle: Duration,

    /// DC shift from clock modulus boundary
    pub dc_shift: i32,

    /// DC sync activation
    pub dc_active: bool,

    /// Link to config table
    pub config_index: u16,

    /// Link to SII config
    pub sii_index: u16,

    /// The amount of data per read from EEPROM
    pub eep_read_size: EepReadSize,

    /// False for eeprom to master, true for eeprom to PDI
    pub eep_pdi: bool,

    pub canopen_over_ethercat_details: u8,
    pub file_over_ethercat_details: u8,
    pub ethernet_over_ethercat_details: u8,
    pub servo_over_ethercat_details: u8,

    /// E-bus current
    pub ebus_current: u16,

    /// If > 0 block use of LRW in processdata
    pub block_logical_read_write: u8,

    pub group: u8,

    /// First unused Fieldbus Memory Management Unit
    pub fmmu_unused: u8,

    /// Whether the slave is responding, not used by the SOEM library
    pub is_lost: bool,

    /// Registered configuration function PO -> SO (DEPRECATED)
    pub po2_so_config: Option<fn(slave: u16) -> i32>,

    /// Registered configuration function PO->SO
    pub po2_so_configx: Option<fn(context: &mut Context, slave: u16) -> i32>,

    pub name: HeaplessString<{ MAX_NAME_LENGTH as usize + 1 }>,
}

///EtherCAT slave group
#[derive(Debug)]
pub struct SlaveGroup<'io_map> {
    /// Logical start address for this group
    pub logical_start_address: u32,

    /// Output bytes, 0 if output bits < 0
    pub output_bytes: u32,

    /// Output IOmap buffer
    pub outputs: &'io_map [u8],

    /// Input bytes, 0 if input bits < 8
    pub input_bytes: u32,

    /// Input IOmap buffer
    pub inputs: &'io_map [u8],

    /// Has DC capability
    pub has_dc: bool,

    /// Next DC slave
    pub dc_next: u16,

    /// E-bus current
    pub ebus_current: u16,

    /// If >0 block use of logical read write in process data
    pub block_logical_read_write: u8,

    /// Number of used IO segments
    pub used_segment_count: u16,

    pub first_input_segment: u16,
    pub input_offset: u16,

    /// Expected workcounter outputs
    pub work_counter_outputs: u16,

    /// Expected workcounter inputs
    pub work_counter_inputs: u16,

    pub check_slaves_states: bool,

    /// OI segmentation list. Datagrams must not break SM in two.
    pub io_segments: [u32; MAX_IO_SEGMENTS],
}

impl<'io_map> Default for SlaveGroup<'io_map> {
    fn default() -> Self {
        const STATIC_SLICE: &[u8] = &[];
        Self {
            logical_start_address: 0,
            output_bytes: 0,
            outputs: STATIC_SLICE,
            input_bytes: 0,
            inputs: STATIC_SLICE,
            has_dc: false,
            dc_next: 0,
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

/// Eeprom Fieldbus Memory Management Unit
#[derive(Debug, Clone, Copy)]
pub struct EepromFmmu {
    pub start_position: u16,
    pub number_fmmu: u8,
    pub fmmu: [u8; 4],
}

#[derive(Debug, Clone)]
pub struct EepromSyncManager {
    pub start_position: u16,
    pub number_sync_manager: u8,
    pub phase_start: u16,
    pub phase_length: u16,
    pub control_register: u8,

    /// Not used by SOEM
    pub slave_register: u8,

    pub activate: u8,

    /// Not used by SOEM
    pub process_data_interface_control: u8,
}

/// Eeprom Process Data Object
#[derive(Debug)]
pub struct EepromPdo {
    pub start_position: u16,
    pub length: u16,
    pub number_pdo: u16,
    pub index: [u16; MAX_EE_PDO],
    pub sync_manager: [u16; MAX_EE_PDO],
    pub bitsize: [u16; MAX_EE_PDO],
    pub sync_manager_bit_size: [u16; MAX_EE_PDO],
}

impl Default for EepromPdo {
    fn default() -> Self {
        Self {
            start_position: 0,
            length: 0,
            number_pdo: 0,
            index: [0; MAX_EE_PDO],
            sync_manager: [0; MAX_EE_PDO],
            bitsize: [0; MAX_EE_PDO],
            sync_manager_bit_size: [0; MAX_EE_PDO],
        }
    }
}

/// Mailbox buffer array
#[derive(Debug)]
pub struct MailboxIn {
    data: [u8; MAX_MAILBOX_SIZE],
    read_index: usize,
}

impl Read for MailboxIn {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if self.read_index <= MAX_MAILBOX_SIZE {
            return Err(io::Error::new(
                io::ErrorKind::UnexpectedEof,
                "Mailbox is empty",
            ));
        }
        buf.copy_from_slice(&self.data);
        let written = buf.len().min(MAX_MAILBOX_SIZE - self.read_index);
        self.read_index += written;
        Ok(written)
    }
}

impl MailboxIn {
    pub fn empty(context: &mut Context, slave: u16, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn clear(&mut self) {
        todo!()
    }

    pub fn receive(
        &mut self,
        context: &mut Context,
        slave: u16,
        timeout: Duration,
    ) -> Result<u16, MailboxError> {
        todo!()
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
            read_index: 0,
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
        let written = self.data.as_mut_slice().write(buf)?;
        self.write_index += written;
        Ok(written)
    }

    fn flush(&mut self) -> std::io::Result<()> {
        self.write_index = 0;
        Ok(())
    }
}

impl MailboxOut {
    pub fn empty(context: &mut Context, slave: u16, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn send(
        &mut self,
        context: &mut Context,
        slave: u16,
        timeout: Duration,
    ) -> Result<u16, MailboxError> {
        todo!()
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
    pub length: u16,
    pub address: u16,
    pub priority: u8,
    pub mailbox_type: u8,
}

impl MailboxHeader {
    pub fn write_to(&self, bytes: &mut impl Write) -> Result<usize, usize> {
        let mut bytes_written = match bytes.write(&self.length.to_ne_bytes()) {
            Ok(written) if written < 2 => return Err(written),
            Ok(written) => written,
            Err(_) => return Err(0),
        };
        match bytes.write(&self.address.to_ne_bytes()) {
            Ok(written) if written < 2 => return Err(bytes_written + written),
            Ok(written) => bytes_written += written,
            Err(_) => return Err(bytes_written),
        }
        match bytes.write(&[self.priority, self.mailbox_type]) {
            Ok(bytes) if bytes < 2 => Err(bytes + bytes_written),
            Ok(bytes) => Ok(bytes + bytes_written),
            Err(_) => Err(bytes_written),
        }
    }

    pub fn read_from<R: Read>(bytes: &mut R) -> io::Result<Self> {
        let mut value = [0; 6];
        bytes.read_exact(&mut value)?;
        Ok(Self {
            length: u16::from_ne_bytes(value[..2].try_into().unwrap()),
            address: u16::from_ne_bytes(value[2..4].try_into().unwrap()),
            priority: value[4],
            mailbox_type: value[5],
        })
    }
}

pub struct ApplicationLayerStatus {
    pub status: u16,
    pub unused: u16,
    pub code: u16,
}

/// Stack structure to store segmented logical read, logical write, logical read/write constructs
pub struct IndexStack {
    pub pushed: u8,
    pub pulled: u8,
    pub index: [u8; MAX_BUF_COUNT],
    pub data: [Box<dyn Any>; MAX_BUF_COUNT],
    pub length: [u16; MAX_BUF_COUNT],
    pub dc_offset: [u16; MAX_BUF_COUNT],
}

/// Ringbuffer for error storage
pub struct ErrorRing {
    pub head: i16,
    pub tail: i16,
    pub error: [Error; MAX_E_LIST_ENTRIES + 1],
}

/// Sync manager communication type structure for communication access
#[derive(Debug, Clone)]
pub struct SyncManagerCommunicationType {
    pub number: u8,
    pub null: u8,
    pub sync_manager_type: [SyncManagerType; MAX_SM as usize],
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
    type Error = InvalidSyncManagerType;

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
    pub number: u8,
    pub null: u8,
    pub index: [u16; 256],
}

#[allow(unsafe_code)]
unsafe impl NoUninit for PdoAssign {}

#[allow(unsafe_code)]
unsafe impl Zeroable for PdoAssign {}

#[allow(unsafe_code)]
unsafe impl AnyBitPattern for PdoAssign {}

impl<'pdo> From<&'pdo mut PdoAssign> for &'pdo mut [u8] {
    fn from(value: &'pdo mut PdoAssign) -> Self {
        bytemuck::bytes_of_mut(value)
    }
}

impl<'pdo> From<&'pdo PdoAssign> for &'pdo [u8] {
    fn from(value: &'pdo PdoAssign) -> Self {
        bytemuck::bytes_of(value)
    }
}

/// Service data object assign structure for communication access
#[derive(Debug, Clone, Copy)]
pub struct PdoDescription {
    pub number: u8,
    pub null: u8,
    pub pdo: [u32; 256],
}

#[allow(unsafe_code)]
unsafe impl NoUninit for PdoDescription {}

#[allow(unsafe_code)]
unsafe impl Zeroable for PdoDescription {}

#[allow(unsafe_code)]
unsafe impl AnyBitPattern for PdoDescription {}

impl<'pdo> From<&'pdo mut PdoDescription> for &'pdo mut [u8] {
    fn from(value: &'pdo mut PdoDescription) -> Self {
        bytemuck::bytes_of_mut(value)
    }
}

impl<'pdo> From<&'pdo PdoDescription> for &'pdo [u8] {
    fn from(value: &'pdo PdoDescription) -> Self {
        bytemuck::bytes_of(value)
    }
}

/// Context structure referenced by all Ethernet eXtended functions
pub struct Context<'context> {
    /// Port reference may include red port
    pub port: Arc<Mutex<Port<'context>>>,

    pub slavelist: Vec<Slave<'context>>,

    /// Number of slaves found in configuration
    pub slave_count: u16,

    /// Maximum nummber of slaves allowed in slavelist
    pub max_slaves: u16,

    pub grouplist: Vec<SlaveGroup<'context>>,

    /// Maximum number of groups allowed in grouplist
    pub max_group: u32,

    /// Internal reference to eeprom cache buffer
    esibuf: &'context mut [u8],

    /// internal reference to eeprom cache map
    esimap: &'context mut [u8],

    /// Internal current slave for eeprom cache
    esislave: u16,

    /// Internal reference to error list
    elist: &'context mut ErrorRing,

    /// Internal reference to processdata stack buffer info
    index_stack: &'context mut IndexStack,

    /// Reference to ecaterror state
    pub ecaterror: Arc<Mutex<bool>>,

    /// Reference to last DC time from slaves
    pub dc_time: i64,

    /// Internal Sync manager buffer
    pub sync_manager_communication_type: Vec<SyncManagerCommunicationType>,

    /// Internal pdo assign list
    pub pdo_assign: Vec<PdoAssign>,

    /// Internal pdo description list
    pub pdo_description: Vec<PdoDescription>,

    /// Internal eeprom sync manager list
    pub eep_sync_manager: EepromSyncManager,

    // Internal eeprom FMMU list
    pub eep_fmmu: EepromFmmu,

    /// Registered file over ethercat hook
    pub file_over_ethercat_hook: fn(slave: u16, packetnumber: i32, datasize: i32) -> i32,

    /// Registered Ethernet over ethercat hook
    pub ethernet_over_ethercat_hook:
        fn(context: &mut Context, slave: u16, ecembx: &mut dyn Any) -> i32,

    /// Flag to control legacy automatic state change or manual state change
    pub manual_state_change: bool,

    /// Userdata promotes application configuration especially in EC_VER2 with multiple ec_context
    /// instances.
    pub userdata: Vec<u8>,
}

#[derive(Debug, Clone, Copy)]
pub enum PacketError {
    UnexpectedFrameReturned = 1,
    DataContainerTooSmallForType = 3,
    TooManySyncManagers = 10,
}

pub fn find_adapters() -> Vec<EcAdapter> {
    todo!()
}

pub fn free_adapters(adapters: Vec<EcAdapter>) {
    todo!()
}

pub fn next_mailbox_count(count: u8) -> u8 {
    todo!()
}

pub fn push_error(context: &mut Context, error: ErrorInfo) {
    todo!()
}

pub fn pop_error(context: &mut Context, error: &mut Error) -> bool {
    todo!()
}

pub fn is_error(context: &mut Context) -> bool {
    todo!()
}

pub fn packet_error(
    context: &mut Context,
    slave: u16,
    index: u16,
    sub_index: u8,
    error_code: PacketError,
) {
    todo!()
}

pub fn init(context: &mut Context, ifname: &str) -> i32 {
    todo!()
}

pub fn init_redundant(
    context: &mut Context,
    redport: &mut RedPort,
    ifname: &str,
    if2name: &mut String,
) -> i32 {
    todo!()
}

pub fn close(context: &mut Context) {
    todo!()
}

pub fn sii_get_byte(context: &mut Context, slave: u16, address: u16) -> u8 {
    todo!()
}

pub fn sii_find(context: &mut Context, slave: u16, address: SiiCategory) -> u16 {
    todo!()
}

pub fn sii_string<const STRING_SIZE: usize>(
    context: &mut Context,
    str: &mut HeaplessString<STRING_SIZE>,
    slave: u16,
    sn: u16,
) {
    todo!()
}

pub fn sii_fmmu(context: &mut Context, slave: u16, fmmu: &mut EepromFmmu) -> u16 {
    todo!()
}

pub fn sii_sm(context: &mut Context, slave: u16, sm: &mut EepromSyncManager) -> u16 {
    todo!()
}

pub fn sii_sm_next(context: &mut Context, slave: u16, sm: &mut EepromSyncManager, n: u16) -> u16 {
    todo!()
}

pub fn sii_pdo(context: &mut Context, slave: u16, pdo: &mut EepromPdo, t: u8) -> u32 {
    todo!()
}

pub fn readstate(context: &mut Context) -> i32 {
    todo!()
}

pub fn writestate(context: &mut Context) -> i32 {
    todo!()
}

pub fn statecheck(
    context: &mut Context,
    slave: u16,
    request_state: EthercatState,
    timeout: Duration,
) -> u16 {
    todo!()
}

pub fn esi_dump(context: &mut Context, slave: u16, esibuf: &mut Vec<u8>) {
    todo!()
}

pub fn read_eeprom(context: &mut Context, slave: u16, eeproma: u16, timeout: Duration) -> u32 {
    todo!()
}

pub fn write_eeprom(
    context: &mut Context,
    slave: u16,
    eeproma: u16,
    data: u16,
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn eeprom_to_master(context: &mut Context, slave: u16) -> i32 {
    todo!()
}

pub fn eeprom_to_pdi(context: &mut Context, slave: u16) -> i32 {
    todo!()
}

pub fn read_eeprom_ap(context: &mut Context, aiadr: u16, eeproma: u16, timeout: Duration) -> u64 {
    todo!()
}

pub fn write_eeprom_ap(
    context: &mut Context,
    aiadr: u16,
    eeproma: u16,
    data: u16,
    timeout: Duration,
) -> i32 {
    todo!()
}
pub fn read_eeprom_fp(
    context: &mut Context,
    config_address: u16,
    eeproma: u16,
    timeout: Duration,
) -> u64 {
    todo!()
}

pub fn write_eeprom_fp(
    context: &mut Context,
    config_address: u16,
    eeproma: u16,
    data: u16,
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn read_eeprom1(context: &mut Context, slave: u16, eeprom_address: SiiGeneralItem) {
    todo!()
}

pub fn read_eeprom2(context: &mut Context, slave: u16, timeout: Duration) -> u32 {
    todo!()
}

pub fn send_overlap_processdata_group(context: &mut Context, group: u8) -> i32 {
    todo!()
}

pub fn receive_processdata_group(context: &mut Context, group: u8, timeout: Duration) -> i32 {
    todo!()
}

pub fn send_processdata(context: &mut Context) -> i32 {
    todo!()
}

pub fn send_overlap_processdata(context: &mut Context) -> i32 {
    todo!()
}

pub fn receive_processdata(context: &mut Context, timeout: Duration) -> i32 {
    todo!()
}

pub fn send_processdata_group(context: &mut Context, group: u8) -> i32 {
    todo!()
}
