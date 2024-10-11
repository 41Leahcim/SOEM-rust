use std::{
    any::Any,
    sync::{Arc, Mutex},
    time::Duration,
};

use heapless::String as HeaplessString;
use oshw::nicdrv::{Port, RedPort};

use super::r#type::{Error, ErrorInfo, MailboxError, MAX_BUF_COUNT};
use crate::oshw;

/// Max. entries in EtherCAT error list
pub const MAX_E_LIST_ENTRIES: usize = 64;

/// Max length of readable name in slavelist and Object Description List
pub const MAX_NAME_LENGTH: usize = 40;

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
pub const MAX_SM: usize = 8;

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

pub struct SyncManager {
    pub start_address: u16,
    pub sm_length: u16,
    pub sm_flags: u32,
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

pub const SYNC_MANAGER_ENABLE_MASK: u32 = 0xFFFE_FFFF;

/// Sync manager type
pub enum SyncManagerType {
    Unused,
    MailboxWrite,
    MailboxRead,
    Outputs,
    Inputs,
}

/// Amount of data per read from EEProm
pub enum EepReadSize {
    Bytes4,
    Bytes8,
}

/// Detected EtherCAT slave
pub struct Slave {
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
    pub output_bytes: u32,

    /// Output IOmap buffer
    pub outputs: Vec<u8>,

    /// Startbit in first output byte
    pub output_startbit: u8,

    pub input_bits: u16,

    /// Input bytes, if input_bits < 8, output_bytes = 0
    pub input_bytes: u16,

    /// Input IOmap buffer
    pub input: Vec<u8>,

    /// Startbit in IOmap buffer
    pub input_startbit: u8,

    pub sync_manager: [SyncManager; MAX_SM],
    pub sync_manager_type: [SyncManagerType; MAX_SM],

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
    pub servo_over_ethercat_details: u8,

    /// E-bus current
    pub ebus_current: u8,

    /// If > 0 block use of LRW in processdata
    pub block_logical_read_write: u8,

    pub group: u8,

    /// First unused Fieldbus Memory Management Unit
    pub fmmu_unused: u8,

    /// Whether the slave is responding, not used by the SOEM library
    pub is_lost: bool,

    /// Registered configuration function PO -> SO (DEPRECATED)
    pub po2_so_config: fn(slave: u16) -> i32,

    /// Registered configuration function PO->SO
    pub po2_so_configx: fn(context: &mut Context, slave: u16) -> i32,

    pub readable_name: HeaplessString<{ MAX_NAME_LENGTH + 1 }>,
}

///EtherCAT slave group
pub struct SlaveGroup {
    /// Logical start address for this group
    pub logical_start_address: u32,

    /// Output bytes, 0 if output bits < 0
    pub output_bytes: u32,

    /// Output IOmap buffer
    pub outputs: Vec<u8>,

    /// Input bytes, 0 if input bits < 8
    pub input_bytes: u32,

    /// Input IOmap buffer
    pub inputs: Vec<u8>,

    /// Has DC capability
    pub has_dc: bool,

    /// Next DC slave
    pub dc_next: u16,

    /// E-bus current
    pub ebus_current: i16,

    /// If >0 block use of logical read write in process data
    pub block_logical_read_write: u8,

    /// Number of used IO segments
    pub used_segment_count: u16,

    pub first_input_segment: u16,

    /// Expected workcounter outputs
    pub work_counter_outputs: u16,

    /// Expected workcounter inputs
    pub work_counter_inputs: u16,

    pub check_slaves_states: bool,

    /// OI segmentation list. Datagrams must not break SM in two.
    pub io_segments: [u32; MAX_IO_SEGMENTS],
}

/// Eeprom Fieldbus Memory Management Unit
pub struct EepromFmmu {
    pub start_position: u16,
    pub number_fmmu: u8,
    pub fmmu: [u8; 4],
}

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
pub struct EepromPdo {
    pub start_position: u16,
    pub length: u16,
    pub number_pdo: u16,
    pub index: [u16; MAX_EE_PDO],
    pub sync_manager: [u16; MAX_EE_PDO],
    pub bitsize: [u16; MAX_EE_PDO],
    pub sync_manager_bit_size: [u16; MAX_EE_PDO],
}

/// Mailbox buffer array
pub type MailboxBuffer = [u8; MAX_MAILBOX_SIZE];

/// Standard ethercat mailbox header
pub struct MailboxHeader {
    pub length: u16,
    pub address: u16,
    pub priority: u8,
    pub mailbox_type: u8,
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
pub struct SyncManagerCommunicationType {
    pub number: u8,
    pub null: u8,
    pub sync_manager_type: [SyncManagerType; MAX_SM],
}

/// Service data object assign structure for communication access
pub struct PdoAssign {
    pub number: u8,
    pub null: u8,
    pub index: [u16; 256],
}

/// Service data object assign structure for communication access
pub struct PdoDescription {
    pub number: u8,
    pub null: u8,
    pub pdo: [u32; 256],
}

/// Context structure referenced by all Ethernet eXtended functions
pub struct Context<'context> {
    /// Port reference may include red port
    pub port: Arc<Mutex<Port<'context>>>,

    pub slavelist: Arc<Mutex<[Slave]>>,

    /// Number of slaves found in configuration
    pub slave_count: Vec<usize>,

    /// Maximum nummber of slaves allowed in slavelist
    pub max_slaves: usize,

    pub grouplist: &'context mut [SlaveGroup],

    /// Maximum number of groups allowed in grouplist
    pub max_group: i32,

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
    sync_manager_communication_type: Vec<SyncManagerCommunicationType>,

    /// Internal pdo assign list
    pdo_assign: Vec<PdoAssign>,

    /// Internal pdo description list
    pdo_description: Vec<PdoDescription>,

    /// Internal eeprom sync manager list
    eep_sync_manager: Vec<EepromSyncManager>,

    /// Registered file over ethercat hook
    pub file_over_ethercat_hook: fn(slave: u16, packetnumber: i32, datasize: i32) -> i32,

    /// Registered Ethernet over ethercat hook
    pub ethernet_over_ethercat_hook:
        fn(context: &mut Context, slave: u16, ecembx: &mut dyn Any) -> i32,

    /// Flag to control legacy automatic state change or manual state change
    pub manual_state_change: i32,

    /// Userdata promotes application configuration especially in EC_VER2 with multiple ec_context
    /// instances.
    pub userdata: Vec<Box<dyn Any>>,
}

#[cfg(feature = "ec_ver1")]
pub mod ec_ver1 {
    use std::time::Duration;

    use super::super::r#type::Error;

    use super::{EepromFmmu, EepromPdo, EepromSyncManager, MailboxBuffer};

    pub fn ec_pusherror(ec: &Error) {
        todo!()
    }

    pub fn poperror(ec: &Error) -> bool {
        todo!()
    }

    pub fn is_error() -> bool {
        todo!()
    }

    pub fn packeterror() -> bool {
        todo!()
    }

    pub fn init(ifname: &str) -> i32 {
        todo!()
    }

    pub fn init_redundant(ifname: &str, if2name: &mut String) -> i32 {
        todo!()
    }

    pub fn close() {
        todo!()
    }

    pub fn sii_get_byte(slave: u16, address: u16) {
        todo!()
    }

    pub fn sii_find(slave: u16, cat: u16) {
        todo!()
    }

    pub fn sii_string(str: &mut String, slave: u16, sn: u16) {
        todo!()
    }

    pub fn sii_fmmu(slave: u16, fmmu: &mut EepromFmmu) -> u16 {
        todo!()
    }

    pub fn sii_sm(slave: u16, sm: &mut EepromSyncManager) -> u16 {
        todo!()
    }

    pub fn sii_sm_next(slave: u16, sm: &mut EepromSyncManager, n: u16) -> u16 {
        todo!()
    }

    pub fn sii_pdo(slave: u16, pdo: &mut EepromPdo, t: u8) -> u32 {
        todo!()
    }

    pub fn readstate() -> i32 {
        todo!()
    }

    pub fn writestate(slave: u16) -> i32 {
        todo!()
    }

    pub fn statecheck(slave: u16, request_state: u16, timeout: Duration) -> u16 {
        todo!()
    }

    pub fn mailbox_empty(slave: u16, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn mailbox_send(slave: u16, mailbox: &mut MailboxBuffer, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn mailbox_receive(slave: u16, mailbox: &mut MailboxBuffer, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn esi_dump(slave: u16, esibuf: &mut Vec<u8>) {
        todo!()
    }

    pub fn read_eeprom(slave: u16, eeproma: u16, timeout: Duration) {
        todo!()
    }

    pub fn write_eeprom(slave: u16, eeproma: u16, data: u16, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn eeprom_to_master(slave: u16) -> i32 {
        todo!()
    }

    pub fn eeprom_to_pdi(slave: u16) -> i32 {
        todo!()
    }

    pub fn read_eeprom_ap(aiad: u16, eeproma: u16, timeout: Duration) -> u64 {
        todo!()
    }

    pub fn write_eeprom_ap(aiadr: u16, eeproma: u16, timeout: Duration) -> u64 {
        todo!()
    }

    pub fn read_eeprom_fp(config_address: u16, eeproma: u16, timeout: Duration) -> u64 {
        todo!()
    }

    pub fn write_eeprom_fp(config_address: u16, eeproma: u16, data: u16, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn read_eeprom1(slave: u16, eeproma: u16) {
        todo!()
    }

    pub fn read_eeprom2(slave: u16, timeout: Duration) -> u32 {
        todo!()
    }

    pub fn send_processdata_group(group: u8) -> i32 {
        todo!()
    }

    pub fn send_overlap_processdata_group(group: u8) -> i32 {
        todo!()
    }

    pub fn receive_processdata_group(group: u8) {
        todo!()
    }

    pub fn send_processdata() -> i32 {
        todo!()
    }

    pub fn send_overlap_processdata() -> i32 {
        todo!()
    }

    pub fn receive_processdata(timeout: Duration) -> i32 {
        todo!()
    }
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

pub fn clear_mailbox(mailbox: &mut MailboxBuffer) {
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

pub fn packet_error(context: &mut Context, slave: u16, index: u16, sub_index: u8, error_code: u16) {
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

pub fn sii_find(context: &mut Context, slave: u16, address: u16) -> u8 {
    todo!()
}

pub fn sii_string(context: &mut Context, str: &mut String, slave: u16, sn: u16) {
    todo!()
}

pub fn sii_fmmu(context: &mut Context, slave: u16, fmmu: &mut EepromFmmu) -> u16 {
    todo!()
}

pub fn sii_sm(context: &mut Context, slave: u16, sm: &mut EepromSyncManager, n: u16) -> u16 {
    todo!()
}

pub fn sii_pdo(context: &mut Context, slave: u16, pdo: &mut EepromPdo, t: u8) {
    todo!()
}

pub fn readstate(context: &mut Context) -> i32 {
    todo!()
}

pub fn writestate(context: &mut Context) -> i32 {
    todo!()
}

pub fn statecheck(context: &mut Context, slave: u16, request_state: u16, timeout: Duration) -> u16 {
    todo!()
}

pub fn mailbox_empty(context: &mut Context, slave: u16, timeout: Duration) -> i32 {
    todo!()
}

pub fn mailbox_send(
    context: &mut Context,
    slave: u16,
    mailbox: &mut MailboxBuffer,
    timeout: Duration,
) -> Result<u16, MailboxError> {
    todo!()
}

pub fn mailbox_receive(
    context: &mut Context,
    slave: u16,
    mailbox: &mut MailboxBuffer,
    timeout: Duration,
) -> Result<u16, MailboxError> {
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

pub fn read_eeprom1(context: &mut Context, slave: u16, eeproma: u16) {
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
