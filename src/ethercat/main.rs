//! Main EtherCAT functions.
//!
//! Initializartion, state set and read, mailbox primitives, EEPROM primitives,
//! SII reading and processdata exchange.
//!
//! Defines ec_slave[]. All slave information is put in this structure.
//! Needed for most user interaction with slaves.

use std::{
    any::Any,
    array,
    io::{self, Read, Write},
    str::Utf8Error,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::{Duration, SystemTime},
};

use bytemuck::{AnyBitPattern, NoUninit, Zeroable};
use heapless::String as HeaplessString;
use oshw::nicdrv::{Port, RedPort};

use super::{
    base::brd,
    r#type::{
        AbortError, ErrorInfo, ErrorType, Ethercat, EthercatState, SiiCategory, SiiGeneralItem,
        MAX_BUF_COUNT, TIMEOUT_RETURN,
    },
};
use crate::{
    ethercat::{
        base::{add_datagram, setup_datagram},
        r#type::{
            ethercat_to_host, BufferState, CommandType, EthercatRegister, EthernetHeader,
            ETHERCAT_HEADER_SIZE, ETHERCAT_WORK_COUNTER_SIZE, ETHERNET_HEADER_SIZE,
            MAX_EEP_BUF_SIZE, SII_START, TIMEOUT_EEP, TIMEOUT_RET3,
        },
    },
    oshw::{
        self, host_to_network,
        nicdrv::{NicdrvError, SECONDARY_MAC},
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
    InvalidMailboxType(u8),
    InvalidCOEMailboxType(u8),
    InvalidEthercatState(u8),
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

/// EtherCAT Adapter
pub struct Adapter {
    pub name: HeaplessString<MAX_ADAPTER_NAME_LENGTH>,
    pub desc: HeaplessString<MAX_ADAPTER_NAME_LENGTH>,
}

/// Create list of available network adapters
///
/// # Returns
/// List of available network adapters
impl Adapter {
    pub fn find_adapters() -> Vec<Self> {
        oshw::find_adaters()
    }
}

/// Fieldbus Memory Management Unit
#[derive(Debug, Clone, Copy)]
pub struct Fmmu {
    pub log_start: Ethercat<u32>,
    pub log_length: Ethercat<u16>,
    pub log_start_bit: u8,
    pub log_end_bit: u8,
    pub physical_start: Ethercat<u16>,
    pub physical_start_bit: u8,
    pub fmmu_type: u8,
    pub fmmu_active: u8,
    pub unused: u8,
    pub unused2: Ethercat<u16>,
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
    pub start_address: Ethercat<u16>,
    pub sm_length: Ethercat<u16>,
    pub sm_flags: Ethercat<u32>,
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
#[expect(dead_code)]
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
#[derive(Debug, PartialEq, Eq)]
pub enum EepReadSize {
    Bytes4,
    Bytes8,
}

/// Detected EtherCAT slave
pub struct Slave<'slave> {
    /// State of slave
    pub state: EthercatState,

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

/// Record for Ethercat EEPROM communications
pub struct Eeprom {
    comm: u16,
    address: u16,
    d2: u16,
}

pub fn read_eeprom(
    context: &mut Context,
    slave: u16,
    eeproma: u16,
    timeout: Duration,
) -> Ethercat<u32> {
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

/// Eeprom Fieldbus Memory Management Unit
#[derive(Debug, Clone, Copy, Default)]
pub struct EepromFmmu {
    pub start_position: u16,
    pub number_fmmu: u8,
    pub fmmu: [u8; 4],
}

impl EepromFmmu {
    /// Get FMMU data from SII FMMU section in slave EEPROM
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave number
    ///
    /// # Returns
    /// FMMU struct from SII (maximum is 4 FMMU's)
    pub fn sii_fmmu(context: &mut Context, slave: u16) -> Self {
        let slave_usize = usize::from(slave);
        let eeprom_control = context.slavelist[slave_usize].eep_pdi;

        let mut number_fmmu = 0;
        let mut fmmu = [0; 4];
        let start_position = context
            .sii_find(slave, SiiCategory::FMMU)
            .unwrap_or_default();

        if start_position > 0 {
            let address = start_position;
            number_fmmu = context.sii_get_byte(slave, address).unwrap() * 2;
            fmmu[0] = context.sii_get_byte(slave, address + 2).unwrap();
            fmmu[1] = context.sii_get_byte(slave, address + 3).unwrap();
            if number_fmmu > 2 {
                fmmu[2] = context.sii_get_byte(slave, address + 4).unwrap();
                fmmu[3] = context.sii_get_byte(slave, address + 5).unwrap();
            }
        }

        // If eeprom was previously pdi, then restore
        if eeprom_control {
            eeprom_to_pdi(context, slave);
        }
        Self {
            start_position,
            number_fmmu,
            fmmu,
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct EepromSyncManager {
    pub start_position: u16,
    pub sync_manager_count: u8,
    pub phase_start: u16,
    pub phase_length: u16,
    pub control_register: u8,

    /// Not used by SOEM
    pub slave_register: u8,

    pub activate: u8,

    /// Not used by SOEM
    pub process_data_interface_control: u8,
}

impl EepromSyncManager {
    /// Get SyncManaer from SII SyncManager section in slave EEPROM
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave number
    ///
    /// # Returns
    /// First SyncManager struct from SII
    pub fn sii_sm(context: &mut Context, slave: u16) -> Self {
        let eeprom_control = context.slavelist[usize::from(slave)].eep_pdi;

        let start_position = context.sii_find(slave, SiiCategory::SM).unwrap();
        let sync_manager = if start_position > 0 {
            let address = start_position;
            let w = u16::from(context.sii_get_byte(slave, address).unwrap())
                + (u16::from(context.sii_get_byte(slave, address + 1).unwrap()) << 8);
            Self {
                sync_manager_count: (w / 4) as u8,
                phase_start: u16::from(context.sii_get_byte(slave, address + 2).unwrap())
                    + (u16::from(context.sii_get_byte(slave, address + 3).unwrap()) << 8),
                phase_length: u16::from(context.sii_get_byte(slave, address + 4).unwrap())
                    + (u16::from(context.sii_get_byte(slave, address + 5).unwrap()) << 8),
                control_register: context.sii_get_byte(slave, address + 6).unwrap(),
                start_position,
                slave_register: context.sii_get_byte(slave, address + 7).unwrap(),
                activate: context.sii_get_byte(slave, address + 8).unwrap(),
                process_data_interface_control: context.sii_get_byte(slave, address + 9).unwrap(),
            }
        } else {
            Self::default()
        };

        // If eeprom control was previously pdi, restore it
        if eeprom_control {
            eeprom_to_pdi(context, slave);
        }
        sync_manager
    }

    /// Get next SyncManager data from SII SyncManager section in slave EEPROM.
    ///
    /// # Parameters
    /// `self`: First sync manager struct from SII
    /// `context`: Context struct
    /// `slave`: Slave number
    /// `number`: Sync manager number
    ///
    /// # Returns
    /// Requested Eeprom SyncManager if available
    pub fn sii_sm_next(
        &self,
        context: &mut Context,
        slave: u16,
        index: u8,
    ) -> Option<EepromSyncManager> {
        let eeprom_control = context.slavelist[usize::from(slave)].eep_pdi;
        let sync_manager = if index < self.sync_manager_count {
            let address = self.start_position + 2 + u16::from(index) * 8;
            Some(Self {
                phase_start: u16::from(context.sii_get_byte(slave, address).unwrap())
                    + (u16::from(context.sii_get_byte(slave, address + 1).unwrap()) << 8),
                phase_length: u16::from(context.sii_get_byte(slave, address + 2).unwrap())
                    + (u16::from(context.sii_get_byte(slave, address + 3).unwrap()) << 8),
                control_register: context.sii_get_byte(slave, address + 4).unwrap(),
                slave_register: context.sii_get_byte(slave, address + 5).unwrap(),
                activate: context.sii_get_byte(slave, address + 6).unwrap(),
                process_data_interface_control: context.sii_get_byte(slave, address + 7).unwrap(),
                start_position: self.start_position,
                sync_manager_count: self.sync_manager_count,
            })
        } else {
            None
        };

        // If eeprom control was previously pdi then restore
        if eeprom_control {
            eeprom_to_pdi(context, slave);
        }
        sync_manager
    }
}

/// Eeprom Process Data Object
#[derive(Debug)]
pub struct EepromPdo {
    pub start_position: u16,
    pub length: u16,
    pub pdo_count: u16,
    pub index: [u16; MAX_EE_PDO],
    pub sync_manager: [u16; MAX_EE_PDO],
    pub bit_size: [u16; MAX_EE_PDO],
    pub sync_manager_bit_size: [u16; MAX_SM as usize],
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
    /// Get PDO data from SII section in slave EEPROM.
    ///
    /// # Parameters
    /// `context`: Context struct
    /// `slave`: Slave number
    /// `transmitting`: false=RXPDO, true=TXPDO
    ///
    /// # Returns
    /// Mapping size in bits of PDO and the Pdo struct from SII
    pub fn sii_pdo(context: &mut Context, slave: u16, transmitting: bool) -> (u32, EepromPdo) {
        let eeprom_control = context.slavelist[usize::from(slave)].eep_pdi;
        let mut size = 0;
        let mut pdo_count = 0;
        let mut length = 0;
        let mut index = [0; MAX_EE_PDO];
        let mut sync_manager_bit_size = [0; MAX_SM as usize];
        let mut bit_size = [0; MAX_EE_PDO];
        let mut sync_manager = [0; MAX_EE_PDO];
        let pdo = if let Some(start_position) = context.sii_find(
            slave,
            if transmitting {
                SiiCategory::PDOSend
            } else {
                SiiCategory::PDOReceive
            },
        ) {
            let mut address = start_position;
            length = u16::from(context.sii_get_byte(slave, address).unwrap())
                + (u16::from(context.sii_get_byte(slave, address + 1).unwrap()) << 8);
            let mut current = 1;
            address += 2;

            // Traverse through all PDOs
            loop {
                pdo_count += 1;
                index[usize::from(pdo_count)] =
                    u16::from(context.sii_get_byte(slave, address).unwrap())
                        + (u16::from(context.sii_get_byte(slave, address + 1).unwrap()) << 8);
                current += 1;
                let e = u16::from(context.sii_get_byte(slave, address + 2).unwrap());
                sync_manager[usize::from(pdo_count)] =
                    u16::from(context.sii_get_byte(slave, address + 3).unwrap());
                address += 8;
                current += 2;

                // Check whether the sync manager is active and in range
                if sync_manager[usize::from(pdo_count)] < u16::from(MAX_SM) {
                    // Read all entries defined in PDO
                    for _err in 1..=e {
                        current += 4;
                        address += 5;
                        bit_size[usize::from(pdo_count)] +=
                            u16::from(context.sii_get_byte(slave, address).unwrap());
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
                length,
                start_position,
                pdo_count,
                index,
                sync_manager,
                bit_size,
                sync_manager_bit_size,
            }
        } else {
            Self {
                length,
                index,
                sync_manager_bit_size,
                start_position: 0,
                pdo_count,
                sync_manager,
                bit_size,
            }
        };

        // If eeprom control wa previously pdi, then restore
        if eeprom_control {
            eeprom_to_pdi(context, slave);
        }
        (size, pdo)
    }
}

/// Mailbox error structure
struct MailboxError {
    mailbox_header: MailboxHeader,
    error_type: u16,
    detail: u16,
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
    ) -> Result<u16, MainError> {
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
    ) -> Result<u16, MainError> {
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
    pub length: Ethercat<u16>,
    pub address: Ethercat<u16>,
    pub priority: u8,
    pub mailbox_type: u8,
}

impl MailboxHeader {
    pub const fn size() -> usize {
        2 * size_of::<u16>() + 2 * size_of::<u8>()
    }

    pub fn write_to(&self, bytes: &mut impl Write) -> io::Result<()> {
        bytes.write_all(&self.length.to_bytes())?;
        bytes.write_all(&self.address.to_bytes())?;
        bytes.write_all(&[self.priority, self.mailbox_type])
    }

    pub fn read_from<R: Read>(bytes: &mut R) -> io::Result<Self> {
        let mut value = [0; 6];
        bytes.read_exact(&mut value)?;
        Ok(Self {
            length: Ethercat::from_raw(u16::from_ne_bytes(value[..2].try_into().unwrap())),
            address: Ethercat::from_raw(u16::from_ne_bytes(value[2..4].try_into().unwrap())),
            priority: value[4],
            mailbox_type: value[5],
        })
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ApplicationLayerStatus {
    pub status: u16,
    pub unused: u16,
    pub code: u16,
}

unsafe impl NoUninit for ApplicationLayerStatus {}

unsafe impl Zeroable for ApplicationLayerStatus {}

unsafe impl AnyBitPattern for ApplicationLayerStatus {}

impl<'status> From<&'status mut ApplicationLayerStatus> for &'status mut [u8] {
    fn from(value: &'status mut ApplicationLayerStatus) -> Self {
        bytemuck::bytes_of_mut(value)
    }
}

impl ApplicationLayerStatus {
    pub fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
        let mut word = [0; 2];
        reader.read_exact(&mut word)?;
        let status = u16::from_ne_bytes(word);
        reader.read_exact(&mut word)?;
        let unused = u16::from_ne_bytes(word);
        reader.read_exact(&mut word)?;
        let code = u16::from_ne_bytes(word);
        Ok(Self {
            status,
            unused,
            code,
        })
    }
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
    pub head: u16,
    pub tail: u16,
    pub error: [ErrorInfo; MAX_ERROR_LIST_ENTRIES + 1],
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

#[derive(Debug, Clone, Copy)]
pub enum PacketError {
    UnexpectedFrameReturned = 1,
    DataContainerTooSmallForType = 3,
    TooManySyncManagers = 10,
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
    pub ecaterror: Arc<AtomicBool>,

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
    pub file_over_ethercat_hook: Option<fn(slave: u16, packetnumber: i32, datasize: i32) -> i32>,

    /// Registered Ethernet over ethercat hook
    pub ethernet_over_ethercat_hook:
        fn(context: &mut Context, slave: u16, ecembx: &mut [u8]) -> i32,

    /// Flag to control legacy automatic state change or manual state change
    pub manual_state_change: bool,

    /// Userdata promotes application configuration especially in EC_VER2 with multiple ec_context
    /// instances.
    pub userdata: Vec<u8>,
}

impl<'context> Context<'context> {
    /// Initialize library in single NIC mode
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `interface_name`: Device name, f.e. "eth0"
    ///
    /// # Returns
    /// `Ok(())` or error
    pub fn init(&mut self, interface_name: &str) -> Result<(), NicdrvError> {
        self.port.lock().unwrap().setup_nic(interface_name, false)
    }

    /// Initialize library in redundant NIC mode
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `redport`: Mutable reference to redundant port data
    /// - `interface_name`: Primary device name f.e. "etho"
    /// - `interface_name2`: Secondary device name, f.e. "eth1"
    ///
    /// # Returns
    /// `Ok(())` or error
    pub fn init_redundant<'port: 'context>(
        &mut self,
        redport: RedPort<'port>,
        interface_name: &str,
        interface_name2: &str,
    ) -> Result<(), NicdrvError> {
        let mut port = self.port.lock().unwrap();
        port.redport = Some(redport);
        &mut port.setup_nic(interface_name, false)?;
        let rval = port.setup_nic(interface_name2, true);

        // Prepare "dummy" broadcat read tx frame for redundant operation
        let mut ethernet_header =
            EthernetHeader::try_from(port.temp_tx_buffer.lock().unwrap().as_slice())?;
        ethernet_header.source_address[1] = host_to_network(SECONDARY_MAC[0]);
        port.temp_tx_buffer
            .lock()
            .unwrap()
            .copy_from_slice(ethernet_header.as_ref());
        let mut zbuf = [0; 2];
        setup_datagram(
            &mut port.temp_tx_buffer.lock().unwrap(),
            CommandType::BroadcastRead,
            0,
            0,
            2,
            &mut zbuf,
        );
        port.temp_tx_buffer
            .lock()
            .unwrap()
            .resize(
                ETHERCAT_HEADER_SIZE + ETHERNET_HEADER_SIZE + ETHERCAT_WORK_COUNTER_SIZE + 2,
                0,
            )
            .unwrap();
        rval
    }

    /// Pushes error on the error list
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `error`: Error describtion
    pub fn push_error(&mut self, mut error: ErrorInfo) {
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
        self.ecaterror.store(true, Ordering::Relaxed);
    }

    /// Pops an error from the list
    ///
    /// # Parameters
    /// - `self`: Context struct
    ///
    /// # Returns
    /// `Some(ErrorInfo)` if an error is present, None if empty
    pub fn pop_error(&mut self) -> Option<ErrorInfo> {
        self.elist.error[usize::from(self.elist.tail)].signal = false;
        if self.elist.head == self.elist.tail {
            self.ecaterror.store(false, Ordering::Relaxed);
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

    /// Close library
    ///
    /// # Parameters
    /// - `self`: Context struct
    pub fn close(&mut self) -> Result<(), crate::safe_c::CloseError> {
        self.port.lock().unwrap().close_nic()
    }

    pub fn readstate(&mut self) -> i32 {
        todo!()
    }

    pub fn writestate(&mut self) -> i32 {
        todo!()
    }

    /// Report packet error
    ///
    /// # Parameters
    /// - `context`: Context struct
    /// - `slave`: Slave number
    /// - `index`: Index that generated the error
    /// - `sub_index`: Subindex that generated the error
    /// - `error_code`: Error code
    pub fn packet_error(&mut self, slave: u16, index: u16, sub_index: u8, error_code: PacketError) {
        self.ecaterror.store(true, Ordering::Relaxed);
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
    /// # Returns
    /// Requested byte or None
    pub fn sii_get_byte(&mut self, slave: u16, address: u16) -> Option<u8> {
        // Make sure the selected slave is cached
        if slave != self.esislave {
            self.esimap.fill(0);
            self.esislave = slave;
        }

        if address < MAX_EEP_BUF_SIZE {
            let mapw = address >> 5;
            let mut mapb = address - (mapw << 5);
            if self.esimap[usize::from(mapw)] & (1 << mapb) != 0 {
                // Byte is already in buffer
                return Some(self.esibuf[usize::from(address)]);
            } else {
                // Byte is not in buffer, put it there
                let config_address = self.slavelist[usize::from(slave)].config_address;

                // Set eeprom cotrol to master
                eeprom_to_master(self, slave);
                let eeprom_address = address >> 1;
                let eeprom_data64 =
                    read_eeprom_fp(self, config_address, eeprom_address, TIMEOUT_EEP);

                let count =
                    if self.slavelist[usize::from(slave)].eep_read_size == EepReadSize::Bytes8 {
                        // 8 byte response
                        self.esibuf[usize::from(eeprom_address) << 1..]
                            .copy_from_slice(&eeprom_data64.to_ne_bytes());
                        8
                    } else {
                        // 4 byte response
                        self.esibuf[usize::from(eeprom_address) << 1..]
                            .copy_from_slice(&(eeprom_data64 as u32).to_ne_bytes());
                        4
                    };

                // Find bitmap location
                let mut mapw = eeprom_address >> 4;
                mapb = (eeprom_address << 1) - (mapw << 5);
                for _ in 0..count {
                    // Set bitmap for each byte that is read
                    self.esimap[usize::from(mapw)] |= 1 << mapb;
                    mapb += 1;
                    if mapb > 31 {
                        mapb = 0;
                        mapw += 1;
                    }
                }
                return Some(self.esibuf[usize::from(address)]);
            }
        }
        None
    }

    /// Find SII section header in slave EEPROM
    ///
    /// # Parameters
    /// - `self`: Context struct
    /// - `slave`: Slave number
    /// - `category`: Section category
    ///
    /// # Returns
    /// byte address of section at section length entry, None if not available
    pub fn sii_find(&mut self, slave: u16, category: SiiCategory) -> Option<u16> {
        let slave_usize = usize::from(slave);
        let eeprom_control = self.slavelist[slave_usize].eep_pdi;

        let mut address = SII_START << 1;

        // Read first SII section category
        let mut p = u16::from(self.sii_get_byte(slave, address).unwrap());
        p += u16::from(self.sii_get_byte(slave, address + 1).unwrap()) << 8;
        address += 2;

        // Traverse SII while category is not found and not End Of File
        while p != u16::from(category) && p != 0xFFFF {
            // Read section length
            p = match self.sii_get_byte(slave, address) {
                Some(byte) => u16::from(byte),
                None => break,
            };
            p += match self.sii_get_byte(slave, address + 1) {
                Some(byte) => u16::from(byte) << 8,
                None => break,
            };

            // Locate next section category
            address += p << 1;

            // Read section category
            p = match self.sii_get_byte(slave, address) {
                Some(byte) => u16::from(byte),
                None => break,
            };
            p += match self.sii_get_byte(slave, address + 1) {
                Some(byte) => u16::from(byte) << 8,
                None => break,
            }
        }
        if eeprom_control {
            eeprom_to_pdi(self, slave);
        }
        (p == u16::from(category)).then_some(p)
    }

    /// Get string from SII string section in slave EEPROM.
    ///
    /// # Parameters
    ///`self`: Context struct
    /// `string`: Requested string
    /// `slave`: Slave number
    /// `string_number`: String number
    pub fn sii_string<const SIZE: usize>(
        &mut self,
        string: &mut heapless::String<SIZE>,
        slave: u16,
        string_number: u16,
    ) -> Result<(), Utf8Error> {
        let slave_usize = usize::from(slave);
        let eeprom_control = self.slavelist[slave_usize].eep_pdi;

        let mut result = heapless::Vec::<u8, SIZE>::new();
        if let Some(address) = self.sii_find(slave, SiiCategory::String) {
            // Skip SII section header
            let mut address = address + 2;

            // Read number of strings in section
            let number = self.sii_get_byte(slave, address);
            address += 1;

            // Check whether the requested string is available
            if number.is_some_and(|number| string_number <= u16::from(number)) {
                let mut i = 0;
                while i < string_number {
                    i += 1;
                    let Some(length) = self.sii_get_byte(slave, address) else {
                        continue;
                    };
                    if i < string_number {
                        address += u16::from(length);
                    } else {
                        for j in 1..=length {
                            if u16::from(j) <= MAX_NAME_LENGTH {
                                result.push(self.sii_get_byte(slave, address).unwrap());
                                address += 1;
                            } else {
                                address += 1;
                            }
                        }
                    }
                }
            }
        }

        // If eeprom control was previously pdi, then restore
        if eeprom_control {
            eeprom_to_pdi(self, slave);
        }
        *string = HeaplessString::from_utf8(result)?;
        Ok(())
    }

    /// Read all slave states in `ec_slave`
    ///
    /// # Warning
    /// The BOOT state is actually higher than INIT and PRE_OP (see state representation)
    ///
    /// # Parameters
    /// `context`: Context struct
    ///
    /// # Returns
    /// lowest state found.
    fn read_state(&mut self) -> Result<EthercatState, MainError> {
        // Try to establish the state of all slaves, sending only one broadcast datagram.
        // This way, a number of datagrams equal to the number of slaves will be send when needed.
        let mut byte = [0];
        let wkc = brd(
            &mut self.port.lock().unwrap(),
            0,
            EthercatRegister::ApplicationLayerStatus,
            &mut byte,
            TIMEOUT_RETURN,
        )?;

        let all_slaves_present = wkc >= self.slave_count;

        let received_value = ethercat_to_host(Ethercat::from_raw(byte[0]));

        let error_flag = if received_value & u8::from(EthercatState::Error) != 0 {
            false
        } else {
            self.slavelist[0].al_status_code = 0;
            true
        };

        let bitwise_state = received_value & 0xF;
        let all_slaves_same_state = match EthercatState::try_from(bitwise_state) {
            state @ (Ok(EthercatState::Init)
            | Ok(EthercatState::PreOperational)
            | Ok(EthercatState::SafeOperational)
            | Ok(EthercatState::Operational)) => {
                self.slavelist[0].state = state.unwrap();
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
            for slave in self.slavelist[1..=usize::from(self.slave_count)].iter_mut() {
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
                        status: 0,
                        unused: 0,
                        code: 0,
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
                    TIMEOUT_RET3,
                )?;

                for slave in fslave..=lslave {
                    let config_address = self.slavelist[usize::from(slave)].config_address;
                    let received_value = ethercat_to_host(Ethercat::from_raw(
                        sl[usize::from(slave - fslave)].status,
                    ));
                    self.slavelist[usize::from(slave)].al_status_code =
                        ethercat_to_host(Ethercat::from_raw(sl[usize::from(slave - fslave)].code));
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
}

fn fixed_pointer_read_multi(
    context: &mut Context,
    number: u16,
    config_list: &mut [u16],
    sl_status_list: &mut [ApplicationLayerStatus],
    timeout: Duration,
) -> Result<(), MainError> {
    let mut port = context.port.lock().unwrap();
    let index = port.get_index();
    let mut sl_count: u16 = 0;
    setup_datagram(
        &mut port.tx_buffers.lock().unwrap()[usize::from(index)],
        CommandType::FixedPointerRead,
        index,
        config_list[usize::from(sl_count)],
        EthercatRegister::ApplicationLayerStatus.into(),
        (&mut sl_status_list[usize::from(sl_count)]).into(),
    );
    let mut sl_data_position = [0; MAX_FIXED_POINTER_READ_MULTI];
    sl_data_position[usize::from(sl_count)] = ETHERCAT_HEADER_SIZE;
    sl_count += 1;
    for sl_count in sl_count..number - 1 {
        sl_data_position[usize::from(sl_count)] = add_datagram(
            &mut port.tx_buffers.lock().unwrap()[usize::from(index)],
            CommandType::FixedPointerRead,
            index,
            true,
            config_list[usize::from(sl_count)],
            EthercatRegister::ApplicationLayerStatus.into(),
            (&mut sl_status_list[usize::from(sl_count)]).into(),
        );
    }
    sl_count = sl_count.max(number - 1);
    if sl_count < number {
        sl_data_position[usize::from(sl_count)] = add_datagram(
            &mut port.tx_buffers.lock().unwrap()[usize::from(index)],
            CommandType::FixedPointerRead,
            index,
            false,
            config_list[usize::from(sl_count)],
            EthercatRegister::ApplicationLayerStatus.into(),
            (&mut sl_status_list[usize::from(sl_count)]).into(),
        );
    }
    port.src_confirm(index, timeout)?;
    for sl_count in 0..number {
        sl_status_list[usize::from(sl_count)] = ApplicationLayerStatus::read_from(
            &mut &port.rx_buf.lock().unwrap()[usize::from(index)].as_mut_slice()
                [sl_data_position[usize::from(sl_count)]..],
        )?;
    }

    port.set_buf_stat(usize::from(index), BufferState::Empty);
    Ok(())
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

pub fn eeprom_to_master(context: &mut Context, slave: u16) -> i32 {
    todo!()
}

pub fn eeprom_to_pdi(context: &mut Context, slave: u16) -> i32 {
    todo!()
}

pub fn read_eeprom_ap(context: &mut Context, aiadr: u16, eeproma: u16, timeout: Duration) -> u64 {
    todo!()
}

pub fn next_mailbox_count(count: u8) -> u8 {
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

pub fn read_eeprom2(context: &mut Context, slave: u16, timeout: Duration) -> Ethercat<u32> {
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

/// Emergency request structure
struct EmergencyRequest {
    mailbox_header: MailboxHeader,
    can_open: u16,
    error_code: u16,
    error_register: u8,
    bdata: u8,
    w1: u16,
    w2: u16,
}
