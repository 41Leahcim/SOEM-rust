//! Configuration module for EtherCAT master.
//!
//! After successfull initialization with `init()` or `init_redundant()`
//! the slaves can be auto configured with this module.

use std::{collections::VecDeque, fmt::Write as _, io, str::Utf8Error, time::Duration};

use super::{
    base::aprd,
    coe::CoEError,
    main::{Context, EepromRequest, MainError, Slave},
    r#type::TIMEOUT_RETURN3,
};
use crate::{
    ec_println,
    ethercat::{
        base::{aprdw, apwrw, brd, bwr, fprdw, fpwr, fpwrw},
        coe::{read_pdo_map, read_pdo_map_complete_access},
        dc::DistributedClock,
        main::{
            Coedet, EepReadSize, EepromFmmu, EepromPdo, EepromSyncManager, Fmmu, MailboxProtocol,
            SlaveGroup, SyncManager, SyncManagerType, MAX_IO_SEGMENTS, MAX_SM,
            SYNC_MANAGER_ENABLE_MASK,
        },
        r#type::{
            high_word, low_byte, low_word, Ethercat, EthercatRegister, EthercatState, SiiCategory,
            SiiGeneralItem, EEPROM_STATE_MACHINE_READ64, FIRST_DC_DATAGRAM_SIZE, LOG_GROUP_OFFSET,
            MAX_EEPROM_BUFFER_SIZE, MAX_LRW_DATA_LENGTH, TIMEOUT_EEPROM, TIMEOUT_SAFE,
            TIMEOUT_STATE,
        },
        soe::read_id_nmap,
    },
    oshw::nicdrv::NicdrvError,
};

pub const NODE_OFFSET: u16 = 0x1000;
pub const TEMP_NODE: u16 = 0xFFFF;

#[derive(Debug)]
pub enum ConfigError {
    Nicdrv(NicdrvError),
    CoEError(CoEError),
    Main(MainError),
    Utf8(Utf8Error),
    Io(io::Error),
    SlaveCountExceeded,
    SlaveFailsToRespond,
    FoundWrongSlave,
}

impl From<NicdrvError> for ConfigError {
    fn from(value: NicdrvError) -> Self {
        Self::Nicdrv(value)
    }
}

impl From<CoEError> for ConfigError {
    fn from(value: CoEError) -> Self {
        Self::CoEError(value)
    }
}

impl From<Utf8Error> for ConfigError {
    fn from(value: Utf8Error) -> Self {
        Self::Utf8(value)
    }
}

impl From<MainError> for ConfigError {
    fn from(value: MainError) -> Self {
        Self::Main(value)
    }
}

impl From<io::Error> for ConfigError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

/// Standard SyncManager0 flags configuration for mailbox slaves
const DEFAULT_MAILBOX_SM0: u32 = 0x0001_0026;

/// Standard SyncManager1 flags configuration for mailbox slaves
const DEFAULT_MAILBOX_SM1: u32 = 0x0001_0022;

/// Resets the slavelist and grouplist.
///
/// # Parameters
/// - `context`: The context containing the slavelist and grouplist to reset.
fn init_context(context: &mut Context) -> Result<(), MainError> {
    *context.slave_count_mut() = 0;

    // Clean slave list and grouplist
    context.reset_slave_list();
    context.reset_group_list();

    // Clear slave eeprom cache, doesn't actually read any eeprom
    context.sii_get_byte(0, MAX_EEPROM_BUFFER_SIZE)?;
    for lp in 0..context.max_group() {
        context.push_group(SlaveGroup::with_logical_start_address(
            lp << LOG_GROUP_OFFSET,
        ));
    }
    Ok(())
}

/// Resets and detects slaves.
///
/// # Parameters
/// - `context`: The context containing the port with slaves and to store the slave count in
///
/// # Returns
/// The number of slaves detected or an error
fn detect_slaves(context: &mut Context) -> Result<u16, ConfigError> {
    // Make special pre-init register writes to enable MAC[1] local administered bit
    // setting for old netX100 slaves.

    // Ignore alias register
    let mut byte = [0];
    bwr(
        context.port_mut(),
        0,
        EthercatRegister::DeviceLayerAlias,
        &byte,
        TIMEOUT_RETURN3,
    )?;

    // Reset all slaves to initialization
    byte[0] = EthercatState::Init as u8 | EthercatState::Error as u8;
    bwr(
        context.port_mut(),
        0,
        EthercatRegister::ApplicationLayerControl,
        &byte,
        TIMEOUT_RETURN3,
    )?;
    // NetX100 should now be functional

    // Reset all slaves to initialization
    bwr(
        context.port_mut(),
        0,
        EthercatRegister::ApplicationLayerControl,
        &byte,
        TIMEOUT_RETURN3,
    )?;

    // Detect the number of slaves
    let work_counter = brd(
        context.port_mut(),
        0,
        EthercatRegister::Type,
        &mut byte,
        TIMEOUT_SAFE,
    )?;

    // This is strictly less than, since the master is slave 0
    if work_counter < context.max_slaves() {
        *context.slave_count_mut() = work_counter;
        Ok(work_counter)
    } else {
        ec_println!(
            "Error: Too many slaves on network: num_slaves={work_counter}, max_slaves={}",
            context.max_slaves()
        );
        Err(ConfigError::SlaveCountExceeded)
    }
}

fn set_slaves_to_default(context: &mut Context) -> Result<(), NicdrvError> {
    let port = context.port_mut();

    // Deactivate loop manual
    bwr(
        port,
        0,
        EthercatRegister::DeviceLayerPort,
        &[0],
        TIMEOUT_RETURN3,
    )?;

    // Set interrupt mask
    bwr(
        port,
        0,
        EthercatRegister::InterruptMask,
        &[0],
        TIMEOUT_RETURN3,
    )?;

    // Reset CRC counter
    bwr(
        port,
        0,
        EthercatRegister::ReceiveError,
        &[0; 8],
        TIMEOUT_RETURN3,
    )?;

    // Reset Fieldbus memory management units
    bwr(
        port,
        0,
        EthercatRegister::FieldbusMemoryManagementUnit0,
        &[0; 16 * 3],
        TIMEOUT_RETURN3,
    )?;

    // Reset synchronization manager
    bwr(
        port,
        0,
        EthercatRegister::SyncManager0,
        &[0; 8 * 4],
        TIMEOUT_RETURN3,
    )?;

    // Reset activation register
    bwr(
        port,
        0,
        EthercatRegister::DistributedClockSynchronizationActive,
        &[0],
        TIMEOUT_RETURN3,
    )?;

    // Reset system time + offset
    bwr(
        port,
        0,
        EthercatRegister::DistributedClockSystemTime,
        &[0; 4],
        TIMEOUT_RETURN3,
    )?;

    // Digital Clock speedstart
    bwr(
        port,
        0,
        EthercatRegister::DistributedClockSpeedCount,
        &Ethercat::from_host(0x1000u16).to_bytes(),
        TIMEOUT_RETURN3,
    )?;

    // Digital clock filter expression
    bwr(
        port,
        0,
        EthercatRegister::DistributedClockTimeFilter,
        &Ethercat::from_host(0xC00u16).to_bytes(),
        TIMEOUT_RETURN3,
    )?;

    // Ignore alias register
    bwr(
        port,
        0,
        EthercatRegister::DeviceLayerAlias,
        &[0],
        TIMEOUT_RETURN3,
    )?;

    // Reset all slaves to initialization
    bwr(
        port,
        0,
        EthercatRegister::ApplicationLayerControl,
        &[EthercatState::Init as u8 | EthercatState::Error as u8],
        TIMEOUT_RETURN3,
    )?;

    // Force eeprom from PDI
    bwr(
        port,
        0,
        EthercatRegister::EepromConfig,
        &[2],
        TIMEOUT_RETURN3,
    )?;

    // Set EEPROM to master
    bwr(
        port,
        0,
        EthercatRegister::EepromConfig,
        &[0],
        TIMEOUT_RETURN3,
    )?;

    Ok(())
}

fn config_from_table(_context: &mut Context, _slave: u16) -> u8 {
    0
}

/// If slave has SII and same slave ID was done before, use previous data.
/// This is safe, because SII is constant for same slave ID.
///
/// # Parameters
/// - `context`: The context containing the slave to copy to and the slave to copy from
/// - `slave`: The index of the slave to copy the configuration to
///
/// # Returns
/// Whether the configuration was copied
pub fn lookup_previous_sii(context: &mut Context, slave: u16) -> bool {
    if slave == 0 || context.slave_count() == 0 {
        return false;
    }

    let requested_slave = context.get_slave(slave);
    let found_slave = context
        .slavelist()
        .iter()
        .take(usize::from(slave))
        .enumerate()
        .find(|(_, current_slave)| {
            current_slave.eeprom().manufacturer() == requested_slave.eeprom().manufacturer()
                && current_slave.eeprom().id() == requested_slave.eeprom().id()
                && current_slave.eeprom().revision() == requested_slave.eeprom().revision()
        })
        .map(|(index, _)| index);

    let Some(found_slave) = found_slave else {
        return false;
    };

    let slave = usize::from(slave);
    let slaves = &mut context.slavelist_mut();
    *slaves[slave]
        .protocol_details_mut()
        .canopen_over_ethercat_mut() = slaves[found_slave]
        .protocol_details()
        .canopen_over_ethercat();
    *slaves[slave]
        .protocol_details_mut()
        .file_over_ethercat_mut() = slaves[found_slave].protocol_details().file_over_ethercat();
    *slaves[slave]
        .protocol_details_mut()
        .ethernet_over_ethercat_mut() = slaves[found_slave]
        .protocol_details()
        .ethernet_over_ethercat();
    *slaves[slave]
        .protocol_details_mut()
        .servo_over_ethercat_mut() = slaves[found_slave].protocol_details().servo_over_ethercat();

    if slaves[found_slave].block_logical_read_write() > 0 {
        *slaves[slave].block_logical_read_write_mut() = 1;
        *slaves[0].block_logical_read_write_mut() += 1;
    }

    *slaves[slave].ebus_current_mut() = slaves[found_slave].ebus_current();
    *slaves[0].ebus_current_mut() += slaves[slave].ebus_current();
    *slaves[slave].name_mut() = slaves[found_slave].name().clone();

    for sync_manager_index in 0..MAX_SM {
        *slaves[slave].get_sync_manager_mut(sync_manager_index) =
            slaves[found_slave].get_sync_manager(sync_manager_index);
    }
    *slaves[slave].fmmu0_function_mut() = slaves[found_slave].fmmu0_function();
    *slaves[slave].fmmu1_function_mut() = slaves[found_slave].fmmu1_function();
    *slaves[slave].fmmu2_function_mut() = slaves[found_slave].fmmu2_function();
    *slaves[slave].fmmu3_function_mut() = slaves[found_slave].fmmu3_function();
    ec_println!("Copy SII slave {slave} from {found_slave}.");
    true
}

fn async_eeprom_read(
    context: &mut Context,
    next_value: SiiGeneralItem,
    eeprom_requests: VecDeque<EepromRequest>,
    storing_function: impl Fn(&mut Slave, Ethercat<u32>),
) -> Result<VecDeque<EepromRequest>, MainError> {
    eeprom_requests
        .into_iter()
        .zip(0..context.slave_count())
        .map(|(request, slave)| {
            let eedata = request.read_eeprom_data(context, TIMEOUT_EEPROM)?;
            storing_function(context.get_slave_mut(slave), eedata);
            EepromRequest::request_eeprom_data(context, slave, next_value)
        })
        .collect()
}

fn read_mailbox_info(
    context: &mut Context,
    slave: u16,
    eeprom_requests: &mut VecDeque<EepromRequest>,
) -> Result<(), MainError> {
    // Read mailbox information
    let mailbox_info = eeprom_requests
        .pop_front()
        .unwrap()
        .read_eeprom_data(context, TIMEOUT_EEPROM)?;
    *context.get_slave_mut(slave).mailbox_mut().read_offset_mut() =
        low_word(mailbox_info.to_host());
    *context.get_slave_mut(slave).mailbox_mut().read_length_mut() =
        high_word(mailbox_info.to_host());
    if context.get_slave_mut(slave).mailbox().read_length() == 0 {
        *context.get_slave_mut(slave).mailbox_mut().read_length_mut() =
            context.get_slave(slave).mailbox().length();
    }
    eeprom_requests.push_back(EepromRequest::request_eeprom_data(
        context,
        slave,
        SiiGeneralItem::MailboxProtocol,
    )?);
    Ok(())
}

fn read_topology(
    context: &mut Context,
    config_address: u16,
    slave: u16,
) -> Result<(), NicdrvError> {
    // Extract topology from DL status
    let topology = fprdw(
        context.port_mut(),
        config_address,
        EthercatRegister::DeviceLayerStatus,
        TIMEOUT_RETURN3,
    )?
    .to_host();
    let mut topology_count = 0;
    let mut active_ports = 0;

    // Port 0 open and communication established
    if topology & 0x300 == 0x200 {
        topology_count += 1;
        active_ports |= 1;
    }

    // Port 1 open and communication established
    if topology & 0xC00 == 0x800 {
        topology_count += 1;
        active_ports |= 2;
    }

    // Port 2 open and communication established
    if topology & 0x3000 == 0x2000 {
        topology_count += 1;
        active_ports |= 4;
    }

    // Port 3 open and communication established
    if topology & 0xC000 == 0x8000 {
        topology_count += 1;
        active_ports |= 8;
    }
    *context.get_slave_mut(slave).topology_mut() = topology_count;
    *context.get_slave_mut(slave).active_ports_mut() = active_ports;
    Ok(())
}

fn search_parent(context: &mut Context, slave: u16) {
    // 0 = no links, not possible
    // 1 = 1 link, end of line
    // 2 = 2 links, one before and one after
    // 3 = 3 links, split point
    // 4 = 4 links, cross point
    // search for parent
    *context.get_slave_mut(slave).parent_port_mut() = 0;
    if slave > 1 {
        let mut topology_links = 0;
        let slave_count = slave - 1;
        loop {
            let topology = context.get_slave(slave_count).topology();
            match topology {
                // Endpoint found
                1 => topology_links -= 1,

                // Split found
                3 => topology_links += 1,

                // Cross found
                4 => topology_links += 2,

                // Ignore everything else
                _ => {}
            }

            // Parent found
            if topology_links >= 0 && topology > 1 || slave_count == 1 {
                *context.get_slave_mut(slave).parent_mut() = slave_count as u8;
                break;
            }
            topology_links -= 1;

            if slave_count == 0 {
                break;
            }
        }
    }
}

fn configure_mailbox(
    context: &mut Context,
    slave: u16,
    eeprom_requests: &mut VecDeque<EepromRequest>,
) -> Result<(), MainError> {
    let sync_manager_type = context.get_slave_mut(slave).sync_manager_type_mut();
    sync_manager_type[0] = SyncManagerType::MailboxWrite;
    sync_manager_type[1] = SyncManagerType::MailboxRead;
    sync_manager_type[2] = SyncManagerType::Outputs;
    sync_manager_type[3] = SyncManagerType::Inputs;
    let slave_object = context.get_slave_mut(slave);
    *slave_object.get_sync_manager_mut(0).start_address_mut() =
        Ethercat::from_host(slave_object.mailbox().write_offset());
    *slave_object.get_sync_manager(0).sm_length_mut() =
        Ethercat::from_host(slave_object.mailbox().length());
    *slave_object.get_sync_manager_mut(0).sm_flags_mut() = Ethercat::from_host(DEFAULT_MAILBOX_SM0);
    *slave_object.get_sync_manager_mut(1).start_address_mut() =
        Ethercat::from_host(slave_object.mailbox().read_offset());
    *slave_object.get_sync_manager_mut(1).sm_length_mut() =
        Ethercat::from_host(slave_object.mailbox().read_length());
    *slave_object.get_sync_manager_mut(1).sm_flags_mut() = Ethercat::from_host(DEFAULT_MAILBOX_SM1);
    *context.get_slave_mut(slave).mailbox_mut().protocols_mut() = eeprom_requests
        .pop_front()
        .unwrap()
        .read_eeprom_data(context, TIMEOUT_EEPROM)?
        .to_host() as u16;
    Ok(())
}

fn configure_slave_with_sii(context: &mut Context, slave: u16) -> Result<(), MainError> {
    let sii_general = context
        .sii_find(slave, SiiCategory::General)
        .unwrap_or_default();
    if sii_general != 0 {
        *context
            .get_slave_mut(slave)
            .protocol_details_mut()
            .canopen_over_ethercat_mut() = context.sii_get_byte(slave, sii_general + 7).unwrap();
        *context
            .get_slave_mut(slave)
            .protocol_details_mut()
            .file_over_ethercat_mut() = context.sii_get_byte(slave, sii_general + 8).unwrap();
        *context
            .get_slave_mut(slave)
            .protocol_details_mut()
            .ethernet_over_ethercat_mut() = context.sii_get_byte(slave, sii_general + 9).unwrap();
        *context
            .get_slave_mut(slave)
            .protocol_details_mut()
            .servo_over_ethercat_mut() = context.sii_get_byte(slave, sii_general + 0xA).unwrap();
        if context.sii_get_byte(slave, sii_general + 0xD).unwrap() & 0x2 > 0 {
            *context.get_slave_mut(slave).block_logical_read_write_mut() = 1;
            *context.get_slave_mut(0).block_logical_read_write_mut() += 1;
        }
        *context.get_slave_mut(slave).ebus_current_mut() =
            u16::from(context.sii_get_byte(slave, sii_general + 0xE).unwrap())
                + (u16::from(context.sii_get_byte(slave, sii_general + 0xF).unwrap()) << 8);
        *context.get_slave_mut(0).ebus_current_mut() += context.get_slave(slave).ebus_current();
    }

    // SII strings section
    if context.sii_find(slave, SiiCategory::String).is_ok() {
        *context.get_slave_mut(slave).name_mut() = context.sii_string(slave, 1)?;
    } else {
        // No name for slave found, use constructed name
        let manufacturer = context.get_slave_mut(slave).eeprom().manufacturer();
        let id = context.get_slave(slave).eeprom().id();
        context.get_slave_mut(slave).name_mut().clear();
        write!(
            context.get_slave_mut(slave).name_mut(),
            "? M:{manufacturer:8.8x} I:{id:8.8x}",
        )
        .unwrap();
    }

    // SII SM section
    *context.eep_sync_manager_mut() = EepromSyncManager::sii_sm(context, slave)?;
    let number_sm = context.eep_sync_manager().sync_manager_count();

    if number_sm > 0 {
        *context
            .get_slave_mut(slave)
            .get_sync_manager_mut(0)
            .start_address_mut() = Ethercat::from_host(context.eep_sync_manager().phase_start());
        *context
            .get_slave_mut(slave)
            .get_sync_manager_mut(0)
            .sm_length_mut() = Ethercat::from_host(context.eep_sync_manager().phase_length());
        *context
            .get_slave_mut(slave)
            .get_sync_manager_mut(0)
            .sm_flags_mut() = Ethercat::from_host(
            u32::from(context.eep_sync_manager().control_register())
                + (u32::from(context.eep_sync_manager().activate()) << 16),
        );
        let mut sync_manager_count = 1;
        let mut last_eep_sync_manager = context.eep_sync_manager().clone();
        while let Ok(eep_sync_manager) =
            last_eep_sync_manager.sii_sm_next(context, slave, sync_manager_count)
        {
            *context
                .get_slave_mut(slave)
                .get_sync_manager_mut(sync_manager_count)
                .start_address_mut() = Ethercat::from_host(eep_sync_manager.phase_start());
            *context
                .get_slave(slave)
                .get_sync_manager(sync_manager_count)
                .sm_length_mut() = Ethercat::from_host(eep_sync_manager.phase_length());
            *context
                .get_slave_mut(slave)
                .get_sync_manager_mut(sync_manager_count)
                .sm_flags_mut() = Ethercat::from_host(
                u32::from(eep_sync_manager.control_register())
                    + (u32::from(eep_sync_manager.activate()) << 16),
            );
            sync_manager_count += 1;
            last_eep_sync_manager = eep_sync_manager;
        }
        *context.eep_sync_manager_mut() = last_eep_sync_manager;
    }

    *context.eep_fmmu_mut() = EepromFmmu::sii_fmmu(context, slave)?;
    if context.eep_fmmu().fmmu_count() != 0 {
        if context.eep_fmmu().fmmu()[0] != 0xFF {
            *context.get_slave_mut(slave).fmmu0_function_mut() = context.eep_fmmu().fmmu()[0];
        }
        if context.eep_fmmu().fmmu()[1] != 0xFF {
            *context.get_slave_mut(slave).fmmu1_function_mut() = context.eep_fmmu().fmmu()[1];
        }
        if context.eep_fmmu().fmmu()[2] != 0xFF {
            *context.get_slave_mut(slave).fmmu2_function_mut() = context.eep_fmmu().fmmu()[2];
        }
        if context.eep_fmmu().fmmu()[3] != 0xFF {
            *context.get_slave_mut(slave).fmmu3_function_mut() = context.eep_fmmu().fmmu()[3];
        }
    }
    Ok(())
}

fn configure_sync_manager(
    context: &mut Context,
    slave: u16,
    config_address: u16,
) -> Result<(), MainError> {
    let slave_object = context.get_slave_mut(slave);

    // Should never happen
    if slave_object.get_sync_manager(0).start_address().is_zero() {
        ec_println!("Slave {slave} has no proper mailbox in configuration, try default.");
        *slave_object.get_sync_manager(0).start_address_mut() = Ethercat::from_host(0x1000);
        *slave_object.get_sync_manager(0).sm_length_mut() = Ethercat::from_host(0x80);
        *slave_object.get_sync_manager(0).sm_flags_mut() = Ethercat::from_host(DEFAULT_MAILBOX_SM0);
        *slave_object.get_sync_manager_type_mut(0) = SyncManagerType::MailboxWrite;
    }

    // Should never happen
    if slave_object.get_sync_manager(1).start_address().is_zero() {
        ec_println!("Slave {slave} has no proper mailbox out configuration, try default.");
        *slave_object.get_sync_manager(1).start_address_mut() = Ethercat::from_host(0x1080);
        *slave_object.get_sync_manager(1).sm_length_mut() = Ethercat::from_host(0x80);
        *slave_object.get_sync_manager(1).sm_flags_mut() = Ethercat::from_host(DEFAULT_MAILBOX_SM1);
        *slave_object.get_sync_manager_type_mut(1) = SyncManagerType::MailboxRead;
    }

    // Program SM0 mailbox in and SM1 mailbox out for slave.
    // Writing both sm in one datagram will solve timing issue in old NETX
    let sync_manager = context.get_slave(slave).get_sync_manager(0);
    fpwr(
        context.port_mut(),
        config_address,
        EthercatRegister::SyncManager0 as u16,
        &sync_manager.bytes()?,
        TIMEOUT_RETURN3,
    )?;
    Ok(())
}

/// Enumerate and initialize all slaves
///
/// # Parameters
/// - `context`: Context struct
/// - `usetable`: true when using configtable to initialize slaves, false otherwise
///
///
/// # Errors
/// Returns an error if:
/// - Failed to detect slaves
/// - Failed to request interface type of slave
/// - Failed to set node address of slave
/// - Failed to set behaviour of slave when it receives a non-ecat frame
/// - Failed to initiate a parallel EEPROM request
///
/// # Returns
/// Number of slaves found or error
#[expect(clippy::missing_panics_doc)]
pub fn config_init(context: &mut Context, use_table: bool) -> Result<u16, ConfigError> {
    ec_println!("ec_config_init {use_table}");
    init_context(context)?;
    let slave_count = detect_slaves(context)?;

    set_slaves_to_default(context)?;
    let mut eeprom_requests = (0..context.slave_count())
        .map(|slave| {
            let address_position = 1u16.wrapping_sub(slave);

            {
                // Read interface type of slave
                let val16 = aprdw(
                    context.port_mut(),
                    address_position,
                    EthercatRegister::ProcessDataInterfaceControl,
                    TIMEOUT_RETURN3,
                )?;
                *context.get_slave_mut(slave).interface_type_mut() = val16.to_host();

                // A node offset is used to improve readability of network frames.
                // This has no impact on the number of addressable slaves (auto wrap around)

                // Set node address of slave
                apwrw(
                    context.port_mut(),
                    address_position,
                    EthercatRegister::StaDr,
                    Ethercat::from_host(slave + NODE_OFFSET),
                    TIMEOUT_RETURN3,
                )?;

                // Kill non ecat frames for first slave, pass all frames for following slaves
                let pass_ecat_frames_only = slave == 1;

                // Set non ecat frame behaviour
                apwrw(
                    context.port_mut(),
                    address_position,
                    EthercatRegister::DeviceLayerControl,
                    Ethercat::from_host(u16::from(pass_ecat_frames_only)),
                    TIMEOUT_RETURN3,
                )?;
                let config_address = aprdw(
                    context.port_mut(),
                    address_position,
                    EthercatRegister::StaDr,
                    TIMEOUT_RETURN3,
                )?
                .to_host();
                *context.get_slave_mut(slave).config_address_mut() = config_address;

                *context.get_slave_mut(slave).alias_address_mut() = fprdw(
                    context.port_mut(),
                    config_address,
                    EthercatRegister::Alias,
                    TIMEOUT_RETURN3,
                )?
                .to_host();

                let eeprom_state = fprdw(
                    context.port_mut(),
                    config_address,
                    EthercatRegister::EepromControlStat,
                    TIMEOUT_RETURN3,
                )?
                .to_host();
                *context.get_slave_mut(slave).eeprom_mut().read_size_mut() =
                    if eeprom_state & EEPROM_STATE_MACHINE_READ64 != 0 {
                        EepReadSize::Bytes8
                    } else {
                        EepReadSize::Bytes4
                    };
            }

            // Read the manufacturer of the slave from it's EEPROM
            EepromRequest::request_eeprom_data(context, slave, SiiGeneralItem::Manufacturer)
        })
        .collect::<Result<VecDeque<EepromRequest>, MainError>>()?;

    // Store manufacturer and request id
    eeprom_requests = async_eeprom_read(
        context,
        SiiGeneralItem::Id,
        eeprom_requests,
        |slave, manufacturer| {
            *slave.eeprom_mut().manufacturer_mut() = manufacturer.to_host();
        },
    )?;

    // Store ID and request revision
    eeprom_requests = async_eeprom_read(
        context,
        SiiGeneralItem::Revision,
        eeprom_requests,
        |slave, id| {
            *slave.eeprom_mut().id_mut() = id.to_host();
        },
    )?;

    // Store revision and request mailbox address + mailbox size
    eeprom_requests = async_eeprom_read(
        context,
        SiiGeneralItem::RxMailboxAddress,
        eeprom_requests,
        |slave, revision| {
            *slave.eeprom_mut().revision_mut() = revision.to_host();
        },
    )?;

    for slave in 1..context.slave_count() {
        // Mailbox address and mailbox size
        let mailbox_rx = eeprom_requests
            .pop_front()
            .unwrap()
            .read_eeprom_data(context, TIMEOUT_EEPROM)?;
        *context.get_slave_mut(slave).mailbox_mut().read_offset_mut() =
            low_word(mailbox_rx.to_host());
        *context.get_slave_mut(slave).mailbox_mut().length_mut() = high_word(mailbox_rx.to_host());

        if context.get_slave(slave).mailbox().length() > 0 {
            // Read mailbox offset
            eeprom_requests.push_back(EepromRequest::request_eeprom_data(
                context,
                slave,
                SiiGeneralItem::TxMailboxAddress,
            )?);
        }
    }

    for slave in 1..context.slave_count() {
        if context.get_slave(slave).mailbox().length() > 0 {
            read_mailbox_info(context, slave, &mut eeprom_requests)?;
        }
        let config_address = context.get_slave(slave).config_address();
        let escsup = fprdw(
            context.port_mut(),
            config_address,
            EthercatRegister::EscSup,
            TIMEOUT_RETURN3,
        )?;

        // Check whether the slave supports distributed clock
        if context.get_slave(slave).distributed_clock().is_none() && escsup.to_host() & 4 > 0 {
            context
                .get_slave_mut(slave)
                .set_distributed_clock(DistributedClock::default());
        }

        read_topology(context, config_address, slave)?;

        // Physical type
        *context.get_slave_mut(slave).physical_type_mut() = low_byte(
            fprdw(
                context.port_mut(),
                config_address,
                EthercatRegister::PortDescriptor,
                TIMEOUT_RETURN3,
            )?
            .to_host(),
        );

        search_parent(context, slave);

        // Check state change init
        context.check_state(slave, EthercatState::Init, TIMEOUT_STATE)?;

        // Set default mailbox configuration if slave has mailbox
        if context.get_slave(slave).mailbox().length() > 0 {
            configure_mailbox(context, slave, &mut eeprom_requests)?;
        }

        let config_index = if use_table {
            config_from_table(context, slave)
        } else {
            0
        };

        // Slave not in configuration table, find out via SII
        if config_index == 0 && !lookup_previous_sii(context, slave) {
            configure_slave_with_sii(context, slave)?;
        }
        if context.get_slave(slave).mailbox().length() > 0 {
            configure_sync_manager(context, slave, config_address)?;
        }

        // Some slaves need eeprom available to PDI in init -> preop transition
        context.eeprom_to_pdi(slave)?;

        // User may override automatic state change
        if !context.manual_state_change() {
            // Request pre-op for slave
            fpwrw(
                context.port_mut(),
                config_address,
                EthercatRegister::ApplicationLayerControl,
                Ethercat::from_host(
                    EthercatState::PreOperational as u16 | EthercatState::Error as u16,
                ),
                TIMEOUT_RETURN3,
            )?;
        }
    }

    Ok(slave_count)
}

/// If slave has SII mapping and the same slave ID done before, use previous mapping.
/// This is safe, because SII mapping is constant for same slave ID.
///
/// # Returns
/// Whether the requested slave config was copied from an other slave.
pub fn lookup_mapping(
    context: &mut Context,
    slave: u16,
    output_size: &mut u32,
    input_size: &mut u32,
) -> bool {
    if slave > 1 && context.slave_count() > 0 {
        let slave_usize = usize::from(slave);
        let requested_slave = &context.get_slave(slave);
        let found_slave = context
            .slavelist()
            .iter()
            .enumerate()
            .take(slave_usize)
            .skip(1)
            .find(|(_, current_slave)| {
                current_slave.eeprom().manufacturer() == requested_slave.eeprom().manufacturer()
                    && current_slave.eeprom().id() == requested_slave.eeprom().id()
                    && current_slave.eeprom().revision() == requested_slave.eeprom().revision()
            })
            .map(|(index, _)| index as u16);
        if let Some(found_slave) = found_slave {
            for nsm in 0..MAX_SM {
                *context
                    .get_slave_mut(slave)
                    .get_sync_manager_mut(nsm)
                    .sm_length_mut() = context
                    .get_slave(found_slave)
                    .get_sync_manager(nsm)
                    .sm_length();
                *context.get_slave_mut(slave).get_sync_manager_type_mut(nsm) =
                    context.get_slave(found_slave).get_sync_manager_type(nsm);
            }
            *output_size = context.get_slave(found_slave).output_bits().into();
            *input_size = context.get_slave(found_slave).input_bits().into();
            *context.get_slave_mut(slave).output_bits_mut() =
                context.get_slave(found_slave).output_bits();
            *context.get_slave_mut(slave).input_bits_mut() =
                context.get_slave(found_slave).input_bits();
            ec_println!("Copy mapping slave {slave} from {found_slave}.");
            return true;
        }
    }
    false
}

/// # Errors
/// Returns an error if:
/// - The state of the slave couldn't be checked.
/// - PDO map couldn't be read/parsed
pub fn map_coe_soe(
    context: &mut Context,
    slave: u16,
    thread_number: usize,
) -> Result<(), CoEError> {
    // Check state change pre-op
    context.check_state(slave, EthercatState::PreOperational, TIMEOUT_STATE)?;

    ec_println!(
        " >Slave {slave}, configaddr {:x}, state {:2.2x}",
        context.get_slave(slave).config_address(),
        u8::from(context.get_slave(slave).state())
    );

    // Execute special slave configuration hook pre-op to safe-op.

    // Only if registered
    if let Some(po2_so_config) = context.get_slave(slave).po2_so_config() {
        po2_so_config(slave);
    }

    // Only if registered
    if let Some(po2_so_configx) = context.get_slave(slave).po2_so_configx() {
        po2_so_configx(context, slave);
    }

    // If slave not found in configlist, find IO mapping in slave self
    if context.get_slave(slave).config_index() == 0 {
        return Ok(());
    }
    let mut input_size = 0;
    let mut output_size = 0;

    // If slave supports CANopen over EtherCAT
    if context.get_slave(slave).mailbox().protocols()
        & u16::from(MailboxProtocol::CanopenOverEthercat)
        == 0
    {
        let mut initialized = false;

        // If there is complete access
        if context
            .get_slave(slave)
            .protocol_details()
            .canopen_over_ethercat()
            & u8::from(Coedet::Sdoca)
            != 0
        {
            // Read PDO mapping via CANopen over EtherCAT using complete access
            match read_pdo_map_complete_access(
                context,
                slave,
                thread_number,
                &mut output_size,
                &mut input_size,
            ) {
                Err(e) => return Err(e),
                Ok(_) => initialized = true,
            }
        }

        // Complete access not available or failed
        if !initialized {
            // Read PDO mapping via CANopen over EtherCAT
            read_pdo_map(context, slave, &mut output_size, &mut input_size)?;
        }

        ec_println!("  CoE ouput_size:{output_size} input_size: {input_size}");
    }

    // If slave supports Servo over EtherCAT
    if input_size != 0
        && output_size != 0
        && context.get_slave(slave).mailbox().protocols()
            & u16::from(MailboxProtocol::ServoOverEthercat)
            != 0
    {
        // Read AT/MDT mapping via Servo over EtherCAT
        read_id_nmap(context, slave, &mut output_size, &mut input_size)?;
        *context
            .get_slave_mut(slave)
            .get_sync_manager_mut(2)
            .sm_length_mut() = Ethercat::from_host(output_size.div_ceil(8) as u16);
        *context
            .get_slave_mut(slave)
            .get_sync_manager_mut(3)
            .sm_length_mut() = Ethercat::from_host(input_size.div_ceil(8) as u16);
        ec_println!("  SOE output_size:{output_size} input_size:{input_size}");
    }
    *context.get_slave_mut(slave).output_bits_mut() = output_size as u16;
    *context.get_slave_mut(slave).input_bits_mut() = input_size as u16;

    Ok(())
}

fn map_sii(context: &mut Context, slave: u16) -> Result<(), MainError> {
    let mut output_size = u32::from(context.get_slave(slave).output_bits());
    let mut input_size = u32::from(context.get_slave(slave).input_bits());

    // Find PDO in previous slave with same ID
    if input_size == 0 && output_size == 0 {
        lookup_mapping(context, slave, &mut output_size, &mut input_size);
    }

    // Find PDO mapping by SII
    if input_size == 0 && output_size == 0 {
        let eepdo;
        (input_size, eepdo) = EepromPdo::sii_pdo(context, slave, false)?;

        ec_println!("  SII input_size:{input_size}");
        for sm_index in 0..MAX_SM {
            if eepdo.get_sync_manager_bit_size(sm_index) > 0 {
                *context
                    .get_slave_mut(slave)
                    .get_sync_manager_mut(sm_index)
                    .sm_length_mut() =
                    Ethercat::from_host(eepdo.get_sync_manager_bit_size(sm_index).div_ceil(8));
                *context
                    .get_slave_mut(slave)
                    .get_sync_manager_type_mut(sm_index) = SyncManagerType::Inputs;
                ec_println!(
                    "    SM{sm_index} length {}",
                    eepdo.get_sync_manager_bit_size(sm_index)
                );
            }
        }
    }
    *context.get_slave_mut(slave).output_bits_mut() = output_size as u16;
    *context.get_slave_mut(slave).input_bits_mut() = input_size as u16;
    ec_println!("     input_size:{input_size} output_size:{output_size}");
    Ok(())
}

fn map_sync_manager(
    context: &mut Context,
    slave: u16,
    config_address: u16,
    sync_manager_index: u8,
    ethercat_register: EthercatRegister,
) -> Result<(), NicdrvError> {
    if context.get_slave(slave).mailbox().length() == 0
        && !context
            .get_slave(slave)
            .get_sync_manager(sync_manager_index)
            .start_address()
            .is_zero()
    {
        let sync_manager = context
            .get_slave(slave)
            .get_sync_manager(sync_manager_index);
        fpwr(
            context.port_mut(),
            config_address,
            ethercat_register as u16,
            &sync_manager.bytes()?,
            TIMEOUT_RETURN3,
        )?;
        ec_println!(
            "    SM0 type:{:?} StartAddress:{:4.4x} Flags:{:8.8x}",
            context
                .get_slave(slave)
                .get_sync_manager_type(sync_manager_index),
            context
                .get_slave(slave)
                .get_sync_manager(sync_manager_index)
                .start_address()
                .to_host(),
            context
                .get_slave(slave)
                .get_sync_manager(sync_manager_index)
                .sm_flags()
                .to_host()
        );
    }
    Ok(())
}

fn map_sm(context: &mut Context, slave: u16) -> Result<(), NicdrvError> {
    let config_address = context.get_slave(slave).config_address();

    ec_println!("  SM programming");
    map_sync_manager(
        context,
        slave,
        config_address,
        0,
        EthercatRegister::SyncManager0,
    )?;
    map_sync_manager(
        context,
        slave,
        config_address,
        1,
        EthercatRegister::SyncManager1,
    )?;

    // Program SM2 to SMx
    for sm_index in 2..MAX_SM {
        if !context
            .get_slave(slave)
            .get_sync_manager(sm_index)
            .start_address()
            .is_zero()
        {
            // Check if SM length is zero -> clear enable flag
            *context
                .get_slave_mut(slave)
                .get_sync_manager_mut(sm_index)
                .sm_flags_mut() = if context
                .get_slave(slave)
                .get_sync_manager(sm_index)
                .sm_length()
                .is_zero()
            {
                Ethercat::from_host(
                    context
                        .get_slave(slave)
                        .get_sync_manager(sm_index)
                        .sm_flags()
                        .to_host()
                        & SYNC_MANAGER_ENABLE_MASK,
                )
            } else {
                // If SM length is non zero, always set the enable flag
                Ethercat::from_host(
                    context
                        .get_slave(slave)
                        .get_sync_manager(sm_index)
                        .sm_flags()
                        .to_host()
                        | !SYNC_MANAGER_ENABLE_MASK,
                )
            };
            let sync_manager = context.get_slave(slave).get_sync_manager(sm_index);
            fpwr(
                context.port_mut(),
                config_address,
                u16::from(EthercatRegister::SyncManager0)
                    + u16::from(sm_index) * size_of::<SyncManager>() as u16,
                &sync_manager.bytes()?,
                TIMEOUT_RETURN3,
            )?;
            ec_println!(
                "    SM{sm_index} Type:{:?} StartAddress:{:4.4x} Flags:{:8.8x}",
                context.get_slave(slave).get_sync_manager_type(sm_index),
                context
                    .get_slave(slave)
                    .get_sync_manager(sm_index)
                    .start_address()
                    .to_host(),
                context
                    .get_slave(slave)
                    .get_sync_manager(sm_index)
                    .sm_flags()
                    .to_host()
            );
        }
    }
    if context.get_slave(slave).input_bits() > 7 {
        *context.get_slave_mut(slave).input_bytes_mut() =
            context.get_slave(slave).input_bits().div_ceil(8);
    }
    if context.get_slave(slave).output_bits() > 7 {
        *context.get_slave_mut(slave).output_bytes_mut() =
            context.get_slave(slave).output_bits().div_ceil(8);
    }
    Ok(())
}

/// # Errors
/// Returns an error if:
/// - The state of a slave couldn't be checked
/// - A PDO map couldn't be parsed
pub fn config_find_mappings(context: &mut Context, group: u8) -> Result<(), ConfigError> {
    // Find CANopen over EtherCAT and Servo over EtherCAT mapping of slaves in multiple threads
    for slave in 1..context.slave_count() {
        if group == 0 || group == context.get_slave_mut(slave).group() {
            // Serialized version
            map_coe_soe(context, slave, 0)?;
        }
    }

    // Find SII mapping of slave and program
    for slave in 1..context.slave_count() {
        if group == 0 || group == context.get_slave(slave).group() {
            map_sii(context, slave)?;
            map_sm(context, slave)?;
        }
    }
    Ok(())
}

fn fmmu_count(context: &mut Context, slave: u16) -> u8 {
    if context.get_slave(slave).output_bits() != 0 {
        context
            .get_slave(slave)
            .fmmu()
            .iter()
            .enumerate()
            .skip(context.get_slave(slave).fmmu_unused().into())
            .find(|(_, fmmu)| fmmu.log_start().is_zero())
            .map_or(context.get_slave(slave).fmmu().len(), |(index, _)| index) as u8
    } else {
        context.get_slave(slave).fmmu_unused()
    }
}

fn find_input_sync_manager(context: &mut Context, slave: u16, sm_count: &mut u8) {
    *sm_count += 1;
    while *sm_count < MAX_SM - 1
        && context.get_slave(slave).get_sync_manager_type(*sm_count) != SyncManagerType::Inputs
    {
        *sm_count += 1;
    }
}

fn find_fmmu_input_address_end(
    context: &mut Context,
    slave: u16,
    sm_count: &mut u8,
    bit_count: &mut u16,
    sm_length: &mut u16,
    byte_count: &mut u16,
    end_address: &mut u16,
) {
    // More SM for input
    while *bit_count < context.get_slave(slave).input_bits() && *sm_count < MAX_SM - 1 {
        find_input_sync_manager(context, slave, sm_count);

        // If addresses from more SM connect use one FMMU
        if context
            .get_slave(slave)
            .get_sync_manager(*sm_count)
            .start_address()
            .to_host()
            > *end_address
        {
            break;
        }
        ec_println!("      SM{sm_count}");
        *sm_length = context
            .get_slave(slave)
            .get_sync_manager(*sm_count)
            .sm_length()
            .to_host();
        *byte_count += *sm_length;
        *bit_count += *sm_length * 8;
        *end_address = context
            .get_slave(slave)
            .get_sync_manager(*sm_count)
            .start_address()
            .to_host()
            + *sm_length;
    }
}

fn configure_fmmu_input_bit_oriented_slave(
    context: &mut Context,
    slave: u16,
    fmmu_count: u8,
    log_address: &mut u32,
    bit_position: &mut u8,
) -> u16 {
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_start_mut() = Ethercat::from_host(*log_address);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_start_bit_mut() = *bit_position;
    *bit_position += (context.get_slave(slave).input_bits() - 1) as u8;
    if *bit_position > 7 {
        *log_address += 1;
        *bit_position -= 8;
    }
    let fmmu_size = (*log_address
        - context
            .get_slave(slave)
            .get_fmmu(fmmu_count)
            .log_start()
            .to_host()
        + 1) as u16;
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_length_mut() = Ethercat::from_host(fmmu_size);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_end_bit_mut() = *bit_position;
    *bit_position += 1;
    if *bit_position > 7 {
        *log_address += 1;
        *bit_position -= 8;
    }
    fmmu_size
}

fn configure_fmmu_input_byte_oriented_slave(
    context: &mut Context,
    slave: u16,
    fmmu_count: u8,
    log_address: &mut u32,
    bit_position: &mut u8,
    byte_count: u16,
    fmmu_done: u16,
) -> u16 {
    // Byte oriented slave
    if *bit_position != 0 {
        *log_address += 1;
        *bit_position = 0;
    }
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_start_mut() = Ethercat::from_host(*log_address);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_start_bit_mut() = *bit_position;
    *bit_position = 7;
    let fmmu_size = if byte_count + fmmu_done > context.get_slave(slave).input_bytes() {
        context.get_slave(slave).input_bytes() - fmmu_done
    } else {
        byte_count
    };
    *log_address += u32::from(fmmu_size);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_length_mut() = Ethercat::from_host(fmmu_size);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_end_bit_mut() = *bit_position;
    *bit_position = 0;
    fmmu_size
}

fn program_fmmu_for_input(
    context: &mut Context,
    slave: u16,
    config_address: u16,
    fmmu_count: u8,
    add_to_inputs_wkc: &mut bool,
) -> Result<(), NicdrvError> {
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .physical_start_bit_mut() = 0;
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .fmmu_type_mut() = 1;
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .fmmu_active_mut() = true;

    // Program FMMU for input
    let fmmu = context.get_slave(slave).get_fmmu(fmmu_count);
    fpwr(
        context.port_mut(),
        config_address,
        EthercatRegister::FieldbusMemoryManagementUnit0 as u16
            + u16::from(fmmu_count) * size_of::<Fmmu>() as u16,
        &fmmu.bytes()?,
        TIMEOUT_RETURN3,
    )?;

    // Set flag to add one for an input FMMU,
    // a single ESC can only contribute once.
    *add_to_inputs_wkc = true;
    Ok(())
}

fn configure_fmmu_input_start<'context>(
    context: &mut Context<'context>,
    slave: u16,
    group: u8,
    io_map: &'context [u8],
    fmmu_count: u8,
) {
    *context.get_slave_mut(slave).input_offset_mut() = if group != 0 {
        context
            .get_slave(slave)
            .get_fmmu(fmmu_count)
            .log_start()
            .to_host()
            - context.get_group(group).logical_start_address()
            - u32::from(context.get_slave(slave).output_bytes())
    } else {
        context
            .get_slave(slave)
            .get_fmmu(fmmu_count)
            .log_start()
            .to_host()
            - u32::from(context.get_slave(slave).output_bytes())
    };
    *context.get_slave_mut(slave).io_map_mut() = io_map;

    *context.get_slave_mut(slave).input_startbit_mut() = context
        .get_slave(slave)
        .get_fmmu(fmmu_count)
        .log_start_bit();
    ec_println!(
        "    Inputs {:p} startbit {}",
        context.get_slave(slave).input().unwrap(),
        context.get_slave(slave).input_startbit()
    );
}

/// # Errors
/// Returns an error if:
/// - The mapping couldn't be send to the slave
fn config_create_input_mappings<'a, 'b: 'a>(
    context: &mut Context<'a>,
    io_map: &'b [u8],
    group: u8,
    slave: u16,
    log_address: &mut u32,
    bit_position: &mut u8,
) -> Result<(), NicdrvError> {
    ec_println!(" =Slave {slave}, INPUT MAPPING");

    let config_address = context.get_slave(slave).config_address();

    // Find free FMMU
    let mut fmmu_count = fmmu_count(context, slave);

    // Search for SM's that contribute to the input mapping
    let mut sm_count = 0;
    let mut fmmu_done = 0;
    let mut byte_count = 0;
    let mut bit_count = 0;
    let mut add_to_inputs_wkc = false;
    while sm_count < MAX_SM && fmmu_done < context.get_slave(slave).input_bits().div_ceil(8) {
        ec_println!("    FMMU {fmmu_count}");
        while sm_count < MAX_SM - 1
            && context.get_slave(slave).get_sync_manager_type(sm_count) != SyncManagerType::Inputs
        {
            sm_count += 1;
        }

        ec_println!("      SM{sm_count}");
        *context
            .get_slave_mut(slave)
            .get_fmmu_mut(fmmu_count)
            .physical_start_mut() = context
            .get_slave(slave)
            .get_sync_manager(sm_count)
            .start_address();
        let mut sm_length = context
            .get_slave(slave)
            .get_sync_manager(sm_count)
            .sm_length()
            .to_host();
        byte_count += sm_length;
        bit_count += sm_length * 8;
        let mut end_address = context
            .get_slave(slave)
            .get_sync_manager(sm_count)
            .start_address()
            .to_host()
            + sm_length;

        find_fmmu_input_address_end(
            context,
            slave,
            &mut sm_count,
            &mut bit_count,
            &mut sm_length,
            &mut byte_count,
            &mut end_address,
        );

        // Bit oriented slave
        let fmmu_size = if context.get_slave(slave).input_bytes() == 0 {
            configure_fmmu_input_bit_oriented_slave(
                context,
                slave,
                fmmu_count,
                log_address,
                bit_position,
            )
        } else {
            configure_fmmu_input_byte_oriented_slave(
                context,
                slave,
                fmmu_count,
                log_address,
                bit_position,
                byte_count,
                fmmu_done,
            )
        };

        fmmu_done += fmmu_size;
        if !context
            .get_slave(slave)
            .get_fmmu(fmmu_count)
            .log_length()
            .is_zero()
        {
            program_fmmu_for_input(
                context,
                slave,
                config_address,
                fmmu_count,
                &mut add_to_inputs_wkc,
            )?;
        }
        if context.get_slave(slave).input().is_none() {
            configure_fmmu_input_start(context, slave, group, io_map, fmmu_count);
        }
        fmmu_count += 1;
    }

    *context.get_slave_mut(slave).fmmu_unused_mut() = fmmu_count;

    // Add one WKC for an input if flag is true
    if add_to_inputs_wkc {
        *context.get_group_mut(group).work_counter_inputs_mut() += 1;
    }
    Ok(())
}

fn configure_fmmu_output_bit_oriented_slave(
    context: &mut Context,
    slave: u16,
    log_address: &mut u32,
    bit_position: &mut u8,
    fmmu_count: u8,
) -> u16 {
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_start_mut() = Ethercat::from_host(*log_address);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_start_bit_mut() = *bit_position;
    *bit_position += (context.get_slave(slave).output_bits() - 1) as u8;
    if *bit_position > 7 {
        *log_address += 1;
        *bit_position -= 8;
    }
    let fmmu_size = (*log_address
        - context
            .get_slave(slave)
            .get_fmmu(fmmu_count)
            .log_start()
            .to_host()
        + 1) as u16;
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_length_mut() = Ethercat::from_host(fmmu_size);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_end_bit_mut() = *bit_position;
    *bit_position += 1;
    if *bit_position > 7 {
        *log_address += 1;
        *bit_position -= 8;
    }
    fmmu_size
}

fn configure_fmmu_output_byte_oriented_slave(
    context: &mut Context,
    slave: u16,
    log_address: &mut u32,
    bit_position: &mut u8,
    byte_count: &mut u16,
    fmmu_count: u8,
    fmmu_done: u16,
) -> u16 {
    if *bit_position != 0 {
        *log_address += 1;
        *bit_position = 0;
    }
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_start_mut() = Ethercat::from_host(*log_address);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_start_bit_mut() = *bit_position;
    *bit_position = 7;
    let mut fmmu_size = *byte_count;
    if fmmu_size + fmmu_done > context.get_slave(slave).output_bytes() {
        fmmu_size = context.get_slave(slave).output_bytes() - fmmu_done;
    }
    *log_address += u32::from(fmmu_size);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_length_mut() = Ethercat::from_host(fmmu_size);
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .log_end_bit_mut() = *bit_position;
    *bit_position = 0;
    fmmu_size
}

fn program_fmmu_for_output(
    context: &mut Context,
    slave: u16,
    config_address: u16,
    fmmu_count: u8,
    add_to_outputs_wkc: &mut bool,
) -> Result<(), NicdrvError> {
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .physical_start_bit_mut() = 0;
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .fmmu_type_mut() = 2;
    *context
        .get_slave_mut(slave)
        .get_fmmu_mut(fmmu_count)
        .fmmu_active_mut() = true;

    // Program FMMU for output
    let fmmu = context.get_slave(slave).get_fmmu(fmmu_count);
    fpwr(
        context.port_mut(),
        config_address,
        EthercatRegister::FieldbusMemoryManagementUnit0 as u16
            + u16::from(fmmu_count) * size_of::<Fmmu>() as u16,
        &fmmu.bytes()?,
        TIMEOUT_RETURN3,
    )?;

    // Set flag to add one for an input FMMU.
    // A single ESC can only contribute once.
    *add_to_outputs_wkc = true;
    Ok(())
}

fn find_fmmu_output_address_end(
    context: &mut Context,
    slave: u16,
    bit_count: &mut u16,
    byte_count: &mut u16,
    sm_count: &mut u8,
    sm_length: &mut u16,
    end_address: &mut u16,
) {
    while *bit_count < context.get_slave(slave).output_bits() && *sm_count < MAX_SM - 1 {
        *sm_count += 1;
        while *sm_count < MAX_SM - 1
            && context.get_slave(slave).get_sync_manager_type(*sm_count) != SyncManagerType::Outputs
        {
            *sm_count += 1;
        }

        // If addresses from more SM connect use one FMMU, otherwise break up in multiple FMMU
        if context
            .get_slave(slave)
            .get_sync_manager(*sm_count)
            .sm_length()
            .to_host()
            > *end_address
        {
            break;
        }

        ec_println!("      SM{sm_count}");
        *sm_length = context
            .get_slave(slave)
            .get_sync_manager(*sm_count)
            .sm_length()
            .to_host();
        *byte_count += *sm_length;
        *bit_count += *sm_length;
        *end_address = context
            .get_slave(slave)
            .get_sync_manager(*sm_count)
            .start_address()
            .to_host();
    }
}

fn configure_fmmu_output_start<'context>(
    context: &mut Context<'context>,
    slave: u16,
    io_map: &'context [u8],
    group: u8,
    fmmu_count: u8,
) {
    *context.get_slave_mut(slave).io_map_mut() = io_map;
    *context.get_slave_mut(slave).output_offset_mut() = if group != 0 {
        context
            .get_slave(slave)
            .get_fmmu(fmmu_count)
            .log_start()
            .to_host()
            - context.get_group(group).logical_start_address()
    } else {
        context
            .get_slave(slave)
            .get_fmmu(fmmu_count)
            .log_start()
            .to_host()
    };

    *context.get_slave_mut(slave).output_startbit_mut() = context
        .get_slave(slave)
        .get_fmmu(fmmu_count)
        .log_start_bit();
    ec_println!(
        "    slave {slave} Outputs {:p} startbit {}",
        context.get_slave(slave).outputs().unwrap(),
        context.get_slave(slave).output_startbit()
    );
}

fn config_create_output_mappings<'a, 'b: 'a>(
    context: &mut Context<'a>,
    io_map: &'b [u8],
    group: u8,
    slave: u16,
    log_address: &mut u32,
    bit_position: &mut u8,
) -> Result<(), NicdrvError> {
    let mut bit_count = 0;
    let mut fmmu_done = 0;
    let mut add_to_outputs_wkc = false;
    let mut byte_count = 0;
    let mut sm_count = 0;

    ec_println!("  OUTPUT MAPPING");

    let mut fmmu_count = context.get_slave(slave).fmmu_unused();
    let config_address = context.get_slave(slave).config_address();

    // Search for SM's that contribute to the output mapping
    while sm_count < MAX_SM && fmmu_done < context.get_slave(slave).output_bits().div_ceil(8) {
        ec_println!("    FMMU {fmmu_count}");
        while sm_count < MAX_SM - 1
            && context.get_slave(slave).get_sync_manager_type(sm_count) != SyncManagerType::Outputs
        {
            sm_count += 1;
        }
        ec_println!("      SM{sm_count}");
        *context
            .get_slave_mut(slave)
            .get_fmmu_mut(fmmu_count)
            .physical_start_mut() = context
            .get_slave(slave)
            .get_sync_manager(sm_count)
            .start_address();
        let mut sm_length = context
            .get_slave(slave)
            .get_sync_manager(sm_count)
            .sm_length()
            .to_host();
        byte_count += sm_length;
        bit_count += sm_length * 8;
        let mut end_address = context
            .get_slave(slave)
            .get_sync_manager(sm_count)
            .start_address()
            .to_host()
            + sm_length;

        find_fmmu_output_address_end(
            context,
            slave,
            &mut bit_count,
            &mut byte_count,
            &mut sm_count,
            &mut sm_length,
            &mut end_address,
        );

        // Bit oriented slave
        let fmmu_size = if context.get_slave(slave).output_bytes() == 0 {
            configure_fmmu_output_bit_oriented_slave(
                context,
                slave,
                log_address,
                bit_position,
                fmmu_count,
            )
        } else {
            // Byte oriented slave
            configure_fmmu_output_byte_oriented_slave(
                context,
                slave,
                log_address,
                bit_position,
                &mut byte_count,
                fmmu_count,
                fmmu_done,
            )
        };
        fmmu_done += fmmu_size;

        if !context
            .get_slave(slave)
            .get_fmmu(fmmu_count)
            .log_length()
            .is_zero()
        {
            program_fmmu_for_output(
                context,
                slave,
                config_address,
                fmmu_count,
                &mut add_to_outputs_wkc,
            )?;
        }

        if context.get_slave(slave).outputs().is_none() {
            configure_fmmu_output_start(context, slave, io_map, group, fmmu_count);
        }
        fmmu_count += 1;
    }
    *context.get_slave_mut(slave).fmmu_unused_mut() = fmmu_count;

    // Add one WKC for an output if needed
    if add_to_outputs_wkc {
        *context.get_group_mut(group).work_counter_outputs_mut() += 1;
    }

    Ok(())
}

fn main_config_map_group<'context, 'io_map: 'context>(
    context: &mut Context<'context>,
    io_map: &'io_map mut [u8],
    group: u8,
    force_byte_alignment: bool,
) -> Result<u32, ConfigError> {
    let mut segment_size: u16 = 0;
    let mut current_segment: u16 = 0;
    if context.slave_count() == 0 || u32::from(group) >= context.max_group() {
        return Ok(0);
    }
    ec_println!("ec_config_map_group IOmap:{io_map:p} group:{group}");
    let mut logical_address = context.get_group(group).logical_start_address();
    let mut logical_address2 = logical_address;
    let mut bit_position = 0;
    *context.get_group_mut(group).used_segment_count_mut() = 0;
    *context.get_group_mut(group).work_counter_outputs_mut() = 0;
    *context.get_group_mut(group).work_counter_inputs_mut() = 0;

    // Find mappings and program synchronization managers
    config_find_mappings(context, group)?;

    // Do output mapping of slave and program FMMUs
    for slave in 1..context.slave_count() {
        if group == 0 || group == context.get_slave(slave).group() {
            // Create output mapping
            if context.get_slave(slave).output_bits() != 0 {
                config_create_output_mappings(
                    context,
                    io_map,
                    group,
                    slave,
                    &mut logical_address,
                    &mut bit_position,
                )?;

                // Force byte alignment if the output < 8 bits
                if force_byte_alignment && bit_position != 0 {
                    logical_address += 1;
                    bit_position = 0;
                }

                let difference = (logical_address - logical_address2) as u16;
                logical_address2 = logical_address;

                if u32::from(segment_size) + u32::from(difference)
                    > (MAX_LRW_DATA_LENGTH - FIRST_DC_DATAGRAM_SIZE) as u32
                {
                    *context
                        .get_group_mut(group)
                        .get_io_segment_mut(current_segment) = u32::from(segment_size);
                    if usize::from(current_segment) < MAX_IO_SEGMENTS - 1 {
                        current_segment += 1;
                        segment_size = difference;
                    }
                } else {
                    segment_size += difference;
                }
            }
        }
    }
    if bit_position != 0 {
        logical_address += 1;
        logical_address2 = logical_address;
        bit_position = 0;
        if u32::from(segment_size) + 1 > (MAX_LRW_DATA_LENGTH - FIRST_DC_DATAGRAM_SIZE) as u32 {
            *context
                .get_group_mut(group)
                .get_io_segment_mut(current_segment) = u32::from(segment_size);
            if usize::from(current_segment) < MAX_IO_SEGMENTS - 1 {
                current_segment += 1;
                segment_size = 1;
            }
        } else {
            segment_size += 1;
        }
    }
    *context.get_group_mut(group).io_map_mut() = io_map;
    *context.get_group_mut(group).output_bytes_mut() =
        logical_address - context.get_group(group).logical_start_address();
    *context.get_group_mut(group).used_segment_count_mut() = current_segment + 1;
    *context.get_group_mut(group).first_input_segment_mut() = current_segment;
    *context.get_group_mut(group).input_offset_mut() = segment_size;
    if group == 0 {
        *context.get_slave_mut(0).io_map_mut() = io_map;

        // Store output bytes in master record
        *context.get_slave_mut(0).output_bytes_mut() =
            (logical_address - context.get_group(group).logical_start_address()) as u16;
    }

    // Input mapping of slave and program FMMUs
    for slave in 1..context.slave_count() {
        let config_address = context.get_slave(slave).config_address();
        if group != 0 && group != context.get_slave(slave).group() {
            continue;
        }
        // Create input mapping
        if context.get_slave(slave).input_bits() != 0 {
            config_create_input_mappings(
                context,
                io_map,
                group,
                slave,
                &mut logical_address,
                &mut bit_position,
            )?;

            // Force byte alignment if the input < 8 bits
            if force_byte_alignment && bit_position != 0 {
                logical_address += 1;
                bit_position = 0;
            }

            let difference = logical_address - logical_address2;

            if u32::from(segment_size) + difference
                > (MAX_LRW_DATA_LENGTH - FIRST_DC_DATAGRAM_SIZE) as u32
            {
                *context
                    .get_group_mut(group)
                    .get_io_segment_mut(current_segment) = u32::from(segment_size);
                if usize::from(current_segment) < MAX_IO_SEGMENTS - 1 {
                    current_segment += 1;
                    segment_size = difference as u16;
                }
            } else {
                segment_size += difference as u16;
            }
        }

        // Set eeprom control to PDI
        context.eeprom_to_pdi(slave)?;

        // User may override automatic state change
        if !context.manual_state_change() {
            // Request safe operation for slave
            fpwrw(
                context.port_mut(),
                config_address,
                EthercatRegister::ApplicationLayerControl,
                Ethercat::from_host(u16::from(EthercatState::Operational)),
                TIMEOUT_RETURN3,
            )?;
        }

        if context.get_slave(slave).block_logical_read_write() != 0 {
            *context.get_group_mut(group).block_logical_read_write_mut() += 1;
        }
        *context.get_group_mut(group).ebus_current_mut() += context.get_slave(slave).ebus_current();
    }

    if bit_position != 0 {
        logical_address += 1;
        if usize::from(segment_size) + 1 > MAX_LRW_DATA_LENGTH - FIRST_DC_DATAGRAM_SIZE {
            *context
                .get_group_mut(group)
                .get_io_segment_mut(current_segment) = u32::from(segment_size);
            if usize::from(current_segment) < MAX_IO_SEGMENTS - 1 {
                current_segment += 1;
                segment_size = 1;
            }
        } else {
            segment_size += 1;
        }
    }

    *context
        .get_group_mut(group)
        .get_io_segment_mut(current_segment) = u32::from(segment_size);
    *context.get_group_mut(group).used_segment_count_mut() = current_segment + 1;
    *context.get_group_mut(group).input_bytes_mut() = logical_address
        - context.get_group_mut(group).logical_start_address()
        - context.get_group_mut(group).output_bytes();

    if group == 0 {
        // Store input bytes in master record
        *context.get_slave_mut(0).input_offset_mut() = 0;
        *context.get_slave_mut(0).input_bytes_mut() = (logical_address
            - context.get_group(group).logical_start_address()
            - u32::from(context.get_slave(0).output_bytes()))
            as u16;
    }

    ec_println!(
        "IOmapSize {}",
        logical_address - context.get_group(group).logical_start_address()
    );

    Ok(logical_address - context.get_group(group).logical_start_address())
}

/// Map all PDOs in one group of slaves to IO map with outputs/inputs
/// in sequential order (legacy soem way).
///
/// # Parameters
/// - `context`: Context struct
/// - `io_map`: IOmap
/// - `group`: Group to map, 0 is all groups
///
/// # Errors
/// Returns an error if the main config map group couldn't be configured.
///
/// # Returns
/// IOmap size or error
pub fn config_map_group<'context, 'io_map: 'context>(
    context: &mut Context<'context>,
    io_map: &'io_map mut [u8],
    group: u8,
) -> Result<u32, ConfigError> {
    main_config_map_group(context, io_map, group, false)
}

/// Map all PDOs in one group of slaves to IOmap with outputs/inputs
/// in sequential order (legacy SOEM way) and force byte alignment
///
/// # Parameters
/// - `context`: Context struct
/// - `io_map`: IOmap
/// - `group`: Group to map, 0 = all groups
///
/// # Errors
/// Returns an error if the main config map group couldn't be configured.
///
/// # Returns
/// IOmap size or error
pub fn config_map_group_aligned<'context, 'io_map: 'context>(
    context: &mut Context<'context>,
    io_map: &'io_map mut [u8],
    group: u8,
) -> Result<u32, ConfigError> {
    main_config_map_group(context, io_map, group, true)
}

/// Map all PDOs in one group of slaves to IOmap with outputs/inputs
/// overlapping. NOTE: Must use this for TI ESC when using LRW.
///
/// # Parameters
/// - `context`: Context struct
/// - `io_map`: Pointer to IOmap
/// - `group`: Group to map, 0 = all groups
///
/// # Errors
/// Errors if:
/// - The config couldn't be send to a slave.
/// - The slave couldn't be set to the safe operational state
///
/// # Returns
/// IOmap size or error
#[expect(clippy::similar_names)]
pub fn config_overlap_map_group<'context, 'io_map: 'context>(
    context: &mut Context<'context>,
    io_map: &'io_map mut [u8],
    group: u8,
) -> Result<usize, ConfigError> {
    let mut m_logical_address;
    let mut si_logical_address;
    let mut so_logical_address;
    let mut current_segment = 0;
    let mut segment_size = 0;

    if context.slave_count() > 0 && u32::from(group) < context.max_group() {
        ec_println!("ec_config_map_group IOmap:{io_map:p} group:{group}");
        m_logical_address = u32::from(context.get_group(group).used_segment_count());
        si_logical_address = m_logical_address;
        so_logical_address = m_logical_address;
        let mut bit_position = 0;
        *context.get_group_mut(group).used_segment_count_mut() = 0;
        *context.get_group_mut(group).work_counter_outputs_mut() = 0;
        *context.get_group_mut(group).work_counter_inputs_mut() = 0;

        // Find mappings and program syncmanagers
        config_find_mappings(context, group)?;

        // Do IO mapping of slave and program FMMUs
        for slave in 1..context.slave_count() {
            let config_address = context.get_slave(slave).config_address();
            (si_logical_address, so_logical_address) = (m_logical_address, m_logical_address);

            if group == 0 || group == context.get_slave(slave).group() {
                // Create output mapping
                if context.get_slave(slave).output_bits() != 0 {
                    config_create_output_mappings(
                        context,
                        io_map,
                        group,
                        slave,
                        &mut so_logical_address,
                        &mut bit_position,
                    )?;

                    if bit_position != 0 {
                        so_logical_address += 1;
                        bit_position = 0;
                    }
                }

                // Create input mapping
                if context.get_slave(slave).input_bits() != 0 {
                    config_create_input_mappings(
                        context,
                        io_map,
                        group,
                        slave,
                        &mut si_logical_address,
                        &mut bit_position,
                    )?;

                    if bit_position != 0 {
                        si_logical_address += 1;
                        bit_position = 0;
                    }
                }

                let temp_logical_address = si_logical_address.max(so_logical_address);
                let difference = temp_logical_address - m_logical_address;
                m_logical_address = temp_logical_address;

                if (segment_size + difference) as usize
                    > (MAX_LRW_DATA_LENGTH - FIRST_DC_DATAGRAM_SIZE)
                {
                    *context
                        .get_group_mut(group)
                        .get_io_segment_mut(current_segment) = segment_size;
                    if usize::from(current_segment) <= MAX_IO_SEGMENTS {
                        current_segment += 1;
                        segment_size = difference;
                    }
                } else {
                    segment_size += difference;
                }

                // Set EEPROM control to PDI
                context.eeprom_to_pdi(slave)?;

                // User may override automatic state change
                if !context.manual_state_change() {
                    // Request safe operation for slave
                    fpwrw(
                        context.port_mut(),
                        config_address,
                        EthercatRegister::ApplicationLayerControl,
                        Ethercat::from_host(u16::from(EthercatState::SafeOperational)),
                        TIMEOUT_RETURN3,
                    )?;
                }
                if context.get_slave(slave).block_logical_read_write() != 0 {
                    *context.get_group_mut(group).block_logical_read_write_mut() += 1;
                }

                *context.get_group_mut(group).ebus_current_mut() +=
                    context.get_slave(slave).ebus_current();
            }
        }

        *context
            .get_group_mut(group)
            .get_io_segment_mut(current_segment) = segment_size;
        *context.get_group_mut(group).used_segment_count_mut() = current_segment + 1;
        *context.get_group_mut(group).first_input_segment_mut() = 0;
        *context.get_group_mut(group).first_input_segment_mut() = 0;

        *context.get_group_mut(group).output_bytes_mut() =
            so_logical_address - context.get_group(group).logical_start_address();
        *context.get_group_mut(group).input_bytes_mut() =
            si_logical_address - context.get_group(group).logical_start_address();
        *context.get_group_mut(group).io_map_mut() = io_map;

        // Move calculated inputs with output_bytes offset
        for slave in 1..=context.slave_count() {
            if (group == 0 || group == context.get_slave(slave).group())
                && context.get_slave(slave).input_bits() > 0
            {
                *context.get_slave_mut(slave).input_offset_mut() = 0;
            }
        }

        if group == 0 {
            // Store output bytes in master record
            *context.get_slave_mut(0).io_map_mut() = io_map;
            *context.get_slave_mut(0).output_bytes_mut() =
                (so_logical_address - context.get_group(group).logical_start_address()) as u16;
            *context.get_slave_mut(0).input_offset_mut() = 0;
            *context.get_slave_mut(0).input_bytes_mut() =
                (si_logical_address - context.get_group(group).logical_start_address()) as u16;
        }

        let io_map_size =
            context.get_group(group).output_bytes() + context.get_group(group).input_bytes();
        ec_println!("IOmapSize {io_map_size}");

        Ok(io_map_size as usize)
    } else {
        Ok(0)
    }
}

/// Recover slave
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave to recover
/// - `timeout`: local timeout f.e. `TIMEOUT_RETURN3`
///
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
/// - Master couldn't take control of the eeprom
///
/// # Returns
/// `Ok(())` or Error
pub fn recover_slave(
    context: &mut Context,
    slave: u16,
    timeout: Duration,
) -> Result<(), ConfigError> {
    let config_address = context.get_slave(slave).config_address();
    let address_ph = 1_u16.wrapping_sub(slave);

    // Check if another slave than requested has been found
    let read_address: u16 = 0xFFFE;
    let mut word = read_address.to_ne_bytes();
    aprd(
        context.port_mut(),
        address_ph,
        EthercatRegister::StaDr.into(),
        &mut word,
        timeout,
    )?;
    let read_address = u16::from_ne_bytes(word);

    // Correct slave found, finished
    if read_address == config_address {
        return Ok(());
    }

    // Only try if no config address was found
    if read_address == 0 {
        // Clear possible slaves at `TEMP_NODE`.
        fpwrw(
            context.port_mut(),
            TEMP_NODE,
            EthercatRegister::StaDr,
            Ethercat::from_host(0),
            Duration::default(),
        )?;

        // Set temporary node address of slave
        if apwrw(
            context.port_mut(),
            address_ph,
            EthercatRegister::StaDr,
            Ethercat::from_host(TEMP_NODE),
            timeout,
        )
        .is_err()
        {
            fpwrw(
                context.port_mut(),
                TEMP_NODE,
                EthercatRegister::StaDr,
                Ethercat::from_host(0),
                Duration::default(),
            )?;
            return Err(ConfigError::SlaveFailsToRespond);
        }

        // Temporary config address
        *context.get_slave_mut(slave).config_address_mut() = TEMP_NODE;

        // Set eeprom control to master
        context.eeprom_to_master(slave)?;

        // Check if slave is the same as configured before
        if fprdw(
            context.port_mut(),
            TEMP_NODE,
            EthercatRegister::Alias,
            timeout,
        )? == Ethercat::from_host(context.get_slave_mut(slave).alias_address())
            && context.read_eeprom(slave, SiiGeneralItem::Id.into(), TIMEOUT_EEPROM)?
                == Ethercat::from_host(context.get_slave(slave).eeprom().id())
            && context.read_eeprom(slave, SiiGeneralItem::Manufacturer.into(), TIMEOUT_EEPROM)?
                == Ethercat::from_host(context.get_slave(slave).eeprom().manufacturer())
            && context.read_eeprom(slave, SiiGeneralItem::Revision.into(), TIMEOUT_EEPROM)?
                == Ethercat::from_host(context.get_slave(slave).eeprom().revision())
        {
            fpwrw(
                context.port_mut(),
                TEMP_NODE,
                EthercatRegister::StaDr,
                Ethercat::from_host(config_address),
                timeout,
            )?;
            *context.get_slave_mut(slave).config_address_mut() = config_address;
        } else {
            // Slave is not the expected one, remove config address
            fpwrw(
                context.port_mut(),
                TEMP_NODE,
                EthercatRegister::StaDr,
                Ethercat::from_host(0),
                timeout,
            )?;
            *context.get_slave_mut(slave).config_address_mut() = config_address;
            return Err(ConfigError::FoundWrongSlave);
        }
    }
    Ok(())
}

/// Reconfigure slave
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave to reconfigure
/// - `timeout`: Local timeout f.e. `TIMEOUT_RETURN3`
///
/// # Errors
/// Returns an error if:
/// - If the application layer couldn't be set to an ethercat state
/// - The master couldn't take control of the eeprom
/// - A message couldn't be send/received
/// - The slave state couldn't be checked
///
/// # Returns
/// slave state or error
pub fn reconfig_slave(
    context: &mut Context,
    slave: u16,
    timeout: Duration,
) -> Result<u16, ConfigError> {
    let config_address = context.get_slave(slave).config_address();
    fpwrw(
        context.port_mut(),
        config_address,
        EthercatRegister::ApplicationLayerControl,
        Ethercat::from_host(EthercatState::Init.into()),
        timeout,
    )?;

    // Set EEPROM to PDI
    context.eeprom_to_pdi(slave)?;

    // Check state change init
    let mut state = context.check_state(slave, EthercatState::Init, TIMEOUT_STATE)?;
    if state == EthercatState::Init.into() {
        // Program all enabled Sync Managers
        for sm_index in 0..MAX_SM {
            if !context
                .get_slave(slave)
                .get_sync_manager(sm_index)
                .start_address()
                .is_zero()
            {
                let sync_manager = context.get_slave(slave).get_sync_manager(sm_index);
                fpwr(
                    context.port_mut(),
                    config_address,
                    u16::from(EthercatRegister::SyncManager0)
                        + u16::from(sm_index) * size_of::<SyncManager>() as u16,
                    &sync_manager.bytes()?,
                    timeout,
                )?;
            }
        }
        fpwrw(
            context.port_mut(),
            config_address,
            EthercatRegister::ApplicationLayerControl,
            Ethercat::from_host(EthercatState::PreOperational.into()),
            timeout,
        )?;

        // Check state change pre-operational
        if context.check_state(slave, EthercatState::PreOperational, TIMEOUT_STATE)?
            == EthercatState::PreOperational.into()
        {
            // Execute special slave configuration hook pre-operational to safe-operational

            // Only if registered
            if let Some(po2_so_config) = context.get_slave(slave).po2_so_config() {
                po2_so_config(slave);
            }

            // Only if registered
            if let Some(po2_so_configx) = context.get_slave(slave).po2_so_configx() {
                po2_so_configx(context, slave);
            }

            // Set safe operational status
            fpwrw(
                context.port_mut(),
                config_address,
                EthercatRegister::ApplicationLayerControl,
                Ethercat::from_host(EthercatState::SafeOperational.into()),
                timeout,
            )?;

            // Check state change safe operational
            state = context.check_state(slave, EthercatState::SafeOperational, TIMEOUT_STATE)?;

            // Program configured FMMU
            for fmmu_count in 0..context.get_slave(slave).fmmu_unused() {
                let fmmu = context.get_slave(slave).get_fmmu(fmmu_count);
                fpwr(
                    context.port_mut(),
                    config_address,
                    u16::from(EthercatRegister::FieldbusMemoryManagementUnit0)
                        + size_of::<Fmmu>() as u16 * u16::from(fmmu_count),
                    &fmmu.bytes()?,
                    timeout,
                )?;
            }
        }
    }

    Ok(state)
}
