use std::{fmt::Write, time::Duration};

use super::{
    base::aprd,
    coe::CoEError,
    main::{Context, Slave},
    r#type::host_to_ethercat,
};
use crate::{
    ec_println,
    ethercat::{
        base::{aprdw, apwrw, brd, bwr, fprd, fprdw, fpwr, fpwrw},
        coe::{read_pdo_map, read_pdo_map_complete_access},
        main::{
            eeprom_to_master, eeprom_to_pdi, read_eeprom, read_eeprom1, read_eeprom2, sii_find,
            sii_fmmu, sii_get_byte, sii_pdo, sii_sm, sii_sm_next, sii_string, statecheck, Coedet,
            EepReadSize, EepromPdo, Fmmu, MailboxProtocol, SlaveGroup, SyncManager,
            SyncManagerType, MAX_IO_SEGMENTS, MAX_SM, SYNC_MANAGER_ENABLE_MASK,
        },
        r#type::{
            ethercat_to_host, high_word, low_byte, low_word, EthercatRegister, EthercatState,
            SiiCategory, SiiGeneralItem, EEPROM_STATE_MACHINE_READ64, FIRST_DC_DATAGRAM_SIZE,
            LOG_GROUP_OFFSET, MAX_EEP_BUF_SIZE, MAX_LRW_DATA_LENGTH, TIMEOUT_EEP, TIMEOUT_RET3,
            TIMEOUT_SAFE, TIMEOUT_STATE,
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
    SlaveCountExceeded,
    CoEError(CoEError),
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

/// Standard SyncManager0 flags configuration for mailbox slaves
const DEFAULT_MAILBOX_SM0: u32 = 0x00010026;

/// Standard SyncManager1 flags configuration for mailbox slaves
const DEFAULT_MAILBOX_SM1: u32 = 0x00010022;

/// Resets the slavelist and grouplist.
///
/// # Parameters
/// - `context`: The context containing the slavelist and grouplist to reset.
fn init_context(context: &mut Context) {
    context.slave_count = 0;

    // Clean slave list and grouplist
    context.slavelist.clear();
    context.slavelist.reserve(usize::from(context.max_slaves));
    context.grouplist.clear();
    context.grouplist.reserve(context.max_group as usize);

    // Clear slave eeprom cache, doesn't actually read any eeprom
    sii_get_byte(context, 0, MAX_EEP_BUF_SIZE);
    for lp in 0..context.max_group {
        context.grouplist.push(SlaveGroup {
            logical_start_address: lp << LOG_GROUP_OFFSET,
            ..Default::default()
        });
    }
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
        &mut context.port.lock().unwrap(),
        0,
        EthercatRegister::DeviceLayerAlias,
        &mut byte,
        TIMEOUT_RET3,
    )?;

    // Reset all slaves to initialization
    byte[0] = EthercatState::Init as u8 | EthercatState::Error as u8;
    bwr(
        &mut context.port.lock().unwrap(),
        0,
        EthercatRegister::ApplicationLayerControl,
        &mut byte,
        TIMEOUT_RET3,
    )?;
    // NetX100 should now be functional

    // Reset all slaves to initialization
    bwr(
        &mut context.port.lock().unwrap(),
        0,
        EthercatRegister::ApplicationLayerControl,
        &mut byte,
        TIMEOUT_RET3,
    )?;

    // Detect the number of slaves
    let work_counter = brd(
        &mut context.port.lock().unwrap(),
        0,
        EthercatRegister::Type,
        &mut byte,
        TIMEOUT_SAFE,
    )?;

    // This is strictly less than, since the master is slave 0
    if work_counter < context.max_slaves {
        context.slave_count = work_counter;
        Ok(work_counter)
    } else {
        ec_println!(
            "Error: Too many slaves on network: num_slaves={work_counter}, max_slaves={}",
            context.max_slaves
        );
        Err(ConfigError::SlaveCountExceeded)
    }
}

fn set_slaves_to_default(context: &mut Context) -> Result<(), NicdrvError> {
    let mut zero_buffer = [0; 64];
    let mut byte = [0];
    let port = &mut context.port.lock().unwrap();

    // Deactivate loop manual
    bwr(
        port,
        0,
        EthercatRegister::DeviceLayerPort,
        &mut byte,
        TIMEOUT_RET3,
    )?;

    let mut word = host_to_ethercat(4_u16).to_ne_bytes();

    // Set interrupt mask
    bwr(
        port,
        0,
        EthercatRegister::InterruptMask,
        &mut word,
        TIMEOUT_RET3,
    )?;

    // Reset CRC counter
    bwr(
        port,
        0,
        EthercatRegister::ReceiveError,
        &mut zero_buffer[..8],
        TIMEOUT_RET3,
    )?;

    // Reset Fieldbus memory management units
    bwr(
        port,
        0,
        EthercatRegister::FieldbusMemoryManagementUnit0,
        &mut zero_buffer[..16 * 3],
        TIMEOUT_RET3,
    )?;

    // Reset synchronization manager
    bwr(
        port,
        0,
        EthercatRegister::SyncManager0,
        &mut zero_buffer[..8 * 4],
        TIMEOUT_RET3,
    )?;

    // Reset activation register
    byte[0] = 0;
    bwr(
        port,
        0,
        EthercatRegister::DistributedClockSynchronizationActive,
        &mut byte,
        TIMEOUT_RET3,
    )?;

    // Reset system time + offset
    bwr(
        port,
        0,
        EthercatRegister::DistributedClockSystemTime,
        &mut zero_buffer[..4],
        TIMEOUT_RET3,
    )?;

    // Digital Clock speedstart
    word = host_to_ethercat(0x1000u16).to_ne_bytes();
    bwr(
        port,
        0,
        EthercatRegister::DistributedClockSpeedCount,
        &mut word,
        TIMEOUT_RET3,
    )?;

    // Digital clock filter expression
    word = host_to_ethercat(0xC00u16).to_ne_bytes();
    bwr(
        port,
        0,
        EthercatRegister::DistributedClockTimeFilter,
        &mut word,
        TIMEOUT_RET3,
    )?;

    // Ignore alias register
    byte[0] = 0;
    bwr(
        port,
        0,
        EthercatRegister::DeviceLayerAlias,
        &mut byte,
        TIMEOUT_RET3,
    )?;

    // Reset all slaves to initialization
    byte[0] = EthercatState::Init as u8 | EthercatState::Error as u8;
    bwr(
        port,
        0,
        EthercatRegister::ApplicationLayerControl,
        &mut byte,
        TIMEOUT_RET3,
    )?;

    // Force eeprom from PDI
    byte[0] = 2;
    bwr(
        port,
        0,
        EthercatRegister::EepromConfig,
        &mut byte,
        TIMEOUT_RET3,
    )?;

    // Set EEPROM to master
    byte[0] = 0;
    bwr(
        port,
        0,
        EthercatRegister::EepromConfig,
        &mut byte,
        TIMEOUT_RET3,
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
    let slave = usize::from(slave);

    if slave == 0 || context.slave_count == 0 {
        return false;
    }

    let requested_slave = &context.slavelist[slave];
    let found_slave = context
        .slavelist
        .iter()
        .take(slave)
        .enumerate()
        .find(|(_, current_slave)| {
            current_slave.eep_manufacturer == requested_slave.eep_manufacturer
                && current_slave.eep_id == requested_slave.eep_id
                && current_slave.eep_revision == requested_slave.eep_revision
        })
        .map(|(index, _)| index);

    let Some(found_slave) = found_slave else {
        return false;
    };

    let slaves = &mut context.slavelist;
    slaves[slave].canopen_over_ethercat_details = slaves[found_slave].canopen_over_ethercat_details;
    slaves[slave].file_over_ethercat_details = slaves[found_slave].file_over_ethercat_details;
    slaves[slave].ethernet_over_ethercat_details =
        slaves[found_slave].ethernet_over_ethercat_details;
    slaves[slave].servo_over_ethercat_details = slaves[found_slave].servo_over_ethercat_details;

    if slaves[found_slave].block_logical_read_write > 0 {
        slaves[slave].block_logical_read_write = 1;
        slaves[0].block_logical_read_write += 1;
    }

    slaves[slave].ebus_current = slaves[found_slave].ebus_current;
    slaves[0].ebus_current += slaves[slave].ebus_current;
    slaves[slave].name = slaves[found_slave].name.clone();

    for sync_manager_index in 0..usize::from(MAX_SM) {
        slaves[slave].sync_manager[sync_manager_index] =
            slaves[found_slave].sync_manager[sync_manager_index];
    }
    slaves[slave].fmmu0_function = slaves[found_slave].fmmu0_function;
    slaves[slave].fmmu1_function = slaves[found_slave].fmmu1_function;
    slaves[slave].fmmu2_function = slaves[found_slave].fmmu2_function;
    slaves[slave].fmmu3_function = slaves[found_slave].fmmu3_function;
    ec_println!("Copy SII slave {slave} from {found_slave}.");
    true
}

fn async_eeprom_read(
    context: &mut Context,
    next_value: SiiGeneralItem,
    storing_function: impl Fn(&mut Slave, u32),
) {
    for slave in 1..context.slave_count {
        let eedata = read_eeprom2(context, slave, TIMEOUT_EEP);
        storing_function(&mut context.slavelist[usize::from(slave)], eedata);
        read_eeprom1(context, slave, next_value);
    }
}

/// Enumerate and initialize all slaves
///
/// # Parameters
/// - `context`: Context struct
/// - `usetable`: true when using configtable to initialize slaves, false otherwise
///
/// # Returns
/// Number of slaves found or error
pub fn config_init(context: &mut Context, use_table: bool) -> Result<u16, ConfigError> {
    ec_println!("ec_config_init {use_table}");
    init_context(context);
    let slave_count = detect_slaves(context)?;

    set_slaves_to_default(context)?;

    for slave in 0..context.slave_count {
        let slave_usize = usize::from(slave);
        let address_position = 1u16.wrapping_sub(slave);

        {
            let port = &mut context.port.lock().unwrap();

            // Read interface type of slave
            let val16 = aprdw(
                port,
                address_position,
                EthercatRegister::ProcessDataInterfaceControl,
                TIMEOUT_RET3,
            )?;
            context.slavelist[slave_usize].interface_type = ethercat_to_host(val16);

            // A node offset is used to improve readability of network frames.
            // This has no impact on the number of addressable slaves (auto wrap around)

            // Set node address of slave
            apwrw(
                port,
                address_position,
                EthercatRegister::StaDr,
                host_to_ethercat(slave + NODE_OFFSET),
                TIMEOUT_RET3,
            )?;

            // Kill non ecat frames for first slave, pass all frames for following slaves
            let pass_ecat_frames_only = slave == 1;

            // Set non ecat frame behaviour
            apwrw(
                port,
                address_position,
                EthercatRegister::DeviceLayerControl,
                host_to_ethercat(u16::from(pass_ecat_frames_only)),
                TIMEOUT_RET3,
            )?;
            let config_address = ethercat_to_host(aprdw(
                port,
                address_position,
                EthercatRegister::StaDr,
                TIMEOUT_RET3,
            )?);
            context.slavelist[slave_usize].config_address = config_address;

            let mut word = [0; 2];
            fprd(
                port,
                config_address,
                EthercatRegister::Alias,
                &mut word,
                TIMEOUT_RET3,
            )?;
            context.slavelist[slave_usize].alias_address =
                ethercat_to_host(u16::from_ne_bytes(word));

            fprd(
                port,
                config_address,
                EthercatRegister::EepromControlStat,
                &mut word,
                TIMEOUT_RET3,
            )?;

            let eeprom_state = ethercat_to_host(u16::from_ne_bytes(word));
            context.slavelist[slave_usize].eep_read_size =
                if eeprom_state & EEPROM_STATE_MACHINE_READ64 != 0 {
                    EepReadSize::Bytes8
                } else {
                    EepReadSize::Bytes4
                };
        }

        // Read the manufacturer of the slave from it's EEPROM
        read_eeprom1(context, slave, SiiGeneralItem::Manufacturer);
    }

    // Store manufacturer and request id
    async_eeprom_read(context, SiiGeneralItem::Id, |slave, manufacturer| {
        slave.eep_manufacturer = ethercat_to_host(manufacturer);
    });

    // Store ID and request revision
    async_eeprom_read(context, SiiGeneralItem::Revision, |slave, id| {
        slave.eep_id = ethercat_to_host(id);
    });

    // Store revision and request mailbox address + mailbox size
    async_eeprom_read(
        context,
        SiiGeneralItem::RxMailboxAddress,
        |slave, revision| {
            slave.eep_revision = ethercat_to_host(revision);
        },
    );

    for slave in 1..context.slave_count {
        let slave_usize = usize::from(slave);
        // Mailbox address and mailbox size
        let mailbox_rx = read_eeprom2(context, slave, TIMEOUT_EEP);
        context.slavelist[slave_usize].mailbox_read_offset = low_word(ethercat_to_host(mailbox_rx));
        context.slavelist[slave_usize].mailbox_length = high_word(ethercat_to_host(mailbox_rx));

        if context.slavelist[slave_usize].mailbox_length > 0 {
            // Read mailbox offset
            read_eeprom1(context, slave, SiiGeneralItem::TxMailboxAddress);
        }
    }

    for slave in 1..context.slave_count {
        let slave_usize = usize::from(slave);
        if context.slavelist[slave_usize].mailbox_length > 0 {
            // Read mailbox information
            let mailbox_info = read_eeprom2(context, slave, TIMEOUT_EEP);
            context.slavelist[slave_usize].mailbox_read_offset =
                low_word(ethercat_to_host(mailbox_info));
            context.slavelist[slave_usize].mailbox_read_length =
                high_word(ethercat_to_host(mailbox_info));
            if context.slavelist[slave_usize].mailbox_read_length == 0 {
                context.slavelist[slave_usize].mailbox_read_length =
                    context.slavelist[slave_usize].mailbox_length;
            }
            read_eeprom1(context, slave, SiiGeneralItem::MailboxProtocol);
        }
        let config_address = context.slavelist[slave_usize].config_address;
        let escsup = fprdw(
            &mut context.port.lock().unwrap(),
            config_address,
            EthercatRegister::EscSup,
            TIMEOUT_RET3,
        )?;

        // Check whether the slave supports distributed clock
        context.slavelist[slave_usize].has_dc = ethercat_to_host(escsup) & 4 > 0;

        // Extract topology from DL status
        let topology = ethercat_to_host(fprdw(
            &mut context.port.lock().unwrap(),
            config_address,
            EthercatRegister::DeviceLayerStatus,
            TIMEOUT_RET3,
        )?);
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

        // Physical type
        context.slavelist[slave_usize].physical_type = low_byte(ethercat_to_host(fprdw(
            &mut context.port.lock().unwrap(),
            config_address,
            EthercatRegister::PortDescriptor,
            TIMEOUT_RET3,
        )?));
        context.slavelist[slave_usize].topology = topology_count;
        context.slavelist[slave_usize].active_ports = active_ports;

        // 0 = no links, not possible
        // 1 = 1 link, end of line
        // 2 = 2 links, one before and one after
        // 3 = 3 links, split point
        // 4 = 4 links, cross point
        // search for parent
        context.slavelist[slave_usize].parent_port = 0;
        if slave > 1 {
            let mut topology_links = 0;
            let slave_count = slave - 1;
            loop {
                let topology = context.slavelist[usize::from(slave_count)].topology;
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
                    context.slavelist[slave_usize].parent = slave_count as u8;
                    break;
                }
                topology_links -= 1;

                if slave_count == 0 {
                    break;
                }
            }
        }

        // Check state change init
        statecheck(context, slave, EthercatState::Init, TIMEOUT_STATE);

        // Set default mailbox configuration if slave has mailbox
        if context.slavelist[slave_usize].mailbox_length > 0 {
            let sync_manager_type = &mut context.slavelist[slave_usize].sync_manager_type;
            sync_manager_type[0] = SyncManagerType::MailboxWrite;
            sync_manager_type[1] = SyncManagerType::MailboxRead;
            sync_manager_type[2] = SyncManagerType::Outputs;
            sync_manager_type[3] = SyncManagerType::Inputs;
            let slave_object = &mut context.slavelist[slave_usize];
            slave_object.sync_manager[0].start_address =
                host_to_ethercat(slave_object.mailbox_write_offset);
            slave_object.sync_manager[0].sm_length = host_to_ethercat(slave_object.mailbox_length);
            slave_object.sync_manager[0].sm_flags = host_to_ethercat(DEFAULT_MAILBOX_SM0);
            slave_object.sync_manager[1].start_address =
                host_to_ethercat(slave_object.mailbox_read_offset);
            slave_object.sync_manager[1].sm_length =
                host_to_ethercat(slave_object.mailbox_read_offset);
            slave_object.sync_manager[1].sm_flags = host_to_ethercat(DEFAULT_MAILBOX_SM1);
            context.slavelist[slave_usize].mailbox_protocols =
                ethercat_to_host(read_eeprom2(context, slave, TIMEOUT_EEP)) as u16;
        }

        let config_index = if use_table {
            config_from_table(context, slave)
        } else {
            0
        };

        // Slave not in configuration table, find out via SII
        if config_index == 0 && !lookup_previous_sii(context, slave) {
            let sii_general = sii_find(context, slave, SiiCategory::General);
            if sii_general != 0 {
                context.slavelist[slave_usize].canopen_over_ethercat_details =
                    sii_get_byte(context, slave, sii_general + 7);
                context.slavelist[slave_usize].file_over_ethercat_details =
                    sii_get_byte(context, slave, sii_general + 8);
                context.slavelist[slave_usize].ethernet_over_ethercat_details =
                    sii_get_byte(context, slave, sii_general + 9);
                context.slavelist[slave_usize].servo_over_ethercat_details =
                    sii_get_byte(context, slave, sii_general + 0xA);
                if sii_get_byte(context, slave, sii_general + 0xD) & 0x2 > 0 {
                    context.slavelist[slave_usize].block_logical_read_write = 1;
                    context.slavelist[0].block_logical_read_write += 1;
                }
                context.slavelist[slave_usize].ebus_current =
                    u16::from(sii_get_byte(context, slave, sii_general + 0xE))
                        + (u16::from(sii_get_byte(context, slave, sii_general + 0xF)) << 8);
                context.slavelist[0].ebus_current += context.slavelist[slave_usize].ebus_current;
            }

            // SII strings section
            if sii_find(context, slave, SiiCategory::String) > 0 {
                let mut name = heapless::String::new();
                sii_string(context, &mut name, slave, 1);
                context.slavelist[slave_usize].name = name;
            } else {
                // No name for slave found, use constructed name
                let manufacturer = context.slavelist[slave_usize].eep_manufacturer;
                let id = context.slavelist[slave_usize].eep_id;
                context.slavelist[slave_usize].name.clear();
                write!(
                    context.slavelist[slave_usize].name,
                    "? M:{manufacturer:8.8x} I:{id:8.8x}",
                )
                .unwrap();
            }

            // SII SM section
            let mut eep_sync_manager = context.eep_sync_manager.clone();
            let number_sm = sii_sm(context, slave, &mut eep_sync_manager);
            context.eep_sync_manager = eep_sync_manager;
            if number_sm > 0 {
                context.slavelist[slave_usize].sync_manager[0].start_address =
                    host_to_ethercat(context.eep_sync_manager.phase_start);
                context.slavelist[slave_usize].sync_manager[0].sm_length =
                    host_to_ethercat(context.eep_sync_manager.phase_length);
                context.slavelist[slave_usize].sync_manager[0].sm_flags = host_to_ethercat(
                    u32::from(context.eep_sync_manager.control_register)
                        + (u32::from(context.eep_sync_manager.activate) << 16),
                );
                let mut sync_manager_count = 1;
                let mut eep_sync_manager = context.eep_sync_manager.clone();
                while sync_manager_count < MAX_SM
                    && sii_sm_next(
                        context,
                        slave,
                        &mut eep_sync_manager,
                        sync_manager_count.into(),
                    ) != 0
                {
                    let sync_manager_count_usize = usize::from(sync_manager_count);
                    context.slavelist[slave_usize].sync_manager[sync_manager_count_usize]
                        .start_address = host_to_ethercat(eep_sync_manager.phase_start);
                    context.slavelist[slave_usize].sync_manager[sync_manager_count_usize]
                        .sm_length = host_to_ethercat(eep_sync_manager.phase_length);
                    context.slavelist[slave_usize].sync_manager[sync_manager_count_usize]
                        .sm_flags = host_to_ethercat(
                        u32::from(eep_sync_manager.control_register)
                            + (u32::from(eep_sync_manager.activate) << 16),
                    );
                    sync_manager_count += 1;
                }
                context.eep_sync_manager = eep_sync_manager;
            }

            let mut eep_fmmu = context.eep_fmmu;
            if sii_fmmu(context, slave, &mut eep_fmmu) != 0 {
                if context.eep_fmmu.fmmu[0] != 0xFF {
                    context.slavelist[slave_usize].fmmu0_function = context.eep_fmmu.fmmu[0];
                }
                if context.eep_fmmu.fmmu[1] != 0xFF {
                    context.slavelist[slave_usize].fmmu1_function = context.eep_fmmu.fmmu[1];
                }
                if context.eep_fmmu.fmmu[2] != 0xFF {
                    context.slavelist[slave_usize].fmmu2_function = context.eep_fmmu.fmmu[2];
                }
                if context.eep_fmmu.fmmu[3] != 0xFF {
                    context.slavelist[slave_usize].fmmu3_function = context.eep_fmmu.fmmu[3];
                }
            }
            context.eep_fmmu = eep_fmmu;
        }
        if context.slavelist[slave_usize].mailbox_length > 0 {
            let slave_object = &mut context.slavelist[slave_usize];

            // Should never happen
            if slave_object.sync_manager[0].start_address == 0 {
                ec_println!("Slave {slave} has no proper mailbox in configuration, try default.");
                slave_object.sync_manager[0].start_address = host_to_ethercat(0x1000);
                slave_object.sync_manager[0].sm_length = host_to_ethercat(0x80);
                slave_object.sync_manager[0].sm_flags = host_to_ethercat(DEFAULT_MAILBOX_SM0);
                slave_object.sync_manager_type[0] = SyncManagerType::MailboxWrite;
            }

            // Should never happen
            if slave_object.sync_manager[1].start_address == 0 {
                ec_println!("Slave {slave} has no proper mailbox out configuration, try default.");
                slave_object.sync_manager[1].start_address = host_to_ethercat(0x1080);
                slave_object.sync_manager[1].sm_length = host_to_ethercat(0x80);
                slave_object.sync_manager[1].sm_flags = host_to_ethercat(DEFAULT_MAILBOX_SM1);
                slave_object.sync_manager_type[1] = SyncManagerType::MailboxRead;
            }

            // Program SM0 mailbox in and SM1 mailbox out for slave.
            // Writing both sm in one datagram will solve timing issue in old NETX
            fpwr(
                &mut context.port.lock().unwrap(),
                config_address,
                EthercatRegister::SyncManager0 as u16,
                <&mut [u8]>::from(&mut context.slavelist[slave_usize].sync_manager[0]),
                TIMEOUT_RET3,
            )?;
        }

        // Some slaves need eeprom available to PDI in init -> preop transition
        eeprom_to_pdi(context, slave);

        // User may override automatic state change
        if !context.manual_state_change {
            // Request pre-op for slave
            fpwrw(
                &mut context.port.lock().unwrap(),
                config_address,
                EthercatRegister::ApplicationLayerControl,
                host_to_ethercat(
                    EthercatState::PreOperational as u16 | EthercatState::Error as u16,
                ),
                TIMEOUT_RET3,
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
    if slave > 1 && context.slave_count > 0 {
        let slave_usize = usize::from(slave);
        let requested_slave = &context.slavelist[slave_usize];
        let found_slave = context
            .slavelist
            .iter()
            .enumerate()
            .take(slave_usize)
            .skip(1)
            .find(|(_, current_slave)| {
                current_slave.eep_manufacturer == requested_slave.eep_manufacturer
                    && current_slave.eep_id == requested_slave.eep_id
                    && current_slave.eep_revision == requested_slave.eep_revision
            })
            .map(|(index, _)| index);
        if let Some(found_slave) = found_slave {
            for nsm in 0..usize::from(MAX_SM) {
                context.slavelist[slave_usize].sync_manager[nsm].sm_length =
                    context.slavelist[found_slave].sync_manager[nsm].sm_length;
                context.slavelist[slave_usize].sync_manager_type[nsm] =
                    context.slavelist[found_slave].sync_manager_type[nsm];
            }
            *output_size = context.slavelist[found_slave].output_bits.into();
            *input_size = context.slavelist[found_slave].input_bits.into();
            context.slavelist[slave_usize].output_bits = context.slavelist[found_slave].output_bits;
            context.slavelist[slave_usize].input_bits = context.slavelist[found_slave].input_bits;
            ec_println!("Copy mapping slave {slave} from {found_slave}.");
            return true;
        }
    }
    false
}

pub fn map_coe_soe(
    context: &mut Context,
    slave: u16,
    thread_number: usize,
) -> Result<(), CoEError> {
    // Check state change pre-op
    statecheck(context, slave, EthercatState::PreOperational, TIMEOUT_STATE);

    let slave_usize = usize::from(slave);
    ec_println!(
        " >Slave {slave}, configaddr {:x}, state {:2.2x}",
        context.slavelist[slave_usize].config_address,
        context.slavelist[slave_usize].state
    );

    // Execute special slave configuration hook pre-op to safe-op.

    // Only if registered
    if let Some(po2_so_config) = context.slavelist[slave_usize].po2_so_config {
        po2_so_config(slave);
    }

    // Only if registered
    if let Some(po2_so_configx) = context.slavelist[slave_usize].po2_so_configx {
        po2_so_configx(context, slave);
    }

    // If slave not found in configlist, find IO mapping in slave self
    if context.slavelist[slave_usize].config_index == 0 {
        return Ok(());
    }
    let mut input_size = 0;
    let mut output_size = 0;

    // If slave supports CANopen over EtherCAT
    if context.slavelist[slave_usize].mailbox_protocols
        & u16::from(MailboxProtocol::CanopenOverEthercat)
        == 0
    {
        let mut initialized = false;

        // If there is complete access
        if context.slavelist[slave_usize].canopen_over_ethercat_details & u8::from(Coedet::Sdoca)
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
                Ok(()) => initialized = true,
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
        && context.slavelist[slave_usize].mailbox_protocols
            & u16::from(MailboxProtocol::ServoOverEthercat)
            != 0
    {
        // Read AT/MDT mapping via Servo over EtherCAT
        read_id_nmap(context, slave, &mut output_size, &mut input_size);
        context.slavelist[slave_usize].sync_manager[2].sm_length =
            host_to_ethercat(output_size.div_ceil(8)) as u16;
        context.slavelist[slave_usize].sync_manager[3].sm_length =
            host_to_ethercat(input_size.div_ceil(8)) as u16;
        ec_println!("  SOE output_size:{output_size} input_size:{input_size}");
    }
    context.slavelist[slave_usize].output_bits = output_size as u16;
    context.slavelist[slave_usize].input_bits = input_size as u16;

    Ok(())
}

fn map_sii(context: &mut Context, slave: u16) {
    let slave_usize = slave as usize;

    let mut output_size = u32::from(context.slavelist[slave_usize].output_bits);
    let mut input_size = u32::from(context.slavelist[slave_usize].input_bits);

    // Find PDO in previous slave with same ID
    if input_size == 0 && output_size == 0 {
        lookup_mapping(context, slave, &mut output_size, &mut input_size);
    }

    // Find PDO mapping by SII
    if input_size == 0 && output_size == 0 {
        let mut eepdo = EepromPdo::default();
        input_size = sii_pdo(context, slave, &mut eepdo, 0);

        ec_println!("  SII input_size:{input_size}");
        for sm_index in 0..usize::from(MAX_SM) {
            if eepdo.sync_manager_bit_size[sm_index] > 0 {
                context.slavelist[slave_usize].sync_manager[sm_index].sm_length =
                    host_to_ethercat(eepdo.sync_manager_bit_size[sm_index].div_ceil(8));
                context.slavelist[slave_usize].sync_manager_type[sm_index] =
                    SyncManagerType::Inputs;
                ec_println!(
                    "    SM{sm_index} length {}",
                    eepdo.sync_manager_bit_size[sm_index]
                );
            }
        }
    }
    context.slavelist[slave_usize].output_bits = output_size as u16;
    context.slavelist[slave_usize].input_bits = input_size as u16;
    ec_println!("     input_size:{input_size} output_size:{output_size}");
}

fn map_sm(context: &mut Context, slave: u16) -> Result<(), NicdrvError> {
    let slave_usize = usize::from(slave);
    let config_address = context.slavelist[slave_usize].config_address;

    ec_println!("  SM programming");
    if context.slavelist[slave_usize].mailbox_length == 0
        && context.slavelist[slave_usize].sync_manager[0].start_address != 0
    {
        fpwr(
            &mut context.port.lock().unwrap(),
            config_address,
            EthercatRegister::SyncManager0 as u16,
            <&mut [u8]>::from(&mut context.slavelist[slave_usize].sync_manager[0]),
            TIMEOUT_RET3,
        )?;
        ec_println!(
            "    SM0 type:{:?} StartAddress:{:4.4x} Flags:{:8.8x}",
            context.slavelist[slave_usize].sync_manager_type[0],
            ethercat_to_host(context.slavelist[slave_usize].sync_manager[0].start_address),
            ethercat_to_host(context.slavelist[slave_usize].sync_manager[0].sm_flags)
        );
    }
    if !context.slavelist[slave_usize].mailbox_length == 0
        && context.slavelist[slave_usize].sync_manager[1].start_address != 0
    {
        fpwr(
            &mut context.port.lock().unwrap(),
            config_address,
            EthercatRegister::SyncManager1 as u16,
            <&mut [u8]>::from(&mut context.slavelist[slave_usize].sync_manager[1]),
            TIMEOUT_RET3,
        )?;
        ec_println!(
            "    SM1 type:{:?} StartAddress:{:4.4x} Flags:{:8.8x}",
            context.slavelist[slave_usize].sync_manager_type[1],
            ethercat_to_host(context.slavelist[slave_usize].sync_manager[1].start_address),
            ethercat_to_host(context.slavelist[slave_usize].sync_manager[1].sm_flags)
        );
    }

    // Program SM2 to SMx
    for sm_index in 2..usize::from(MAX_SM) {
        if context.slavelist[slave_usize].sync_manager[sm_index].start_address != 0 {
            // Check if SM length is zero -> clear enable flag
            context.slavelist[slave_usize].sync_manager[sm_index].sm_flags =
                if context.slavelist[slave_usize].sync_manager[sm_index].sm_length == 0 {
                    host_to_ethercat(
                        ethercat_to_host(
                            context.slavelist[slave_usize].sync_manager[sm_index].sm_flags,
                        ) & SYNC_MANAGER_ENABLE_MASK,
                    )
                } else {
                    // If SM length is non zero, always set the enable flag
                    host_to_ethercat(
                        ethercat_to_host(
                            context.slavelist[slave_usize].sync_manager[sm_index].sm_flags,
                        ) | !SYNC_MANAGER_ENABLE_MASK,
                    )
                };
            fpwr(
                &mut context.port.lock().unwrap(),
                config_address,
                u16::from(EthercatRegister::SyncManager0)
                    + (sm_index * size_of::<SyncManager>()) as u16,
                <&mut [u8]>::from(&mut context.slavelist[slave_usize].sync_manager[sm_index]),
                TIMEOUT_RET3,
            )?;
            ec_println!(
                "    SM{sm_index} Type:{:?} StartAddress:{:4.4x} Flags:{:8.8x}",
                context.slavelist[slave_usize].sync_manager_type[sm_index],
                ethercat_to_host(
                    context.slavelist[slave_usize].sync_manager[sm_index].start_address
                ),
                ethercat_to_host(context.slavelist[slave_usize].sync_manager[sm_index].sm_flags)
            );
        }
    }
    if context.slavelist[slave_usize].input_bits > 7 {
        context.slavelist[slave_usize].input_bytes =
            context.slavelist[slave_usize].input_bits.div_ceil(8);
    }
    if context.slavelist[slave_usize].output_bits > 7 {
        context.slavelist[slave_usize].output_bytes =
            context.slavelist[slave_usize].output_bits.div_ceil(8);
    }
    Ok(())
}

pub fn config_find_mappings(context: &mut Context, group: u8) -> Result<(), ConfigError> {
    // Find CANopen over EtherCAT and Servo over EtherCAT mapping of slaves in multiple threads
    for slave in 1..context.slave_count {
        let slave_usize = usize::from(slave);
        if group == 0 || group == context.slavelist[slave_usize].group {
            // Serialized version
            map_coe_soe(context, slave, 0)?;
        }
    }

    // Find SII mapping of slave and program
    for slave in 1..context.slave_count {
        if group == 0 || group == context.slavelist[usize::from(slave)].group {
            map_sii(context, slave);
            map_sm(context, slave)?;
        }
    }
    Ok(())
}

pub fn config_create_input_mappings<'a, 'b: 'a>(
    context: &mut Context<'a>,
    io_map: &'b [u8],
    group: u8,
    slave: u16,
    log_address: &mut u32,
    bit_position: &mut u8,
) -> Result<(), NicdrvError> {
    ec_println!(" =Slave {slave}, INPUT MAPPING");

    let slave_usize = usize::from(slave);
    let config_address = context.slavelist[slave_usize].config_address;

    // Find free FMMU
    let mut fmmu_count = if context.slavelist[slave_usize].output_bits != 0 {
        context.slavelist[slave_usize]
            .fmmu
            .iter()
            .enumerate()
            .skip(context.slavelist[slave_usize].fmmu_unused.into())
            .find(|(_, fmmu)| fmmu.log_start == 0)
            .map_or(context.slavelist[slave_usize].fmmu.len(), |(index, _)| {
                index
            }) as u8
    } else {
        context.slavelist[slave_usize].fmmu_unused
    };

    // Search for SM's that contribute to the input mapping
    let mut sm_count = 0;
    let mut fmmu_done = 0;
    let mut byte_count = 0;
    let mut bit_count = 0;
    let mut add_to_inputs_wkc = false;
    while sm_count < usize::from(MAX_SM)
        && fmmu_done < context.slavelist[slave_usize].input_bits.div_ceil(8)
    {
        let fmmu_size;
        ec_println!("    FMMU {fmmu_count}");
        while sm_count < usize::from(MAX_SM - 1)
            && context.slavelist[slave_usize].sync_manager_type[sm_count] != SyncManagerType::Inputs
        {
            sm_count += 1;
        }

        ec_println!("      SM{sm_count}");
        context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].physical_start =
            context.slavelist[slave_usize].sync_manager[sm_count].start_address;
        let mut sm_length =
            ethercat_to_host(context.slavelist[slave_usize].sync_manager[sm_count].sm_length);
        byte_count += sm_length;
        bit_count += sm_length * 8;
        let mut end_address =
            ethercat_to_host(context.slavelist[slave_usize].sync_manager[sm_count].start_address)
                + sm_length;

        // More SM for input
        while bit_count < context.slavelist[slave_usize].input_bits
            && sm_count < usize::from(MAX_SM) - 1
        {
            sm_count += 1;
            while sm_count < usize::from(MAX_SM) - 1
                && context.slavelist[slave_usize].sync_manager_type[sm_count]
                    != SyncManagerType::Inputs
            {
                sm_count += 1;
            }

            // If addresses from more SM connect use one FMMU
            if ethercat_to_host(context.slavelist[slave_usize].sync_manager[sm_count].start_address)
                > end_address
            {
                break;
            }
            ec_println!("      SM{sm_count}");
            sm_length =
                ethercat_to_host(context.slavelist[slave_usize].sync_manager[sm_count].sm_length);
            byte_count += sm_length;
            bit_count += sm_length * 8;
            end_address = ethercat_to_host(
                context.slavelist[slave_usize].sync_manager[sm_count].start_address,
            ) + sm_length;
        }

        // Bit oriented slave
        if context.slavelist[slave_usize].input_bytes == 0 {
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start =
                host_to_ethercat(*log_address);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start_bit =
                *bit_position;
            *bit_position += (context.slavelist[slave_usize].input_bits - 1) as u8;
            if *bit_position > 7 {
                *log_address += 1;
                *bit_position -= 8;
            }
            fmmu_size = (*log_address
                - ethercat_to_host(
                    context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start,
                )
                + 1) as u16;
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_length =
                host_to_ethercat(fmmu_size);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_end_bit =
                *bit_position;
            *bit_position += 1;
            if *bit_position > 7 {
                *log_address += 1;
                *bit_position -= 8;
            }
        } else {
            // Byte oriented slave
            if *bit_position != 0 {
                *log_address += 1;
                *bit_position = 0;
            }
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start =
                host_to_ethercat(*log_address);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start_bit =
                *bit_position;
            *bit_position = 7;
            fmmu_size = if byte_count + fmmu_done > context.slavelist[slave_usize].input_bytes {
                context.slavelist[slave_usize].input_bytes - fmmu_done
            } else {
                byte_count
            };
            *log_address += u32::from(fmmu_size);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_length =
                host_to_ethercat(fmmu_size);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_end_bit =
                *bit_position;
            *bit_position = 0;
        }

        fmmu_done += fmmu_size;
        if context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_length != 0 {
            let fmmu_count_usize = usize::from(fmmu_count);
            context.slavelist[slave_usize].fmmu[fmmu_count_usize].physical_start_bit = 0;
            context.slavelist[slave_usize].fmmu[fmmu_count_usize].fmmu_type = 1;
            context.slavelist[slave_usize].fmmu[fmmu_count_usize].fmmu_active = 1;

            // Program FMMU for input
            fpwr(
                &mut context.port.lock().unwrap(),
                config_address,
                EthercatRegister::FieldbusMemoryManagementUnit0 as u16
                    + u16::from(fmmu_count) * size_of::<Fmmu>() as u16,
                <&mut [u8]>::from(
                    &mut context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)],
                ),
                TIMEOUT_RET3,
            )?;

            // Set flag to add one for an input FMMU,
            // a single ESC can only contribute once.
            add_to_inputs_wkc = true;
        }
        if context.slavelist[slave_usize].input.is_none() {
            if group != 0 {
                context.slavelist[slave_usize].input = Some(
                    &io_map[(ethercat_to_host(
                        context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start,
                    ) - context.grouplist[usize::from(group)].logical_start_address)
                        as usize..],
                )
            } else {
                context.slavelist[slave_usize].input = Some(
                    &io_map[ethercat_to_host(
                        context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start,
                    ) as usize..],
                );
            }
            context.slavelist[slave_usize].input_startbit =
                context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start_bit;
            ec_println!(
                "    Inputs {:p} startbit {}",
                context.slavelist[slave_usize].input.unwrap(),
                context.slavelist[slave_usize].input_startbit
            );
        }
        fmmu_count += 1;
    }

    context.slavelist[slave_usize].fmmu_unused = fmmu_count;

    // Add one WKC for an input if flag is true
    if add_to_inputs_wkc {
        context.grouplist[usize::from(group)].work_counter_inputs += 1;
    }
    Ok(())
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
    let mut add_to_inputs_wkc = false;
    let mut byte_count = 0;
    let mut fmmu_size;
    let mut sm_count = 0;

    ec_println!("  OUTPUT MAPPING");

    let slave_usize = usize::from(slave);
    let mut fmmu_count = context.slavelist[slave_usize].fmmu_unused;
    let config_address = context.slavelist[slave_usize].config_address;

    // Search for SM's that contribute to the output mapping
    while sm_count < MAX_SM && fmmu_done < context.slavelist[slave_usize].output_bits.div_ceil(8) {
        ec_println!("    FMMU {fmmu_count}");
        while sm_count < MAX_SM - 1
            && context.slavelist[slave_usize].sync_manager_type[usize::from(sm_count)]
                != SyncManagerType::Outputs
        {
            sm_count += 1;
        }
        ec_println!("      SM{sm_count}");
        context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].physical_start =
            context.slavelist[slave_usize].sync_manager[usize::from(sm_count)].start_address;
        let mut sm_length = ethercat_to_host(
            context.slavelist[slave_usize].sync_manager[usize::from(sm_count)].sm_length,
        );
        byte_count += sm_length;
        bit_count += sm_length * 8;
        let mut end_address = ethercat_to_host(
            context.slavelist[slave_usize].sync_manager[usize::from(sm_count)].start_address
                + sm_length,
        );
        while bit_count < context.slavelist[slave_usize].output_bits && sm_count < MAX_SM - 1 {
            sm_count += 1;
            while sm_count < MAX_SM - 1
                && context.slavelist[slave_usize].sync_manager_type[usize::from(sm_count)]
                    != SyncManagerType::Outputs
            {
                sm_count += 1;
            }

            // If addresses from more SM connect use one FMMU, otherwise break up in multiple FMMU
            if ethercat_to_host(
                context.slavelist[slave_usize].sync_manager[usize::from(sm_count)].sm_length,
            ) > end_address
            {
                break;
            }

            ec_println!("      SM{sm_count}");
            sm_length = ethercat_to_host(
                context.slavelist[slave_usize].sync_manager[usize::from(sm_count)].sm_length,
            );
            byte_count += sm_length;
            bit_count += sm_length;
            end_address = ethercat_to_host(
                context.slavelist[slave_usize].sync_manager[usize::from(sm_count)].start_address,
            );
        }

        // Bit oriented slave
        if context.slavelist[slave_usize].output_bytes == 0 {
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start =
                host_to_ethercat(*log_address);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start_bit =
                *bit_position;
            *bit_position += (context.slavelist[slave_usize].output_bits - 1) as u8;
            if *bit_position > 7 {
                *log_address += 1;
                *bit_position -= 8;
            }
            fmmu_size = (*log_address
                - ethercat_to_host(
                    context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start,
                )
                + 1) as u16;
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_length =
                host_to_ethercat(fmmu_size);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_end_bit =
                *bit_position;
            *bit_position += 1;
            if *bit_position > 7 {
                *log_address += 1;
                *bit_position -= 8;
            }
        } else {
            // Byte oriented slave
            if *bit_position != 0 {
                *log_address += 1;
                *bit_position = 0;
            }
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start =
                host_to_ethercat(*log_address);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start_bit =
                *bit_position;
            *bit_position = 7;
            fmmu_size = byte_count;
            if fmmu_size + fmmu_done > context.slavelist[slave_usize].output_bytes {
                fmmu_size = context.slavelist[slave_usize].output_bytes - fmmu_done;
            }
            *log_address += u32::from(fmmu_size);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_length =
                host_to_ethercat(fmmu_size);
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_end_bit =
                *bit_position;
            *bit_position = 0;
        }
        fmmu_done += fmmu_size;

        if context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_length != 0 {
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].physical_start_bit = 0;
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].fmmu_type = 2;
            context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].fmmu_active = 1;

            // Program FMMU for output
            fpwr(
                &mut context.port.lock().unwrap(),
                config_address,
                EthercatRegister::FieldbusMemoryManagementUnit0 as u16
                    + u16::from(fmmu_count) * size_of::<Fmmu>() as u16,
                <&mut [u8]>::from(
                    &mut context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)],
                ),
                TIMEOUT_RET3,
            )?;

            // Set flag to add one for an input FMMU.
            // A single ESC can only contribute once.
            add_to_inputs_wkc = true;
        }

        if context.slavelist[slave_usize].input.is_none() {
            if group != 0 {
                context.slavelist[slave_usize].input = Some(
                    &io_map[(ethercat_to_host(
                        context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start,
                    ) - context.grouplist[usize::from(group)].logical_start_address)
                        as usize..],
                );
            } else {
                context.slavelist[slave_usize].input = Some(
                    &io_map[ethercat_to_host(
                        context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start,
                    ) as usize..],
                )
            }
            context.slavelist[slave_usize].input_startbit =
                context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)].log_start_bit;
            ec_println!(
                "    Inputs {:p} startbit {}",
                context.slavelist[slave_usize].input.unwrap(),
                context.slavelist[slave_usize].input_startbit
            );
        }
        fmmu_count += 1;
    }
    context.slavelist[slave_usize].fmmu_unused = fmmu_count;

    // Add one WKC for an input if needed
    if add_to_inputs_wkc {
        context.grouplist[usize::from(group)].work_counter_inputs += 1;
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
    if context.slave_count > 0 && u32::from(group) < context.max_group {
        let group_usize = usize::from(group);
        ec_println!("ec_config_map_group IOmap:{io_map:p} group:{group}");
        let mut logical_address = context.grouplist[group_usize].logical_start_address;
        let mut logical_address2 = logical_address;
        let mut bit_position = 0;
        context.grouplist[group_usize].used_segment_count = 0;
        context.grouplist[group_usize].work_counter_outputs = 0;
        context.grouplist[group_usize].work_counter_inputs = 0;

        // Find mappings and program synchronization managers
        config_find_mappings(context, group)?;

        // Do output mapping of slave and program FMMUs
        for slave in 1..context.slave_count {
            let slave_usize = usize::from(slave);

            if group == 0 || group == context.slavelist[slave_usize].group {
                // Create output mapping
                if context.slavelist[slave_usize].output_bits != 0 {
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
                        context.grouplist[group_usize].io_segments[usize::from(current_segment)] =
                            u32::from(segment_size);
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
                context.grouplist[group_usize].io_segments[usize::from(current_segment)] =
                    u32::from(segment_size);
                if usize::from(current_segment) < MAX_IO_SEGMENTS - 1 {
                    current_segment += 1;
                    segment_size = 1;
                }
            } else {
                segment_size += 1;
            }
        }
        context.grouplist[group_usize].outputs = io_map;
        context.grouplist[group_usize].output_bytes =
            logical_address - context.grouplist[group_usize].logical_start_address;
        context.grouplist[group_usize].used_segment_count = current_segment + 1;
        context.grouplist[group_usize].first_input_segment = current_segment;
        context.grouplist[group_usize].input_offset = segment_size;
        if group == 0 {
            context.slavelist[0].outputs = io_map;

            // Store output bytes in master record
            context.slavelist[0].output_bytes =
                (logical_address - context.grouplist[group_usize].logical_start_address) as u16;
        }

        // Input mapping of slave and program FMMUs
        for slave in 1..context.slave_count {
            let slave_usize = usize::from(slave);
            let config_address = context.slavelist[slave_usize].config_address;
            if group == 0 || group == context.slavelist[slave_usize].group {
                // Create input mapping
                if context.slavelist[slave_usize].input_bits != 0 {
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
                        context.grouplist[group_usize].io_segments[usize::from(current_segment)] =
                            u32::from(segment_size);
                        if usize::from(current_segment) < MAX_IO_SEGMENTS - 1 {
                            current_segment += 1;
                            segment_size = difference as u16;
                        }
                    } else {
                        segment_size += difference as u16;
                    }
                }

                // Set eeprom control to PDI
                eeprom_to_pdi(context, slave);

                // User may override automatic state change
                if !context.manual_state_change {
                    // Request safe operation for slave
                    fpwrw(
                        &mut context.port.lock().unwrap(),
                        config_address,
                        EthercatRegister::ApplicationLayerControl,
                        host_to_ethercat(u16::from(EthercatState::Operational)),
                        TIMEOUT_RET3,
                    )?;
                }

                if context.slavelist[slave_usize].block_logical_read_write != 0 {
                    context.grouplist[group_usize].block_logical_read_write += 1;
                }
                context.grouplist[group_usize].ebus_current +=
                    context.slavelist[slave_usize].ebus_current;
            }
        }

        if bit_position != 0 {
            logical_address += 1;
            if usize::from(segment_size) + 1 > MAX_LRW_DATA_LENGTH - FIRST_DC_DATAGRAM_SIZE {
                context.grouplist[group_usize].io_segments[usize::from(current_segment)] =
                    u32::from(segment_size);
                if usize::from(current_segment) < MAX_IO_SEGMENTS - 1 {
                    current_segment += 1;
                    segment_size = 1;
                }
            } else {
                segment_size += 1;
            }
        }

        context.grouplist[group_usize].io_segments[usize::from(current_segment)] =
            u32::from(segment_size);
        context.grouplist[group_usize].used_segment_count = current_segment + 1;
        context.grouplist[group_usize].inputs =
            &io_map[context.grouplist[group_usize].output_bytes as usize..];
        context.grouplist[group_usize].input_bytes = logical_address
            - context.grouplist[group_usize].logical_start_address
            - context.grouplist[group_usize].output_bytes;

        if group == 0 {
            // Store input bytes in master record
            context.slavelist[0].input =
                Some(&io_map[usize::from(context.slavelist[0].output_bytes)..]);
            context.slavelist[0].input_bytes = (logical_address
                - context.grouplist[group_usize].logical_start_address
                - u32::from(context.slavelist[0].output_bytes))
                as u16;
        }

        ec_println!(
            "IOmapSize {}",
            logical_address - context.grouplist[group_usize].logical_start_address
        );

        return Ok(logical_address - context.grouplist[group_usize].logical_start_address);
    }
    Ok(0)
}

/// Map all PDOs in one group of slaves to IO map with outputs/inputs
/// in sequential order (legacy soem way).
///
/// # Parameters
/// `context`: Context struct
/// `io_map`: IOmap
/// `group`: Group to map, 0 is all groups
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
/// `context`: Context struct
/// `io_map`: IOmap
/// `group`: Group to map, 0 = all groups
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
/// `context`: Context struct
/// `io_map`: Pointer to IOmap
/// `group`: Group to map, 0 = all groups
///
/// # Returns
/// IOmap size or error
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

    if context.slave_count > 0 && u32::from(group) < context.max_group {
        ec_println!("ec_config_map_group IOmap:{io_map:p} group:{group}");
        let group_usize = usize::from(group);
        m_logical_address = u32::from(context.grouplist[group_usize].used_segment_count);
        si_logical_address = m_logical_address;
        so_logical_address = m_logical_address;
        let mut bit_position = 0;
        context.grouplist[group_usize].used_segment_count = 0;
        context.grouplist[group_usize].work_counter_outputs = 0;
        context.grouplist[group_usize].work_counter_inputs = 0;

        // Find mappings and program syncmanagers
        config_find_mappings(context, group)?;

        // Do IO mapping of slave and program FMMUs
        for slave in 1..context.slave_count {
            let slave_usize = usize::from(slave);
            let config_address = context.slavelist[slave_usize].config_address;
            (si_logical_address, so_logical_address) = (m_logical_address, m_logical_address);

            if group == 0 || group == context.slavelist[slave_usize].group {
                // Create output mapping
                if context.slavelist[slave_usize].output_bits != 0 {
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
                if context.slavelist[slave_usize].input_bits != 0 {
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
                    context.grouplist[group_usize].io_segments[current_segment] = segment_size;
                    if current_segment <= MAX_IO_SEGMENTS {
                        current_segment += 1;
                        segment_size = difference;
                    }
                } else {
                    segment_size += difference;
                }

                // Set EEPROM control to PDI
                eeprom_to_pdi(context, slave);

                // User may override automatic state change
                if !context.manual_state_change {
                    // Request safe operation for slave
                    fpwrw(
                        &mut context.port.lock().unwrap(),
                        config_address,
                        EthercatRegister::ApplicationLayerControl,
                        host_to_ethercat(u16::from(EthercatState::SafeOperational)),
                        TIMEOUT_RET3,
                    )?;
                }
                if context.slavelist[slave_usize].block_logical_read_write != 0 {
                    context.grouplist[group_usize].block_logical_read_write += 1;
                }

                context.grouplist[group_usize].ebus_current +=
                    context.slavelist[slave_usize].ebus_current;
            }
        }

        context.grouplist[group_usize].io_segments[current_segment] = segment_size;
        context.grouplist[group_usize].used_segment_count = current_segment as u16 + 1;
        context.grouplist[group_usize].first_input_segment = 0;
        context.grouplist[group_usize].first_input_segment = 0;

        context.grouplist[group_usize].output_bytes =
            so_logical_address - context.grouplist[group_usize].logical_start_address;
        context.grouplist[group_usize].input_bytes =
            si_logical_address - context.grouplist[group_usize].logical_start_address;
        context.grouplist[group_usize].outputs = io_map;
        context.grouplist[group_usize].inputs =
            &io_map[context.grouplist[group_usize].output_bytes as usize..];

        // Move calculated inputs with output_bytes offset
        for slave in 1..=context.slave_count {
            let slave_usize = usize::from(slave);
            if (group == 0 || group == context.slavelist[slave_usize].group)
                && context.slavelist[slave_usize].input_bits > 0
            {
                if let Some(inputs) = context.slavelist[slave_usize].input {
                    context.slavelist[slave_usize].input =
                        Some(&inputs[context.grouplist[group_usize].output_bytes as usize..]);
                }
            }
        }

        if group == 0 {
            // Store output bytes in master record
            let slave = &mut context.slavelist[0];
            slave.outputs = io_map;
            slave.output_bytes =
                (so_logical_address - context.grouplist[group_usize].logical_start_address) as u16;
            slave.input = Some(&io_map[usize::from(slave.output_bytes)..]);
            slave.input_bytes =
                (si_logical_address - context.grouplist[group_usize].logical_start_address) as u16;
        }

        let io_map_size = context.grouplist[group_usize].output_bytes
            + context.grouplist[group_usize].input_bytes;
        ec_println!("IOmapSize {io_map_size}");

        Ok(io_map_size as usize)
    } else {
        Ok(0)
    }
}

/// Recover slave
///
/// # Parameters
/// `context`: Context struct
/// `slave`: Slave to recover
/// `timeout`: local timeout f.e. `TIMEOUT_RETURN3`
///
/// # Returns
/// `Ok(())` or Error
pub fn recover_slave(
    context: &mut Context,
    slave: u16,
    timeout: Duration,
) -> Result<(), ConfigError> {
    let slave_usize = usize::from(slave);
    let config_address = context.slavelist[slave_usize].config_address;
    let address_ph = 1_u16.wrapping_sub(slave);

    // Check if another slave than requested has been found
    let read_address: u16 = 0xFFFE;
    let mut word = read_address.to_ne_bytes();
    aprd(
        &mut context.port.lock().unwrap(),
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
            &mut context.port.lock().unwrap(),
            TEMP_NODE,
            EthercatRegister::StaDr,
            host_to_ethercat(0),
            Duration::default(),
        )?;

        // Set temporary node address of slave
        if apwrw(
            &mut context.port.lock().unwrap(),
            address_ph,
            EthercatRegister::StaDr,
            host_to_ethercat(TEMP_NODE),
            timeout,
        )
        .is_err()
        {
            fpwrw(
                &mut context.port.lock().unwrap(),
                TEMP_NODE,
                EthercatRegister::StaDr,
                host_to_ethercat(0),
                Duration::default(),
            )?;
            return Err(ConfigError::SlaveFailsToRespond);
        }

        // Temporary config address
        context.slavelist[slave_usize].config_address = TEMP_NODE;

        // Set eeprom control to master
        eeprom_to_master(context, slave);

        // Check if slave is the same as configured before
        if fprdw(
            &mut context.port.lock().unwrap(),
            TEMP_NODE,
            EthercatRegister::Alias,
            timeout,
        )? == host_to_ethercat(context.slavelist[slave_usize].alias_address)
            && read_eeprom(context, slave, SiiGeneralItem::Id.into(), TIMEOUT_EEP)
                == host_to_ethercat(context.slavelist[slave_usize].eep_id)
            && read_eeprom(
                context,
                slave,
                SiiGeneralItem::Manufacturer.into(),
                TIMEOUT_EEP,
            ) == host_to_ethercat(context.slavelist[slave_usize].eep_manufacturer)
            && read_eeprom(context, slave, SiiGeneralItem::Revision.into(), TIMEOUT_EEP)
                == host_to_ethercat(context.slavelist[slave_usize].eep_revision)
        {
            fpwrw(
                &mut context.port.lock().unwrap(),
                TEMP_NODE,
                EthercatRegister::StaDr,
                host_to_ethercat(config_address),
                timeout,
            )?;
            context.slavelist[slave_usize].config_address = config_address;
        } else {
            // Slave is not the expected one, remove config address
            fpwrw(
                &mut context.port.lock().unwrap(),
                TEMP_NODE,
                EthercatRegister::StaDr,
                host_to_ethercat(0),
                timeout,
            )?;
            context.slavelist[slave_usize].config_address = config_address;
            return Err(ConfigError::FoundWrongSlave);
        }
    }
    Ok(())
}

/// Reconfigure slave
///
/// # Parameters
/// `context`: Context struct
/// `slave`: Slave to reconfigure
/// `timeout`: Local timeout f.e. `TIMEOUT_RETURN3`
///
/// # Returns
/// slave state or error
pub fn reconfig_slave(
    context: &mut Context,
    slave: u16,
    timeout: Duration,
) -> Result<u16, ConfigError> {
    let slave_usize = usize::from(slave);
    let config_address = context.slavelist[slave_usize].config_address;
    fpwrw(
        &mut context.port.lock().unwrap(),
        config_address,
        EthercatRegister::ApplicationLayerControl,
        host_to_ethercat(EthercatState::Init.into()),
        timeout,
    )?;

    // Set EEPROM to PDI
    eeprom_to_pdi(context, slave);

    // Check state change init
    let mut state = statecheck(context, slave, EthercatState::Init, TIMEOUT_STATE);
    if state == EthercatState::Init.into() {
        // Program all enabled Sync Managers
        for sm_index in 0..MAX_SM {
            if context.slavelist[slave_usize].sync_manager[usize::from(sm_index)].start_address != 0
            {
                fpwr(
                    &mut context.port.lock().unwrap(),
                    config_address,
                    u16::from(EthercatRegister::SyncManager0)
                        + u16::from(sm_index) * size_of::<SyncManager>() as u16,
                    <&mut [u8]>::from(
                        &mut context.slavelist[slave_usize].sync_manager[usize::from(sm_index)],
                    ),
                    timeout,
                )?;
            }
        }
        fpwrw(
            &mut context.port.lock().unwrap(),
            config_address,
            EthercatRegister::ApplicationLayerControl,
            host_to_ethercat(EthercatState::PreOperational.into()),
            timeout,
        )?;

        // Check state change pre-operational
        if statecheck(context, slave, EthercatState::PreOperational, TIMEOUT_STATE)
            == EthercatState::PreOperational.into()
        {
            // Execute special slave configuration hook pre-operational to safe-operational

            // Only if registered
            if let Some(po2_so_config) = context.slavelist[slave_usize].po2_so_config {
                po2_so_config(slave);
            }

            // Only if registered
            if let Some(po2_so_configx) = context.slavelist[slave_usize].po2_so_configx {
                po2_so_configx(context, slave);
            }

            // Set safe operational status
            fpwrw(
                &mut context.port.lock().unwrap(),
                config_address,
                EthercatRegister::ApplicationLayerControl,
                host_to_ethercat(EthercatState::SafeOperational.into()),
                timeout,
            )?;

            // Check state change safe operational
            state = statecheck(
                context,
                slave,
                EthercatState::SafeOperational,
                TIMEOUT_STATE,
            );

            // Program configured FMMU
            for fmmu_count in 0..context.slavelist[slave_usize].fmmu_unused {
                fpwr(
                    &mut context.port.lock().unwrap(),
                    config_address,
                    u16::from(EthercatRegister::FieldbusMemoryManagementUnit0)
                        + size_of::<Fmmu>() as u16 * u16::from(fmmu_count),
                    <&mut [u8]>::from(
                        &mut context.slavelist[slave_usize].fmmu[usize::from(fmmu_count)],
                    ),
                    timeout,
                )?;
            }
        }
    }

    Ok(state)
}
