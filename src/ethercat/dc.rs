//! Distributed Clock EtherCAT functions.

use std::time::Duration;

use crate::oshw::nicdrv::NicdrvError;

use super::{
    base::{fprd, fpwr},
    main::Context,
    r#type::{ethercat_to_host, host_to_ethercat, EthercatRegister, TIMEOUT_RETURN},
};

pub const PORTM: [u8; 4] = [1, 2, 4, 8];

/// First sync pulse delay
pub const SYNC_DELAY: Duration = Duration::from_millis(100);

/// Set DC of slave to fire sync0 at `cycle_time` interval with `cycle_shift` offset.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `active`: true if active, faluse if deactivated
/// - `cycle_time`: Cycle time duration
/// - `cycle_shift`: Cycle shift in nanoseconds
///
/// # Returns
/// Ok(()) or NicdrvError
pub fn dsync0(
    context: &mut Context,
    slave: u16,
    active: bool,
    cycle_time: Duration,
    cycle_shift: i32,
) -> Result<(), NicdrvError> {
    let slave_usize = usize::from(slave);
    let slave_address = context.slavelist[slave_usize].config_address;

    // Stop cyclic operation, ready for next trigger
    let mut ra = 0;
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockSynchronizationActive.into(),
        &mut [ra],
        TIMEOUT_RETURN,
    )?;

    // Act cyclic operation and sync0, sync1 deactivated
    if active {
        ra = 1 + 2;
    }

    // Write access to ethercat
    let h = 0;
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockControlUnit.into(),
        &mut [h],
        TIMEOUT_RETURN,
    )?;

    // Read local time of slave
    let mut time1 = [0; 8];
    fprd(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockSystemTime,
        &mut time1,
        TIMEOUT_RETURN,
    )?;
    let time1 = ethercat_to_host(u64::from_ne_bytes(time1));

    // Calculate first trigger time, always a whole multiple of cycle_time rounded up +
    // the shift_time (can be negative).
    // This insures best synchronization between slaves, slaves with the same cycle_time
    // will synchronize at the same (the cycle_sift can be used to shift the synchronization).
    let time = host_to_ethercat(if !cycle_time.is_zero() {
        ((Duration::from_nanos(time1) + SYNC_DELAY) / cycle_time.as_nanos() as u32
            * cycle_time.as_nanos() as u32
            + cycle_time)
            .as_nanos() as i64
            + i64::from(cycle_shift)
    } else {
        (Duration::from_nanos(time1) + SYNC_DELAY).as_nanos() as i64 + i64::from(cycle_shift)
    });

    // SYNC0 start time
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockStart0.into(),
        &mut time.to_ne_bytes(),
        TIMEOUT_RETURN,
    )?;

    let time_cycle = host_to_ethercat(cycle_time.as_nanos() as u32);
    // SYNC0 cycle time
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockCycle0.into(),
        &mut time_cycle.to_ne_bytes(),
        TIMEOUT_RETURN,
    )?;

    // Active cyclic operation
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockSynchronizationActive.into(),
        &mut [ra],
        TIMEOUT_RETURN,
    )?;

    // Update Slave state
    context.slavelist[slave_usize].dc_active = active;
    context.slavelist[slave_usize].dc_shift = cycle_shift;
    context.slavelist[slave_usize].dc_cycle = cycle_time;
    Ok(())
}

/// Set distributed clock of slave to fire sync0 and sync1 at cycle_time interval with cycle_shift
/// offset.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `active`: true if active, false if deactivated
/// - `cycle_time0`: Cycle time SYNC0 duration
/// - `cycle_time1`: Cycle time SYNC1 duration. This time is a delta time in relation to
///                  the sync0 fire. If `cycle_time1` == 0, SYNC1 fires at the same time
///                  as SYNC0.
/// - `cycle_shift`: Cycleshift in nanoseconds
///
/// # Returns
/// `Ok(())` or `NicdrvError`
pub fn dsync01(
    context: &mut Context,
    slave: u16,
    active: bool,
    cycle_time0: Duration,
    cycle_time1: Duration,
    cycle_shift: i32,
) -> Result<(), NicdrvError> {
    let slave_usize = usize::from(slave);

    // Sync1 can be used as a multiple of sync0, use true cycle time
    let true_cycle_time =
        cycle_time0 * (cycle_time1.as_nanos() / cycle_time0.as_nanos() + 1) as u32;

    let slave_address = context.slavelist[slave_usize].config_address;
    let ra = 0;

    // Stop cyclic operation, ready for next trigger
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockSynchronizationActive.into(),
        &mut [ra],
        TIMEOUT_RETURN,
    )?;

    // Stop cyclic operation, ready for next trigger
    if active {
        // Act cyclic operation and sync0 + sync1
    }

    // Write access to ethercat
    let h = 0;
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockControlUnit.into(),
        &mut [h],
        TIMEOUT_RETURN,
    )?;

    // Read local time of slave
    let mut time1 = [0; 8];
    fprd(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockSystemTime,
        &mut time1,
        TIMEOUT_RETURN,
    )?;
    let time1 = ethercat_to_host(u64::from_ne_bytes(time1));

    // Calculate first trigger time, always a whole multiple of `true_cycle_time` rounded up
    // + the shift_time (can be negative).
    // This insures best synchronization between slaves, slaves with the same cycle_time
    // will synchronize at the same moment (cycle_shift can be used to shift the synchronization).
    let time = host_to_ethercat(if !cycle_time0.is_zero() {
        ((Duration::from_nanos(time1) + SYNC_DELAY) / true_cycle_time.as_nanos() as u32
            * true_cycle_time.as_nanos() as u32
            + true_cycle_time)
            .as_nanos() as i64
            + i64::from(cycle_shift)
    } else {
        (Duration::from_nanos(time1) + SYNC_DELAY).as_nanos() as i64 + i64::from(cycle_shift)
    });

    // SYNC0 start time
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockStart0.into(),
        &mut time.to_ne_bytes(),
        TIMEOUT_RETURN,
    )?;

    // SYNC0 cycle time
    let mut time_cycle = host_to_ethercat(cycle_time0.as_nanos() as u32);
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockCycle0.into(),
        &mut time_cycle.to_ne_bytes(),
        TIMEOUT_RETURN,
    )?;

    time_cycle = host_to_ethercat(cycle_time1.as_nanos() as u32);
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockCycle1.into(),
        &mut time_cycle.to_ne_bytes(),
        TIMEOUT_RETURN,
    )?;
    fpwr(
        &mut context.port.lock().unwrap(),
        slave_address,
        EthercatRegister::DistributedClockSynchronizationActive.into(),
        &mut [ra],
        TIMEOUT_RETURN,
    )?;

    // Update Slave state
    context.slavelist[slave_usize].dc_active = active;
    context.slavelist[slave_usize].dc_shift = cycle_shift;
    context.slavelist[slave_usize].dc_cycle = cycle_time0;
    Ok(())
}

/// Latched port time of slave
fn port_time(context: &mut Context, slave: u16, port: u8) -> Duration {
    let slave = &context.slavelist[usize::from(slave)];
    match port {
        0 => slave.dc_rt_a,
        1 => slave.dc_rt_b,
        2 => slave.dc_rt_c,
        3 => slave.dc_rt_d,
        _ => Duration::default(),
    }
}

/// Calculate previous active port of slave
fn previous_port(context: &mut Context, slave: u16, port: u8) -> u8 {
    let a_port = context.slavelist[usize::from(slave)].active_ports;
    match port {
        0 => {
            if a_port & PORTM[2] != 0 {
                2
            } else if a_port & PORTM[1] != 0 {
                1
            } else if a_port & PORTM[3] != 0 {
                3
            } else {
                port
            }
        }
        1 => {
            if a_port & PORTM[3] != 0 {
                3
            } else if a_port & PORTM[0] != 0 {
                0
            } else if a_port & PORTM[2] != 0 {
                2
            } else {
                port
            }
        }
        2 => {
            if a_port & PORTM[1] != 0 {
                1
            } else if a_port & PORTM[3] != 0 {
                3
            } else if a_port & PORTM[0] != 0 {
                0
            } else {
                port
            }
        }
        3 => {
            if a_port & PORTM[0] != 0 {
                0
            } else if a_port & PORTM[2] != 0 {
                2
            } else if a_port & PORTM[1] != 0 {
                1
            } else {
                port
            }
        }
        _ => port,
    }
}

/// Search unconsumed ports in parent, consume and return first open port
fn parent_port(context: &mut Context, parent: u16) -> u8 {
    // Search order is important here, 3 - 1 - 2 - 0
    let mut consumed_ports = context.slavelist[usize::from(parent)].consumed_ports;
    let parent_port = if consumed_ports & PORTM[3] != 0 {
        consumed_ports &= !PORTM[3];
        3
    } else if consumed_ports & PORTM[1] != 0 {
        consumed_ports &= !PORTM[1];
        1
    } else if consumed_ports & PORTM[2] != 0 {
        consumed_ports &= !PORTM[2];
        2
    } else {
        consumed_ports &= !PORTM[0];
        0
    };
    context.slavelist[usize::from(parent)].consumed_ports = consumed_ports;
    parent_port
}

pub fn config_dc(context: &mut Context) -> bool {
    todo!()
}
