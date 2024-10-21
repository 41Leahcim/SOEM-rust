//! Distributed Clock EtherCAT functions.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::oshw::nicdrv::NicdrvError;

use super::{
    base::{bwr, fprd, fpwr},
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

/// Locate DC slaves, measure propagation delays
///
/// # Parameters
/// `context`: Context struct
///
/// # Returns
/// Whether any slaves were found with Digital Clock
pub fn config_dc(context: &mut Context) -> Result<bool, NicdrvError> {
    context.slavelist[0].has_dc = false;
    context.grouplist[0].has_dc = false;

    // Latch Digital clock receive time a of all slaves
    let mut long = [0; 4];
    bwr(
        &mut context.port.lock().unwrap(),
        0,
        EthercatRegister::DistributedClockTime0,
        &mut long,
        TIMEOUT_RETURN,
    )?;
    let mut ht;

    // EtherCAT uses 2000-01-01 as epoch start instead of 1970-01-01
    let mastertime = SystemTime::now() - Duration::from_secs(946684800);
    let mastertime64 = mastertime.duration_since(UNIX_EPOCH).unwrap();
    let mastertime64 = mastertime64.as_secs() * 1_000_000 + mastertime64.subsec_nanos() as u64;

    let mut previous_dc_slave = 0;
    let mut parent_hold = 0;
    for i in 1..=context.slave_count {
        let i_usize = usize::from(i);
        if context.slavelist[i_usize].has_dc {
            if !context.slavelist[0].has_dc {
                context.slavelist[0].has_dc = true;
                context.slavelist[0].dc_next = i;
                context.slavelist[i_usize].dc_previous = 0;
                context.grouplist[usize::from(context.slavelist[i_usize].group)].has_dc = true;
                context.grouplist[usize::from(context.slavelist[i_usize].group)].dc_next = i;
            } else {
                context.slavelist[usize::from(previous_dc_slave)].dc_next = i;
                context.slavelist[i_usize].dc_previous = previous_dc_slave;
            }

            // This branch has DC slave so remove parenthold
            previous_dc_slave = i;
            let slave_address = context.slavelist[i_usize].config_address;
            fprd(
                &mut context.port.lock().unwrap(),
                slave_address,
                EthercatRegister::DistributedClockTime0,
                &mut long,
                TIMEOUT_RETURN,
            )?;
            ht = i32::from_ne_bytes(long);
            context.slavelist[i_usize].dc_rt_a = Duration::from_nanos(ethercat_to_host(ht) as u64);

            // 64-bit latched DC receive time A of each specific slave
            let mut long_long = [0; 8];
            fprd(
                &mut context.port.lock().unwrap(),
                slave_address,
                EthercatRegister::DistributedClockStartOfFrame,
                &mut long_long,
                TIMEOUT_RETURN,
            )?;

            // Use it as offset in order to set local time around 0 + mastertime
            let hrt = host_to_ethercat(-ethercat_to_host(
                i64::from_ne_bytes(long_long) + mastertime64 as i64,
            ));

            // Save it in the offset register
            fpwr(
                &mut context.port.lock().unwrap(),
                slave_address,
                EthercatRegister::DistributedClockSystemOffset.into(),
                &mut hrt.to_ne_bytes(),
                TIMEOUT_RETURN,
            )?;
            fprd(
                &mut context.port.lock().unwrap(),
                slave_address,
                EthercatRegister::DistributedClockTime1,
                &mut long,
                TIMEOUT_RETURN,
            )?;
            context.slavelist[i_usize].dc_rt_b =
                Duration::from_nanos(ethercat_to_host(i32::from_ne_bytes(long)) as u64);
            fprd(
                &mut context.port.lock().unwrap(),
                slave_address,
                EthercatRegister::DistributedClockTime2,
                &mut long,
                TIMEOUT_RETURN,
            )?;
            context.slavelist[i_usize].dc_rt_c =
                Duration::from_nanos(ethercat_to_host(i32::from_ne_bytes(long)) as u64);
            fprd(
                &mut context.port.lock().unwrap(),
                slave_address,
                EthercatRegister::DistributedClockTime3,
                &mut long,
                TIMEOUT_RETURN,
            )?;
            context.slavelist[i_usize].dc_rt_d =
                Duration::from_nanos(ethercat_to_host(i32::from_ne_bytes(long)) as u64);

            // Make list of active ports and their time stamps
            let mut plist: [u8; 4] = [0; 4];
            let mut time_list = [Duration::default(); 4];
            let mut nlist = 0;
            if context.slavelist[i_usize].active_ports & PORTM[0] != 0 {
                plist[nlist] = 0;
                time_list[nlist] = context.slavelist[i_usize].dc_rt_a;
                nlist += 1;
            }
            if context.slavelist[i_usize].active_ports & PORTM[3] != 0 {
                plist[nlist] = 3;
                time_list[nlist] = context.slavelist[i_usize].dc_rt_d;
                nlist += 1;
            }
            if context.slavelist[i_usize].active_ports & PORTM[1] != 0 {
                plist[nlist] = 1;
                time_list[nlist] = context.slavelist[i_usize].dc_rt_b;
                nlist += 1;
            }
            if context.slavelist[i_usize].active_ports & PORTM[2] != 0 {
                plist[nlist] = 2;
                time_list[nlist] = context.slavelist[i_usize].dc_rt_c;
                nlist += 1;
            }

            // Entryport is port with the lowest timestamp
            let mut entry_port = 0;
            for i in 1..=3 {
                let i_usize = usize::from(i);
                if nlist > i_usize && time_list[i_usize] < time_list[usize::from(entry_port)] {
                    entry_port = i;
                }
            }
            entry_port = plist[usize::from(entry_port)];
            context.slavelist[i_usize].entry_port = entry_port;

            // Consume entryport from active ports
            context.slavelist[i_usize].consumed_ports &= !(1 << entry_port);

            // Finding digital clock parent of current
            let mut parent = i;
            let mut child;
            loop {
                child = parent;
                parent = u16::from(context.slavelist[usize::from(parent)].parent);
                if parent == 0 || context.slavelist[usize::from(parent)].has_dc {
                    break;
                }
            }

            // Only calculate propagation delay if slave is not the first
            if parent > 0 {
                // Find port on parent this slave is connected to
                context.slavelist[i_usize].parent = parent_port(context, parent);
                if context.slavelist[usize::from(parent)].topology == 1 {
                    context.slavelist[i_usize].parent_port =
                        context.slavelist[usize::from(parent)].entry_port;
                }

                // Delta time of parentport - 1 - parentport
                // Note: order of ports is 0 - 3 - 1 - 2
                // Non active ports are skipped
                let previous_active_port_parent =
                    previous_port(context, parent, context.slavelist[i_usize].parent_port);
                let delta_time3 =
                    port_time(context, parent, context.slavelist[i_usize].parent_port)
                        .abs_diff(port_time(context, parent, previous_active_port_parent));

                // Current slave has children.
                // Those children's delays need to be subtracted
                let previous_active_port =
                    previous_port(context, i, context.slavelist[i_usize].entry_port);
                let delta_time1 = if context.slavelist[i_usize].topology > 1 {
                    port_time(context, i, previous_active_port).abs_diff(port_time(
                        context,
                        i,
                        context.slavelist[i_usize].entry_port,
                    ))
                } else {
                    Duration::default()
                };

                // Current slave is not the first child of parent
                // Previous child's delays need to be added
                let delta_time2 = if child - parent > 1 {
                    port_time(context, parent, previous_active_port_parent).abs_diff(port_time(
                        context,
                        parent,
                        context.slavelist[usize::from(parent)].entry_port,
                    ))
                } else {
                    Duration::default()
                };

                // Calculate current slave delay from delta times.
                // Assumption: Forward delay equals return delay
                context.slavelist[i_usize].propagation_delay = ((delta_time3 - delta_time1) / 2)
                    + delta_time2
                    + context.slavelist[usize::from(parent)].propagation_delay;

                // Write propagation delay
                ht = host_to_ethercat(
                    context.slavelist[i_usize].propagation_delay.as_nanos() as i32
                );
                fpwr(
                    &mut context.port.lock().unwrap(),
                    slave_address,
                    EthercatRegister::DistributedClockSystemDelay.into(),
                    &mut ht.to_ne_bytes(),
                    TIMEOUT_RETURN,
                )?;
            }
        } else {
            context.slavelist[i_usize].dc_rt_a = Duration::default();
            context.slavelist[i_usize].dc_rt_b = Duration::default();
            context.slavelist[i_usize].dc_rt_c = Duration::default();
            context.slavelist[i_usize].dc_rt_d = Duration::default();
            let parent = context.slavelist[i_usize].parent;

            // If non DC slave found on first position on branch, hold root parent
            if parent > 0 && context.slavelist[usize::from(parent)].topology > 2 {
                parent_hold = u16::from(parent);
            }

            // If branch has no DC slaves, consume port on root parent
            if parent_hold != 0 && context.slavelist[i_usize].topology == 1 {
                parent_port(context, parent_hold);
                parent_hold = 0;
            }
        }
    }

    Ok(context.slavelist[0].has_dc)
}
