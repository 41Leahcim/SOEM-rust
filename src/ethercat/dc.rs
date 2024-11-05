//! Distributed Clock EtherCAT functions.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use crate::oshw::nicdrv::NicdrvError;

use super::{
    base::{bwr, fprdl, fprdll, fpwr},
    main::Context,
    r#type::{ethercat_to_host, host_to_ethercat, EthercatRegister, TIMEOUT_RETURN},
};

pub const PORTM: [u8; 4] = [1, 2, 4, 8];

/// First sync pulse delay
pub const SYNC_DELAY: Duration = Duration::from_millis(100);

#[derive(Debug, Default)]
pub struct DistributedClock {
    /// DC receivetimes on port A
    receive_time_a: Duration,

    /// DC receivetimes on port B
    receive_time_b: Duration,

    /// DC receivetimes on port C
    receive_time_c: Duration,

    /// DC receivetimes on port D
    receive_time_d: Duration,

    /// Next DC slave
    next: u16,

    /// Previous DC slave
    previous: u16,

    /// DC cycle received in nanoseconds
    cycle: Duration,

    /// DC shift from clock modulus boundary
    shift: i32,

    /// DC sync activation
    active: bool,
}

/// Set DC of slave to fire sync0 at `cycle_time` interval with `cycle_shift` offset.
///
/// # Parameters
/// - `context`: Context struct
/// - `slave`: Slave number
/// - `active`: true if active, faluse if deactivated
/// - `cycle_time`: Cycle time duration
/// - `cycle_shift`: Cycle shift in nanoseconds
///
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
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
    let slave_address = context.get_slave(slave).config_address();

    // Stop cyclic operation, ready for next trigger
    let mut ra = 0;
    fpwr(
        context.port_mut(),
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
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockControlUnit.into(),
        &mut [h],
        TIMEOUT_RETURN,
    )?;

    // Read local time of slave
    let time1 = ethercat_to_host(fprdll(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockSystemTime,
        TIMEOUT_RETURN,
    )?);

    // Calculate first trigger time, always a whole multiple of cycle_time rounded up +
    // the shift_time (can be negative).
    // This insures best synchronization between slaves, slaves with the same cycle_time
    // will synchronize at the same (the cycle_sift can be used to shift the synchronization).
    let time = host_to_ethercat(if cycle_time.is_zero() {
        (Duration::from_nanos(time1) + SYNC_DELAY).as_nanos() as i64 + i64::from(cycle_shift)
    } else {
        ((Duration::from_nanos(time1) + SYNC_DELAY) / cycle_time.as_nanos() as u32
            * cycle_time.as_nanos() as u32
            + cycle_time)
            .as_nanos() as i64
            + i64::from(cycle_shift)
    });

    // SYNC0 start time
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockStart0.into(),
        &mut time.to_bytes(),
        TIMEOUT_RETURN,
    )?;

    let time_cycle = host_to_ethercat(cycle_time.as_nanos() as u32);
    // SYNC0 cycle time
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockCycle0.into(),
        &mut time_cycle.to_bytes(),
        TIMEOUT_RETURN,
    )?;

    // Active cyclic operation
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockSynchronizationActive.into(),
        &mut [ra],
        TIMEOUT_RETURN,
    )?;

    // Update Slave state
    let slave = context.get_slave_mut(slave);
    if let Some(distributed_clock) = slave.distributed_clock_mut() {
        distributed_clock.active = active;
        distributed_clock.shift = cycle_shift;
        distributed_clock.cycle = cycle_time;
    } else {
        slave.set_distributed_clock(DistributedClock {
            active,
            shift: cycle_shift,
            cycle: cycle_time,
            ..Default::default()
        });
    }
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
/// # Errors
/// Returns an error if:
/// - A message couldn't be send/received
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
    // Sync1 can be used as a multiple of sync0, use true cycle time
    let true_cycle_time =
        cycle_time0 * (cycle_time1.as_nanos() / cycle_time0.as_nanos() + 1) as u32;

    let slave_address = context.get_slave(slave).config_address();
    let mut ra = 0;

    // Stop cyclic operation, ready for next trigger
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockSynchronizationActive.into(),
        &mut [ra],
        TIMEOUT_RETURN,
    )?;

    if active {
        // Act cyclic operation and sync0 + sync1
        ra = 1 + 2 + 4;
    }

    // Write access to ethercat
    let h = 0;
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockControlUnit.into(),
        &mut [h],
        TIMEOUT_RETURN,
    )?;

    // Read local time of slave

    let time1 = ethercat_to_host(fprdll(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockSystemTime,
        TIMEOUT_RETURN,
    )?);

    // Calculate first trigger time, always a whole multiple of `true_cycle_time` rounded up
    // + the shift_time (can be negative).
    // This insures best synchronization between slaves, slaves with the same cycle_time
    // will synchronize at the same moment (cycle_shift can be used to shift the synchronization).
    let time = host_to_ethercat(if cycle_time0.is_zero() {
        (Duration::from_nanos(time1) + SYNC_DELAY).as_nanos() as i64 + i64::from(cycle_shift)
    } else {
        ((Duration::from_nanos(time1) + SYNC_DELAY) / true_cycle_time.as_nanos() as u32
            * true_cycle_time.as_nanos() as u32
            + true_cycle_time)
            .as_nanos() as i64
            + i64::from(cycle_shift)
    });

    // SYNC0 start time
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockStart0.into(),
        &mut time.to_bytes(),
        TIMEOUT_RETURN,
    )?;

    // SYNC0 cycle time
    let mut time_cycle = host_to_ethercat(cycle_time0.as_nanos() as u32);
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockCycle0.into(),
        &mut time_cycle.to_bytes(),
        TIMEOUT_RETURN,
    )?;

    time_cycle = host_to_ethercat(cycle_time1.as_nanos() as u32);
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockCycle1.into(),
        &mut time_cycle.to_bytes(),
        TIMEOUT_RETURN,
    )?;
    fpwr(
        context.port_mut(),
        slave_address,
        EthercatRegister::DistributedClockSynchronizationActive.into(),
        &mut [ra],
        TIMEOUT_RETURN,
    )?;

    // Update Slave state
    if let Some(distributed_clock) = context.get_slave_mut(slave).distributed_clock_mut() {
        distributed_clock.active = active;
        distributed_clock.shift = cycle_shift;
        distributed_clock.cycle = cycle_time0;
    } else {
        context
            .get_slave_mut(slave)
            .set_distributed_clock(DistributedClock {
                active,
                shift: cycle_shift,
                cycle: cycle_time0,
                ..Default::default()
            });
    }
    Ok(())
}

/// Latched port time of slave
fn port_time(context: &mut Context, slave: u16, port: u8) -> Option<Duration> {
    let slave = context.get_slave_mut(slave).distributed_clock_mut()?;
    Some(match port {
        0 => slave.receive_time_a,
        1 => slave.receive_time_b,
        2 => slave.receive_time_c,
        3 => slave.receive_time_d,
        _ => Duration::default(),
    })
}

/// Calculate previous active port of slave
fn previous_port(context: &mut Context, slave: u16, port: u8) -> u8 {
    let a_port = context.get_slave(slave).active_ports();
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
    let mut consumed_ports = context.get_slave(parent).consumed_ports();
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
    *context.get_slave_mut(parent).consumed_ports_mut() = consumed_ports;
    parent_port
}

/// Locate DC slaves, measure propagation delays
///
/// # Parameters
/// `context`: Context struct
///
/// # Errors
/// Returns an error if:
/// - The Digital clock receive time of the slaves couldn't be latched
/// - The offset couldn't be send be saved in the offset register
/// - The propagation delay couldn't be send to be written
///
/// # Returns
/// Whether any slaves were found with Digital Clock
pub fn config_dc(context: &mut Context) -> Result<bool, NicdrvError> {
    context.get_slave_mut(0).remove_distributed_clock();
    context.get_group_mut(0).remove_distributed_clock();

    // Latch Digital clock receive time a of all slaves
    let mut long = [0; 4];
    bwr(
        context.port_mut(),
        0,
        EthercatRegister::DistributedClockTime0,
        &mut long,
        TIMEOUT_RETURN,
    )?;
    let mut ht;

    // EtherCAT uses 2000-01-01 as epoch start instead of 1970-01-01
    let mastertime = SystemTime::now() - Duration::from_secs(946_684_800);
    let mastertime64 = mastertime.duration_since(UNIX_EPOCH).unwrap_or_default();
    let mastertime64 = mastertime64.as_secs() * 1_000_000 + u64::from(mastertime64.subsec_nanos());

    let mut previous_dc_slave = 0;
    let mut parent_hold = 0;
    for i in 1..=context.slave_count() {
        let mut distributed_clock = DistributedClock::default();
        if context.get_slave(i).distributed_clock().is_some() {
            if let Some(previous_dc) = context
                .get_slave_mut(previous_dc_slave)
                .distributed_clock_mut()
            {
                previous_dc.next = i;
                distributed_clock.previous = previous_dc_slave;
            } else {
                context
                    .get_slave_mut(0)
                    .set_distributed_clock(DistributedClock {
                        next: i,
                        ..Default::default()
                    });
                distributed_clock.previous = 0;
                context
                    .get_group_mut(context.get_slave(i).group())
                    .set_distributed_clock_next(i);
            }

            // This branch has DC slave so remove parenthold
            previous_dc_slave = i;
            let slave_address = context.get_slave(i).config_address();
            ht = fprdl(
                context.port_mut(),
                slave_address,
                EthercatRegister::DistributedClockTime0,
                TIMEOUT_RETURN,
            )?;
            distributed_clock.receive_time_a =
                Duration::from_nanos(u64::from(ethercat_to_host(ht)));

            // 64-bit latched DC receive time A of each specific slave

            // Use it as offset in order to set local time around 0 + mastertime
            let hrt = host_to_ethercat(
                -((ethercat_to_host(fprdll(
                    context.port_mut(),
                    slave_address,
                    EthercatRegister::DistributedClockStartOfFrame,
                    TIMEOUT_RETURN,
                )?) + mastertime64) as i64),
            );

            // Save it in the offset register
            fpwr(
                context.port_mut(),
                slave_address,
                EthercatRegister::DistributedClockSystemOffset.into(),
                &mut hrt.to_bytes(),
                TIMEOUT_RETURN,
            )?;

            distributed_clock.receive_time_b =
                Duration::from_nanos(u64::from(ethercat_to_host(fprdl(
                    context.port_mut(),
                    slave_address,
                    EthercatRegister::DistributedClockTime1,
                    TIMEOUT_RETURN,
                )?)));
            distributed_clock.receive_time_c =
                Duration::from_nanos(u64::from(ethercat_to_host(fprdl(
                    context.port_mut(),
                    slave_address,
                    EthercatRegister::DistributedClockTime2,
                    TIMEOUT_RETURN,
                )?)));
            distributed_clock.receive_time_d =
                Duration::from_nanos(u64::from(ethercat_to_host(fprdl(
                    context.port_mut(),
                    slave_address,
                    EthercatRegister::DistributedClockTime3,
                    TIMEOUT_RETURN,
                )?)));

            // Make list of active ports and their time stamps
            let mut plist: [u8; 4] = [0; 4];
            let mut time_list = [Duration::default(); 4];
            let mut nlist = 0;
            if context.get_slave(i).active_ports() & PORTM[0] != 0 {
                plist[nlist] = 0;
                time_list[nlist] = distributed_clock.receive_time_a;
                nlist += 1;
            }
            if context.get_slave(i).active_ports() & PORTM[3] != 0 {
                plist[nlist] = 3;
                time_list[nlist] = distributed_clock.receive_time_d;
                nlist += 1;
            }
            if context.get_slave(i).active_ports() & PORTM[1] != 0 {
                plist[nlist] = 1;
                time_list[nlist] = distributed_clock.receive_time_b;
                nlist += 1;
            }
            if context.get_slave(i).active_ports() & PORTM[2] != 0 {
                plist[nlist] = 2;
                time_list[nlist] = distributed_clock.receive_time_c;
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
            *context.get_slave_mut(i).entry_port_mut() = entry_port;

            // Consume entryport from active ports
            *context.get_slave_mut(i).consumed_ports_mut() &= !(1 << entry_port);

            // Finding digital clock parent of current
            let mut parent = i;
            let mut child;
            loop {
                child = parent;
                parent = u16::from(context.get_slave(parent).parent());
                if parent == 0 || context.get_slave(parent).distributed_clock().is_none() {
                    break;
                }
            }

            // Only calculate propagation delay if slave is not the first
            if parent > 0 {
                // Find port on parent this slave is connected to
                *context.get_slave_mut(i).parent_mut() = parent_port(context, parent);
                if context.get_slave(parent).topology() == 1 {
                    *context.get_slave_mut(i).parent_port_mut() =
                        context.get_slave(parent).entry_port();
                }

                // Delta time of parentport - 1 - parentport
                // Note: order of ports is 0 - 3 - 1 - 2
                // Non active ports are skipped
                let previous_active_port_parent =
                    previous_port(context, parent, context.get_slave(i).parent_port());
                let delta_time3 = port_time(context, parent, context.get_slave(i).parent_port())
                    .unwrap_or_default()
                    .abs_diff(
                        port_time(context, parent, previous_active_port_parent).unwrap_or_default(),
                    );

                // Current slave has children.
                // Those children's delays need to be subtracted
                let previous_active_port =
                    previous_port(context, i, context.get_slave(i).entry_port());
                let delta_time1 = if context.get_slave(i).topology() > 1 {
                    port_time(context, i, previous_active_port)
                        .unwrap_or_default()
                        .abs_diff(
                            port_time(context, i, context.get_slave(i).entry_port())
                                .unwrap_or_default(),
                        )
                } else {
                    Duration::default()
                };

                // Current slave is not the first child of parent
                // Previous child's delays need to be added
                let delta_time2 = if child - parent > 1 {
                    port_time(context, parent, previous_active_port_parent)
                        .unwrap_or_default()
                        .abs_diff(
                            port_time(context, parent, context.get_slave(parent).entry_port())
                                .unwrap_or_default(),
                        )
                } else {
                    Duration::default()
                };

                // Calculate current slave delay from delta times.
                // Assumption: Forward delay equals return delay
                *context.get_slave_mut(i).propagation_delay_mut() = ((delta_time3 - delta_time1)
                    / 2)
                    + delta_time2
                    + context.get_slave(parent).propagation_delay();

                // Write propagation delay
                let ht =
                    host_to_ethercat(context.get_slave(i).propagation_delay().as_nanos() as i32);
                fpwr(
                    context.port_mut(),
                    slave_address,
                    EthercatRegister::DistributedClockSystemDelay.into(),
                    &mut ht.to_bytes(),
                    TIMEOUT_RETURN,
                )?;
            }
        } else {
            distributed_clock.receive_time_a = Duration::default();
            distributed_clock.receive_time_b = Duration::default();
            distributed_clock.receive_time_c = Duration::default();
            distributed_clock.receive_time_d = Duration::default();
            let parent = context.get_slave(i).parent();

            // If non DC slave found on first position on branch, hold root parent
            if parent > 0 && context.get_slave(u16::from(parent)).topology() > 2 {
                parent_hold = u16::from(parent);
            }

            // If branch has no DC slaves, consume port on root parent
            if parent_hold != 0 && context.get_slave(i).topology() == 1 {
                parent_port(context, parent_hold);
                parent_hold = 0;
            }
        }
    }

    Ok(context.get_slave(0).distributed_clock().is_some())
}
