//! Setting up a datagram in an ethernet frame.
//! EtherCAT datagram primitives, broadcast, auto increment, configured and
//! logical adddressed data transfers. All base transfers are blocking, so
//! wait for the frame to be returned to the master or timeout. If this is
//! not acceptable, build your own datagrams and use the functions from nicdrv.rs

use std::time::Duration;

use crate::{
    ethercat::r#type::{ethercat_to_host, DATAGRAM_FOLLOWS, ETHERCAT_LENGTH_SIZE},
    oshw::nicdrv::{get_index, set_buf_stat, src_confirm, NicdrvError, Port},
};

use super::r#type::{
    high_word, host_to_ethercat, low_word, Buffer, BufferState, CommandType, Ethercat,
    EthercatHeader, EthercatRegister, ECATTYPE, ETHERCAT_COMMAND_OFFET, ETHERCAT_HEADER_SIZE,
    ETHERCAT_WORK_COUNTER_SIZE, ETHERNET_HEADER_SIZE,
};

/// Write data to EtherCAT datagram
///
/// # Parameters
/// - `datagramdata`: data part of datagram
/// - `command`: Command being executed
/// - `length`: Length of databuffer
/// - `data`: Databuffer to be copied into datagram
fn write_datagram_data(datagram_data: &mut [u8], command: CommandType, data: &[u8]) {
    if !data.is_empty() {
        match command {
            CommandType::Nop
            | CommandType::AutoPointerRead
            | CommandType::FixedPointerRead
            | CommandType::BroadcastRead
            | CommandType::LogicalRead => {
                // No data to write, initialize data so frame is in a known state
                datagram_data.iter_mut().for_each(|byte| *byte = 0);
            }
            _ => datagram_data
                .iter_mut()
                .zip(data)
                .for_each(|(dest, src)| *dest = *src),
        }
    }
}

/// Generate and set EtherCAT datagram in a standard Ethernet frame.
///
/// # Parameters
/// - `frame`: framebuffer
/// - `command`: Command to execute
/// - `index`: Index used for TX and RX buffers
/// - `address_position`: Address position
/// - `address_offset`: Address offset
/// - `length`: Length of datagram excluding EtherCAT header
/// - `data`: Databuffer to be compied in datagram
pub fn setup_datagram(
    frame: &mut Buffer,
    command: CommandType,
    index: u8,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
) {
    // Ethernet header is preset and fixed in frame buffers.
    // EtherCAT header needs to be added after that.
    frame.resize(ETHERNET_HEADER_SIZE, 0).unwrap();
    frame
        .extend_from_slice(
            EthercatHeader {
                ethercat_length: host_to_ethercat(
                    (usize::from(ECATTYPE) + ETHERCAT_HEADER_SIZE + data.len()) as u16,
                ),
                command,
                index,
                address_position: host_to_ethercat(address_position),
                address_offset: host_to_ethercat(address_offset),
                data_length: host_to_ethercat(data.len() as u16),
                interrupt: 0,
            }
            .as_ref(),
        )
        .unwrap();
    frame.extend_from_slice(data).unwrap();

    // Set worker count to 0
    frame.extend_from_slice(&[0; 2]).unwrap();
    frame
        .resize(
            ETHERNET_HEADER_SIZE + ETHERCAT_HEADER_SIZE + ETHERCAT_WORK_COUNTER_SIZE + data.len(),
            0,
        )
        .unwrap();
}

/// Add EtherCAT datagram to a standard ethernet frame with existing datagram(s).
///
/// # Parameters
/// - `frame`: Framebuffer
/// - `command`: Command
/// - `index`: index used for TX and RX buffers
/// - `more`: true if still more datagrams will follow
/// - `address_position`: Address position
/// - `address_offset`: Address offset
/// - `data`: Databuffer to be copied in datagram
///
/// # Returns
/// Offset to data in rx frame, usefull to retrieve data after RX
pub fn add_datagram(
    frame: &mut Buffer,
    command: CommandType,
    index: u8,
    more: bool,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
) -> usize {
    // Copy previous frame size
    let previous_length = frame.len();

    // Set size of frame in buffer array
    frame
        .resize(
            previous_length + ETHERCAT_HEADER_SIZE - ETHERCAT_LENGTH_SIZE
                + ETHERCAT_WORK_COUNTER_SIZE
                + data.len(),
            0,
        )
        .unwrap();

    let mut datagram = EthercatHeader::try_from(&frame[ETHERNET_HEADER_SIZE..]).unwrap();

    // Add new datargam to ethernet frame size
    datagram.ethercat_length = host_to_ethercat(
        (usize::from(ethercat_to_host(datagram.ethercat_length))
            + ETHERCAT_HEADER_SIZE
            + data.len()) as u16,
    );
    datagram.data_length =
        host_to_ethercat(ethercat_to_host(datagram.data_length) | DATAGRAM_FOLLOWS);
    frame[ETHERNET_HEADER_SIZE..].copy_from_slice(datagram.as_ref());

    // Set new EtherCAT header position
    datagram = EthercatHeader::try_from(&frame[previous_length - ETHERCAT_LENGTH_SIZE..]).unwrap();
    datagram.command = command;
    datagram.index = index;
    datagram.address_position = host_to_ethercat(address_position);
    datagram.address_offset = host_to_ethercat(address_offset);
    datagram.data_length = host_to_ethercat(if more {
        // This is not the last datagram to add
        data.len() as u16 | DATAGRAM_FOLLOWS
    } else {
        // This is the last datagram to add
        data.len() as u16
    });
    frame[previous_length - ETHERCAT_LENGTH_SIZE..].copy_from_slice(datagram.as_ref());

    write_datagram_data(
        &mut frame[previous_length + ETHERCAT_HEADER_SIZE - ETHERCAT_LENGTH_SIZE..],
        command,
        data,
    );

    // Set Work Counter to 0
    frame
        .iter_mut()
        .skip(previous_length + ETHERCAT_HEADER_SIZE - ETHERCAT_LENGTH_SIZE + data.len())
        .for_each(|byte| *byte = 0);

    // Return offset to data in rx frame, 14 bytes smaller than tx frame due to stripping of
    // thernet header.
    previous_length + ETHERCAT_HEADER_SIZE - ETHERCAT_LENGTH_SIZE - ETHERCAT_HEADER_SIZE
}

/// Executes a primitive command
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, normally 0
/// - `address_offsrt`: Address offset, slave memory address
/// - `data`: databuffer tobe written to or receive from slave
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
/// - `command`: command to execute
///
/// # Returns Workcounter or error
fn execute_primitive_command(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
    timeout: Duration,
    command: CommandType,
) -> Result<u16, NicdrvError> {
    // Get fresh index
    let index = get_index(port);

    // Setup datagram
    setup_datagram(
        &mut port.tx_buffers.lock().unwrap()[usize::from(index)],
        command,
        index,
        address_position,
        address_offset,
        data,
    );

    // Send data and wait for answer
    let wkc = src_confirm(port, index, timeout)?;

    match command {
        CommandType::Nop
        | CommandType::AutoPointerWrite
        | CommandType::FixedPointerWrite
        | CommandType::BroadcastWrite
        | CommandType::LogicalWrite => {}

        CommandType::AutoPointerRead
        | CommandType::AutoPointerReadWrite
        | CommandType::FixedPointerRead
        | CommandType::FixedPointerReadWrite
        | CommandType::BroadcastRead
        | CommandType::BroadcastReadWrite
        | CommandType::LogicalRead
        | CommandType::LogicalReadWrite
        | CommandType::AutoReadMultipleWrite
        | CommandType::FixedReadMultipleWrite => data
            .iter_mut()
            .zip(port.rx_buf.lock().unwrap()[usize::from(index)][ETHERCAT_HEADER_SIZE..].iter())
            .for_each(|(dest, src)| *dest = *src),
    }

    // Clear buffer status
    set_buf_stat(port, index.into(), BufferState::Empty);

    Ok(wkc)
}

/// Broadcast write primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, normally 0
/// - `address_offset`: Address offset, slave memory address
/// - `data`: databuffer to be written to slaves
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// WorkCounter or error
pub fn bwr(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        address_position,
        address_offset as u16,
        data,
        timeout,
        CommandType::BroadcastWrite,
    )
}

/// Broadcast read primitive (blocking)
///
/// # Parameters
/// - `port`: port context struct
/// - `address_position`: Address position, normally 0
/// - `address_offset`: Address offset, slave memory address
/// - `length`: length of databuffer
/// - `data`: databuffer to put slave data in
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns workcounter or error
pub fn brd(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        address_position,
        address_offset as u16,
        data,
        timeout,
        CommandType::BroadcastRead,
    )
}

/// Auto increment address read multiple write primitive (blocking)
///
/// # Parameter
/// `port`: Port context struct
/// `address_position`: Address position, each slave increments, slave that has 0 reads,
///                     following slaves write.
/// `address_offset`: Address offset, slave memory address
/// `data`: Databuffer to put slave data in
/// `timeout`: Timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// Workcounter or error
pub fn aprd(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        CommandType::AutoPointerRead,
    )
}

/// Auto increment read multiple write primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, each slave increments, slave that has 0 reads,
///                       following slaves write.
/// - `address_offset`: Address offset, slave memory address
/// - `adta`: Databuffer to put slave data in
/// - `timeout`: Timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// Workcounter or error
pub fn armw(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        CommandType::AutoReadMultipleWrite,
    )
}

/// Configured address read multiple write primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, slave with that address reads,
///                       following slaves write.
/// - `address_offset`: Address Offset, slave memory address
/// - `data`: Databuffer to put slave data in
/// - `timeout`: Timeout duration, standard is TIMEOUT_RETURN
///
/// # Returns
/// Workcounter or error
pub fn frmw(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        CommandType::FixedReadMultipleWrite,
    )
}

/// Auto increment read word return primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, each slave increments, slave that has 0 reads,
/// - `address_offset`: Address offset, slave memory address
/// - `timeout`: Timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// Word data from slave or error
pub fn aprdw(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    timeout: Duration,
) -> Result<Ethercat<u16>, NicdrvError> {
    let mut word = [0; 2];
    aprd(
        port,
        address_position,
        address_offset as u16,
        &mut word,
        timeout,
    )?;
    Ok(Ethercat::from_raw(u16::from_ne_bytes(word)))
}

/// Configured address read primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, slave with that address reads
/// - `address_offset`: Addddress offset, slave memory address
/// - `data`: Databuffer to put slave data in
/// - `timeout`: Timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// Workcounter or error
pub fn fprd(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        address_position,
        address_offset as u16,
        data,
        timeout,
        CommandType::FixedPointerRead,
    )
}

/// Configured address read, word return primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, slave with that address reads
/// - `address_offset`: Address offset, slave memory address
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// Word data from slave
pub fn fprdw(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    timeout: Duration,
) -> Result<Ethercat<u16>, NicdrvError> {
    let mut word = [0; 2];
    fprd(port, address_position, address_offset, &mut word, timeout)?;
    Ok(Ethercat::from_raw(u16::from_ne_bytes(word)))
}

/// Configured address read, long return primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, slave with that address reads
/// - `address_offset`: Address offset, slave memory address
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// Long data from slave
pub fn fprdl(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    timeout: Duration,
) -> Result<Ethercat<u32>, NicdrvError> {
    let mut word = [0; 4];
    fprd(port, address_position, address_offset, &mut word, timeout)?;
    Ok(Ethercat::from_raw(u32::from_ne_bytes(word)))
}

/// Configured address read, long long return primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, slave with that address reads
/// - `address_offset`: Address offset, slave memory address
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// Long long data from slave
pub fn fprdll(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    timeout: Duration,
) -> Result<Ethercat<u64>, NicdrvError> {
    let mut word = [0; 8];
    fprd(port, address_position, address_offset, &mut word, timeout)?;
    Ok(Ethercat::from_raw(u64::from_ne_bytes(word)))
}

/// Auto increment address write, primitive (blocking)
///
/// # Parameters
/// - `port`: port context struct
/// - `address_position: Address position, each slave increments, slave that has 0 writes.
/// - `address_offset`: Address offset, slave memory address
/// - `data`: Databuffer to write to slave
/// - `timeout`: TImeout duration, standard is TIMEOUT_RETURN
///
/// # Returns
/// Workcounter or error
pub fn apwr(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        CommandType::AutoPointerWrite,
    )
}

/// Auto increment address write, word primitive (blocking).
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position: Address position, each slave increments, slave that has 0 writes.
/// - `address_offset`: Address offset, slave memory address
/// - `data`: Word data to write to slave
/// - `timeout`: Timeout duration, standard is TIMEOUT_RETURN
///
/// # Returns
/// Workcounter or error
pub fn apwrw(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    data: Ethercat<u16>,
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    apwr(
        port,
        address_position,
        address_offset as u16,
        &mut data.to_bytes(),
        timeout,
    )
}

/// Configured address write, primitive (blocking)
///
/// # Parameters
/// - `port`: port context struct
/// - `address_position`: Address position, slave with that address writes
/// - `address_offset`: Address offset, slave memory address
/// - `data`: Databuffer to write to slave
///
/// # Returns
/// Workcounter or error
pub fn fpwr(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        CommandType::FixedPointerWrite,
    )
}

/// Configured address write, word primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, slave with that address writes
/// - `address_offsrt`: Address offset, slave memory address
/// - `data`: Word to write to slave
/// - `timeout`: Timeout duration, standard is TIMEOUT_RETURN
///
/// # Returns
/// Workcounter or error
pub fn fpwrw(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    data: Ethercat<u16>,
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    fpwr(
        port,
        address_position,
        address_offset as u16,
        &mut data.to_bytes(),
        timeout,
    )
}

/// Logical memory read/write primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `logical_address`: Logical memory address
/// - `data`: Databuffer to write to and read from slave
/// - `timeout`: Timeout duration, standard is TIMEOUT_RETURN
///
/// # Returns
/// Workcounter or error
pub fn lrw(
    port: &mut Port,
    logical_address: u32,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        low_word(logical_address),
        high_word(logical_address),
        data,
        timeout,
        CommandType::LogicalReadWrite,
    )
}

/// Logical memory read primitive (blocking)
///
/// # Parameters
/// - `port`: port context struct
/// - `logical_address`: Logical memory address
/// - `data`: Databuffer to read from slave
/// - `timeout`: Timeout duration, standard is TIMEOUT_RETURN
///
/// # Returns
/// Workcounter or error
pub fn lrd(
    port: &mut Port,
    logical_address: u32,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        low_word(logical_address),
        high_word(logical_address),
        data,
        timeout,
        CommandType::LogicalRead,
    )
}

/// Logical memory write primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `logical_address`: Logical memory address
/// - `data`: Databuffer to write to slave
/// - `timeout`: Timeout duration, standard is TIMEOUT_RETURN
///
/// # Returns
/// Workcounter or error
pub fn lwr(
    port: &mut Port,
    logical_address: u32,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_command(
        port,
        low_word(logical_address),
        high_word(logical_address),
        data,
        timeout,
        CommandType::LogicalWrite,
    )
}

/// Logical memory read/write primitive + clock distribution (blocking)
/// Frame consists of 2 datagrams, 1 logical read write and 1 FixedPointerMultiWrite.
///
/// # Parameters
/// - `port`: Port context struct
/// - `logical_address`: Logical memory address
/// - `data`: Databuffer to write to and read from slave
/// - `distributed_clock_reference_slave`: Distributed clock reference slave address
/// - `distributed_clock_time`: Distributed clock time read from reference slave
/// - `timeout`: Timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Returns
/// Workcounter or error
pub fn lrwdc(
    port: &mut Port,
    logical_address: u32,
    data: &mut [u8],
    distributed_clock_reference_slave: u16,
    distributed_clock_time: &mut [i64],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    let index = get_index(port);
    let distributed_clock_offset = {
        let tx_buffer = &mut port.tx_buffers.lock().unwrap()[usize::from(index)];

        // Logical read write in first datagram
        setup_datagram(
            tx_buffer,
            CommandType::LogicalReadWrite,
            index,
            low_word(logical_address),
            high_word(logical_address),
            data,
        );

        // FPRMW in second datagram
        let distributed_clock_to_ethercat = host_to_ethercat(distributed_clock_time[0]);
        add_datagram(
            tx_buffer,
            CommandType::FixedReadMultipleWrite,
            index,
            false,
            distributed_clock_reference_slave,
            EthercatRegister::DistributedClockSystemTime.into(),
            &mut distributed_clock_to_ethercat.to_bytes(),
        )
    };

    let mut wkc = src_confirm(port, index, timeout).unwrap();
    {
        let rx_buffer = &mut port.rx_buf.lock().unwrap()[usize::from(index)];
        if rx_buffer[ETHERCAT_COMMAND_OFFET] == CommandType::LogicalReadWrite.into() {
            let mut response_iter = rx_buffer[ETHERCAT_HEADER_SIZE..].iter();
            data.iter_mut()
                .zip(response_iter.by_ref())
                .for_each(|(dest, src)| *dest = *src);
            wkc = u16::from_ne_bytes(response_iter.by_ref().take(2).enumerate().fold(
                [0; 2],
                |mut result, (index, value)| {
                    result[index] = *value;
                    result
                },
            ));
            distributed_clock_time[0] = i64::from_ne_bytes(
                rx_buffer[distributed_clock_offset..]
                    .iter()
                    .take(2)
                    .enumerate()
                    .fold([0; 8], |mut result, (index, value)| {
                        result[index] = *value;
                        result
                    }),
            );
        }
    }
    set_buf_stat(port, usize::from(index), BufferState::Empty);

    Ok(wkc)
}
