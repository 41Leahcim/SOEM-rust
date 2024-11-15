//! Setting up a datagram in an ethernet frame.
//! EtherCAT datagram primitives, broadcast, auto increment, configured and
//! logical adddressed data transfers. All base transfers are blocking, so
//! wait for the frame to be returned to the master or timeout. If this is
//! not acceptable, build your own datagrams and use the functions from nicdrv.rs

use std::{io, time::Duration};

use crate::{
    ethercat::r#type::{DATAGRAM_FOLLOWS, ETHERCAT_LENGTH_SIZE},
    oshw::nicdrv::{NicdrvError, Port},
};

use super::{
    r#type::{
        high_word, low_word, Buffer, BufferState, Command, Ethercat, EthercatHeader,
        EthercatRegister, EthernetHeader, ReadCommand, WriteCommand, ECATTYPE,
        ETHERCAT_COMMAND_OFFET, ETHERCAT_WORK_COUNTER_SIZE,
    },
    ReadFrom, WriteTo,
};

/// Write data to EtherCAT datagram
///
/// # Parameters
/// - `datagramdata`: data part of datagram
/// - `command`: Command being executed
/// - `length`: Length of databuffer
/// - `data`: Databuffer to be copied into datagram
fn write_datagram_data(datagram_data: &mut [u8], command: Command, data: &[u8]) {
    if !data.is_empty() {
        match command {
            Command::ReadCommand(
                ReadCommand::Nop
                | ReadCommand::AutoPointerRead
                | ReadCommand::FixedPointerRead
                | ReadCommand::BroadcastRead
                | ReadCommand::LogicalRead,
            ) => {
                // No data to write, initialize data so frame is in a known state
                datagram_data.fill(0);
            }
            _ => datagram_data.copy_from_slice(data),
        }
    }
}

/// Generate and set EtherCAT datagram in a standard Ethernet frame.
///
/// # Parameters
/// - `port`: Port context struct
/// - `frame`: framebuffer
/// - `command`: Command to execute
/// - `index`: Index used for TX and RX buffers
/// - `address_position`: Address position
/// - `address_offset`: Address offset
/// - `length`: Length of datagram excluding EtherCAT header
/// - `data`: Databuffer to be compied in datagram
///
/// # Errors
/// Returns an error if the Ethercat header couldn't be converted to bytes.
///
/// # Panics
/// Will panic if:
/// - Data is too large to fit in buffer
pub fn setup_datagram(
    frame: &mut Buffer,
    command: Command,
    index: u8,
    address_position: u16,
    address_offset: u16,
    data: &[u8],
) -> io::Result<()> {
    // Ethernet header is preset and fixed in frame buffers.
    // EtherCAT header needs to be added after that.
    frame.resize(EthernetHeader::size(), 0).unwrap();
    frame
        .extend_from_slice(
            &EthercatHeader::new(
                Ethercat::from_host(
                    (usize::from(ECATTYPE) + EthercatHeader::size() + data.len()) as u16,
                ),
                command,
                index,
                Ethercat::from_host(address_position),
                Ethercat::from_host(address_offset),
                Ethercat::from_host(data.len() as u16),
                Ethercat::default(),
            )
            .bytes(),
        )
        .unwrap();
    frame.extend_from_slice(data).unwrap();

    // Set worker count to 0
    frame.extend_from_slice(&[0; 2]).unwrap();
    Ok(())
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
/// # Panics
/// Will panic if the datagram doesn't fit in the frame.
///
/// # Returns
/// Offset to data in rx frame, usefull to retrieve data after RX
pub fn add_datagram(
    frame: &mut Buffer,
    command: Command,
    index: u8,
    more: bool,
    address_position: u16,
    address_offset: u16,
    data: &[u8],
) -> usize {
    // Copy previous frame size
    let previous_length = frame.len();

    // Set size of frame in buffer array
    frame
        .resize(
            previous_length + EthercatHeader::size() - ETHERCAT_LENGTH_SIZE
                + ETHERCAT_WORK_COUNTER_SIZE
                + data.len(),
            0,
        )
        .unwrap();

    let mut datagram = EthercatHeader::read_from(&mut &frame[EthernetHeader::size()..]).unwrap();

    // Add new datargam to ethernet frame size
    *datagram.ethercat_length_mut() = Ethercat::from_host(
        (usize::from(datagram.ethercat_length().to_host()) + EthercatHeader::size() + data.len())
            as u16,
    );
    *datagram.data_length_mut() =
        Ethercat::from_host(datagram.data_length().to_host() | DATAGRAM_FOLLOWS);
    datagram
        .write_to(&mut &mut frame[EthernetHeader::size()..])
        .unwrap();

    // Set new EtherCAT header position
    datagram =
        EthercatHeader::read_from(&mut &frame[previous_length - ETHERCAT_LENGTH_SIZE..]).unwrap();
    *datagram.command_mut() = command;
    *datagram.index_mut() = index;
    *datagram.address_position_mut() = Ethercat::from_host(address_position);
    *datagram.address_offset_mut() = Ethercat::from_host(address_offset);
    *datagram.data_length_mut() = Ethercat::from_host(if more {
        // This is not the last datagram to add
        data.len() as u16 | DATAGRAM_FOLLOWS
    } else {
        // This is the last datagram to add
        data.len() as u16
    });
    datagram
        .write_to(&mut &mut frame[previous_length - ETHERCAT_LENGTH_SIZE..])
        .unwrap();

    write_datagram_data(
        &mut frame[previous_length + EthercatHeader::size() - ETHERCAT_LENGTH_SIZE..],
        command,
        data,
    );

    // Set Work Counter to 0
    frame[previous_length + EthercatHeader::size() - ETHERCAT_LENGTH_SIZE + data.len()..].fill(0);

    // Return offset to data in rx frame, 14 bytes smaller than tx frame due to stripping of
    // thernet header.
    previous_length + EthercatHeader::size() - ETHERCAT_LENGTH_SIZE - EthercatHeader::size()
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
fn execute_primitive_read_command(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
    timeout: Duration,
    command: ReadCommand,
) -> Result<u16, NicdrvError> {
    // Get fresh index
    let index = port.get_index();

    // Setup datagram
    setup_datagram(
        &mut port.stack_mut().tx_buffers_mut()[usize::from(index)],
        Command::ReadCommand(command),
        index,
        address_position,
        address_offset,
        data,
    )?;

    // Send data and wait for answer
    let wkc = port.src_confirm(index, timeout);

    if wkc.is_ok() {
        data.copy_from_slice(
            &port.stack().rx_buffers()[usize::from(index)].data()[EthercatHeader::size()..],
        );
    }

    // Clear buffer status
    port.set_buf_stat(index.into(), BufferState::Empty);

    wkc
}

fn execute_primitive_write_command(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &[u8],
    timeout: Duration,
    command: WriteCommand,
) -> Result<u16, NicdrvError> {
    // Get fresh index
    let index = port.get_index();

    // Setup datagram
    setup_datagram(
        &mut port.stack_mut().tx_buffers_mut()[usize::from(index)],
        Command::WriteCommand(command),
        index,
        address_position,
        address_offset,
        data,
    )?;

    // Send data and wait for answer
    let wkc = port.src_confirm(index, timeout);

    // Clear buffer status
    port.set_buf_stat(index.into(), BufferState::Empty);

    wkc
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
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
///
/// # Returns
/// WorkCounter or error
pub fn bwr(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    data: &[u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_write_command(
        port,
        address_position,
        address_offset as u16,
        data,
        timeout,
        WriteCommand::BroadcastWrite,
    )
}

/// Broadcast read primitive (blocking)
///
/// # Parameters
/// - `port`: port context struct
/// - `address_position`: Address position, normally 0
/// - `address_offset`: Address offset, slave memory address
/// - `data`: databuffer to put slave data in
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Errors
/// Will return an error when it fails to send the request or didn't receive a reply.
///
/// # Returns
/// workcounter or error
pub fn brd(
    port: &mut Port,
    address_position: u16,
    address_offset: EthercatRegister,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_read_command(
        port,
        address_position,
        address_offset as u16,
        data,
        timeout,
        ReadCommand::BroadcastRead,
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
/// # Errors
/// Will return an error when it fails to send the request or didn't receive a reply.
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
    execute_primitive_read_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        ReadCommand::AutoPointerRead,
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
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
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
    execute_primitive_read_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        ReadCommand::AutoReadMultipleWrite,
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
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
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
    execute_primitive_read_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        ReadCommand::FixedReadMultipleWrite,
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
/// # Errors
/// Will return an error when it fails to send the request or didn't receive a reply.
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
    Ok(Ethercat::<u16>::from_bytes(word))
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
/// # Errors
/// Will return an error when it fails to send the request or didn't receive a reply.
///
/// # Returns
/// Workcounter or error
pub fn fprd(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_read_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        ReadCommand::FixedPointerRead,
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
/// # Errors
/// Will return an error when it fails to send the request or didn't receive a reply.
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
    fprd(
        port,
        address_position,
        u16::from(address_offset),
        &mut word,
        timeout,
    )?;
    Ok(Ethercat::<u16>::from_bytes(word))
}

/// Configured address read, long return primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, slave with that address reads
/// - `address_offset`: Address offset, slave memory address
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Errors
/// Will return an error when it fails to send the request or didn't receive a reply.
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
    fprd(
        port,
        address_position,
        address_offset.into(),
        &mut word,
        timeout,
    )?;
    Ok(Ethercat::<u32>::from_bytes(word))
}

/// Configured address read, long long return primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `address_position`: Address position, slave with that address reads
/// - `address_offset`: Address offset, slave memory address
/// - `timeout`: timeout duration, standard is `TIMEOUT_RETURN`
///
/// # Errors
/// Will return an error when it fails to send the request or didn't receive a reply.
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
    fprd(
        port,
        address_position,
        address_offset.into(),
        &mut word,
        timeout,
    )?;
    Ok(Ethercat::<u64>::from_bytes(word))
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
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
///
/// # Returns
/// Workcounter or error
pub fn apwr(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &[u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_write_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        WriteCommand::AutoPointerWrite,
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
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
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
        &data.to_bytes(),
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
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
///
/// # Returns
/// Workcounter or error
pub fn fpwr(
    port: &mut Port,
    address_position: u16,
    address_offset: u16,
    data: &[u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_write_command(
        port,
        address_position,
        address_offset,
        data,
        timeout,
        WriteCommand::FixedPointerWrite,
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
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
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
        &data.to_bytes(),
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
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
///
/// # Returns
/// Workcounter or error
pub fn lrw(
    port: &mut Port,
    logical_address: u32,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    let index = port.get_index();
    setup_datagram(
        &mut port.stack_mut().tx_buffers_mut()[usize::from(index)],
        Command::ReadCommand(ReadCommand::LogicalReadWrite),
        index,
        low_word(logical_address),
        high_word(logical_address),
        data,
    )?;
    let wkc = port.src_confirm(index, timeout);
    if wkc.is_ok()
        && port.stack().rx_buffers()[usize::from(index)].data()[ETHERCAT_COMMAND_OFFET]
            == u8::from(ReadCommand::LogicalReadWrite)
    {
        data.copy_from_slice(
            &port.stack().rx_buffers()[usize::from(index)].data()[EthercatHeader::size()..],
        );
    }
    port.set_buf_stat(usize::from(index), BufferState::Empty);
    wkc
}

/// Logical memory read primitive (blocking)
///
/// # Parameters
/// - `port`: port context struct
/// - `logical_address`: Logical memory address
/// - `data`: Databuffer to read from slave
/// - `timeout`: Timeout duration, standard is TIMEOUT_RETURN
///
/// # Errors
/// Will return an error when it fails to send the request or didn't receive a reply.
///
/// # Returns
/// Workcounter or error
pub fn lrd(
    port: &mut Port,
    logical_address: u32,
    data: &mut [u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    let index = port.get_index();
    setup_datagram(
        &mut port.stack_mut().tx_buffers_mut()[usize::from(index)],
        Command::ReadCommand(ReadCommand::LogicalRead),
        index,
        low_word(logical_address),
        high_word(logical_address),
        data,
    )?;
    let wkc = port.src_confirm(index, timeout);
    if wkc.is_ok()
        && port.stack().rx_buffers()[usize::from(index)].data()[ETHERCAT_COMMAND_OFFET]
            == u8::from(ReadCommand::LogicalRead)
    {
        data.copy_from_slice(
            &port.stack().rx_buffers()[usize::from(index)].data()[EthercatHeader::size()..],
        );
    }
    port.set_buf_stat(usize::from(index), BufferState::Empty);
    wkc
}

/// Logical memory write primitive (blocking)
///
/// # Parameters
/// - `port`: Port context struct
/// - `logical_address`: Logical memory address
/// - `data`: Databuffer to write to slave
/// - `timeout`: Timeout duration, standard is TIMEOUT_RETURN
///
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
///
/// # Returns
/// Workcounter or error
pub fn lwr(
    port: &mut Port,
    logical_address: u32,
    data: &[u8],
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    execute_primitive_write_command(
        port,
        low_word(logical_address),
        high_word(logical_address),
        data,
        timeout,
        WriteCommand::LogicalWrite,
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
/// # Panics
/// Will panic if the received data isn't large enough
///
/// # Errors
/// Will return an error when it fails to send the data or didn't receive a reply.
///
/// # Returns
/// Workcounter or error
pub fn lrwdc(
    port: &mut Port,
    logical_address: u32,
    data: &mut [u8],
    distributed_clock_reference_slave: u16,
    distributed_clock_time: &mut i64,
    timeout: Duration,
) -> Result<u16, NicdrvError> {
    let index = port.get_index();
    let distributed_clock_offset = {
        let tx_buffer = &mut port.stack_mut().tx_buffers_mut()[usize::from(index)];

        // Logical read write in first datagram
        setup_datagram(
            tx_buffer,
            ReadCommand::LogicalReadWrite.into(),
            index,
            low_word(logical_address),
            high_word(logical_address),
            data,
        )?;

        // FPRMW in second datagram
        let distributed_clock_to_ethercat = Ethercat::from_host(*distributed_clock_time);
        add_datagram(
            tx_buffer,
            ReadCommand::FixedReadMultipleWrite.into(),
            index,
            false,
            distributed_clock_reference_slave,
            EthercatRegister::DistributedClockSystemTime.into(),
            &distributed_clock_to_ethercat.to_bytes(),
        )
    };

    let mut wkc = port.src_confirm(index, timeout);
    {
        let rx_buffer = &mut port.stack().rx_buffers()[usize::from(index)].data();
        if wkc.is_ok() && rx_buffer[ETHERCAT_COMMAND_OFFET] == ReadCommand::LogicalReadWrite.into()
        {
            let mut response = &rx_buffer[EthercatHeader::size()..];
            data.copy_from_slice(response);
            response = &response[data.len()..];
            wkc = Ok(u16::from_ne_bytes(response[..2].try_into().unwrap()));
            *distributed_clock_time = i64::from_ne_bytes(
                rx_buffer[distributed_clock_offset..distributed_clock_offset + 2]
                    .try_into()
                    .unwrap(),
            );
        }
    }
    port.set_buf_stat(usize::from(index), BufferState::Empty);

    wkc
}
