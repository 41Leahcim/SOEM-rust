//! EtherCAT RAW socket driver
//!
//! Low level interface functions to send and receive EtherCAT packets.
//! EtherCAT has the property that packets are only send by the master,
//! and the send packets always return in the receive buffer.
//! There can be multiple packets "on the wire" before they return.
//! To combine the received packets with the original send packets, a buffer
//! system is installed. The identifier is put in the index item of the
//! EtherCAT header. The index is stored and compared when a frame is received.
//! If there is a match, the packet can be combined with the transmit packet
//! and returned to the higher level function.
//!
//! The socket layer can exhibit a reversal in the packet orde (rare).
//! If the TX order is A-B-C, the return order could be A-C-B. The indexed buffer
//! will reorder the packets automaticaly.
//!
//! The "redundant" option will configure 2 sockets and 2 NIC interfaces.
//! Slaves are connected to both interfaces, 1 on the IN port and 1 on the OUT port.
//! Packets are send via both interfaces. Any one of the connections (also an
//! interconect) can be removed and the slaves are still serviced with packets.
//! The software layer will detect the possible failure modes and compensatte.
//! If needed, the packets from interface A are resent through interface B.
//! This layer is fully transparent from the highest layers.

use std::{array, io, mem::zeroed, ptr, time::Duration};

use crate::{
    ethercat::{
        r#type::{
            Buffer, BufferState, EthercatHeader, EthercatHeaderError, EthernetHeader, BUFFER_SIZE,
            ETH_P_ECAT, MAX_BUFFER_COUNT, TIMEOUT_RETURN,
        },
        ReadFrom,
    },
    osal::OsalTimer,
    safe_c::{CError, ReceiveError, ReceiveFlags, SendError, SendFlags, Socket},
};

use super::Network;

#[derive(Debug)]
pub enum NicdrvError {
    C(CError),
    EthercatHeaderError(EthercatHeaderError),
    Io(io::Error),
    MissingRedportInSecondaryNicSetup,
    NoFrame,
    OtherFrame,
}

impl From<CError> for NicdrvError {
    fn from(value: CError) -> Self {
        Self::C(value)
    }
}

impl From<EthercatHeaderError> for NicdrvError {
    fn from(value: EthercatHeaderError) -> Self {
        Self::EthercatHeaderError(value)
    }
}

impl From<io::Error> for NicdrvError {
    fn from(value: io::Error) -> Self {
        Self::Io(value)
    }
}

/// Primary MAC address used for EtherCAT.
/// THis address is not the MAC address used from the NIC.
/// EtherCAT doesn't care about MAC addressing, but it is used here to
/// differentiate the route the packet traverses through the EtherCAT
/// segment. This is needed to find out the packet flow in redundant
/// configurations.
pub const PRIMARY_MAC: [u16; 3] = [0x101, 0x101, 0x101];

/// Secondary MAC address used for EtherCAT
pub const SECONDARY_MAC: [u16; 3] = [0x404, 0x404, 0x404];

/// Second MAC word is used for identification
const RX_PRIMARY: u16 = PRIMARY_MAC[1];
const RX_SECONDARY: u16 = SECONDARY_MAC[1];

pub struct RxBuffer {
    data: Buffer,
    state: BufferState,
}

impl Default for RxBuffer {
    fn default() -> Self {
        Self {
            data: Buffer::default(),
            state: BufferState::Empty,
        }
    }
}

impl RxBuffer {
    pub const fn data(&self) -> &Buffer {
        &self.data
    }

    pub fn data_mut(&mut self) -> &mut Buffer {
        &mut self.data
    }

    pub fn set_state(&mut self, state: BufferState) {
        self.state = state;
    }

    pub const fn state(&self) -> BufferState {
        self.state
    }
}

/// EtherCAT Pointer structure to Tx and Rx stacks
pub struct Stack {
    /// Send buffers
    tx_buffer: [Buffer; MAX_BUFFER_COUNT],

    /// Temporary send buffer
    temp_tx_buf: usize,

    /// Receive buffers
    rx_buffers: [RxBuffer; MAX_BUFFER_COUNT],
    temp_rx_buf: usize,
}

impl Stack {
    pub const fn tx_buffers(&self) -> &[Buffer; MAX_BUFFER_COUNT] {
        &self.tx_buffer
    }

    pub const fn temp_tx_buf(&self) -> &Buffer {
        &self.tx_buffers()[self.temp_tx_buf]
    }

    pub fn tx_buffers_mut(&mut self) -> &mut [Buffer; MAX_BUFFER_COUNT] {
        &mut self.tx_buffer
    }

    pub fn temp_tx_buf_mut(&mut self) -> &mut Buffer {
        let index = self.temp_tx_buf;
        &mut self.tx_buffers_mut()[index]
    }

    pub const fn rx_buffers(&self) -> &[RxBuffer; MAX_BUFFER_COUNT] {
        &self.rx_buffers
    }

    pub const fn temp_rx_buf(&self) -> &RxBuffer {
        &self.rx_buffers[self.temp_rx_buf]
    }

    pub fn rx_buffers_mut(&mut self) -> &mut [RxBuffer; MAX_BUFFER_COUNT] {
        &mut self.rx_buffers
    }

    pub fn temp_rx_buf_mut(&mut self) -> &mut RxBuffer {
        &mut self.rx_buffers[self.temp_rx_buf]
    }
}

impl Default for Stack {
    fn default() -> Self {
        Self {
            tx_buffer: [const { heapless::Vec::new() }; MAX_BUFFER_COUNT],
            temp_tx_buf: 0,
            rx_buffers: array::from_fn(|_| RxBuffer::default()),
            temp_rx_buf: 0,
        }
    }
}

/// EtherCAT eXtended Pointer structure to buffers for redundant port
pub struct RedPort {
    stack: Stack,
    sockhandle: Socket,

    /// Receive MAC source address
    rx_source_address: [i32; MAX_BUFFER_COUNT],
}

impl RedPort {
    /// # Errors
    /// Returns an error on failure to initialize a new socket.
    pub fn new(interface_name: &str) -> Result<Self, CError> {
        Ok(Self {
            stack: Stack::default(),
            sockhandle: initialize_socket(interface_name)?,
            rx_source_address: [0; MAX_BUFFER_COUNT],
        })
    }
}

/// Ethercat eXtended pointer structure to buffers, variables, and mutexes for port instantiation
pub struct Port {
    stack: Stack,
    sockhandle: Socket,

    /// Receive MAC source address
    rx_source_address: [i32; MAX_BUFFER_COUNT],

    /// last used frame index
    last_index: usize,

    redport: Option<RedPort>,
}

#[derive(Debug, Clone, Copy)]
pub enum RedundancyMode<'red> {
    None,
    Redundant(&'red str),
}

impl Port {
    pub const fn stack(&self) -> &Stack {
        &self.stack
    }

    pub fn stack_mut(&mut self) -> &mut Stack {
        &mut self.stack
    }

    pub fn set_red_port(&mut self, red_port: RedPort) {
        self.redport = Some(red_port);
    }

    /// Basic setup connect NIC to socket.
    ///
    /// # Parameters
    /// - `self`: port context struct
    /// - `interface_name`: Name of NIC device, f.e. "eth0"
    /// - `redundant`: if true, add redundant port
    ///
    /// # Errors
    /// Returns an error if the socket couldn't be initialized.
    #[expect(
        clippy::missing_panics_doc,
        reason = "EthernetHeader is at compile time checked to be smaller than the buffer size."
    )]
    pub fn setup_nic(
        interface_name: &str,
        redundancy_mode: RedundancyMode,
    ) -> Result<Self, NicdrvError> {
        const _: () = assert!(EthernetHeader::size() < BUFFER_SIZE);
        let redport = if let RedundancyMode::Redundant(interface_name) = redundancy_mode {
            Some(RedPort::new(interface_name)?)
        } else {
            None
        };
        let mut result = Self {
            stack: Stack::default(),
            sockhandle: initialize_socket(interface_name)?,
            rx_source_address: [0; MAX_BUFFER_COUNT],
            last_index: 0,
            redport,
        };

        let header = EthernetHeader::new(PRIMARY_MAC);
        for tx_buf in result.stack_mut().tx_buffers_mut().iter_mut() {
            tx_buf.extend_from_slice(&header.bytes()).unwrap();
        }
        Ok(result)
    }

    /// Set rx buffer status
    ///
    /// # Parameters
    /// - `self`: Port context struct
    /// - `index`: index in buffer array
    /// - `bufstat`: status to set
    pub fn set_buf_stat(&mut self, index: usize, bufstat: BufferState) {
        self.stack.rx_buffers[index].state = bufstat;
        if let Some(redport) = self.redport.as_mut() {
            redport.stack.rx_buffers[index].state = bufstat;
        }
    }

    /// Get new frame identifier index and allocate corresponding rx buffer
    ///
    /// # Parameters
    /// - `self`: port context struct
    ///
    /// # Returns
    /// New index
    pub fn get_index(&mut self) -> u8 {
        let mut index = self.last_index + 1;

        if index >= MAX_BUFFER_COUNT {
            index = 0;
        }
        let rx_buf = &mut self.stack.rx_buffers;
        let index = (index..)
            .take(MAX_BUFFER_COUNT)
            .zip(rx_buf.iter().skip(index))
            .find(|&(_, rx_buf)| rx_buf.state == BufferState::Empty)
            .map_or(index, |(index, _)| index);
        rx_buf[index].state = BufferState::Alloc;
        if let Some(redport) = self.redport.as_mut() {
            redport.stack.rx_buffers[index].state = BufferState::Alloc;
        }

        index as u8
    }

    fn get_stack_mut(&mut self, secondary: bool) -> &mut Stack {
        if let Some(redport) = self.redport.as_mut().filter(|_| secondary) {
            &mut redport.stack
        } else {
            &mut self.stack
        }
    }

    fn get_stack(&self, secondary: bool) -> &Stack {
        if let Some(redport) = self.redport.as_ref().filter(|_| secondary) {
            &redport.stack
        } else {
            &self.stack
        }
    }

    fn get_socket(&self, secondary: bool) -> &Socket {
        if let Some(redport) = self.redport.as_ref().filter(|_| secondary) {
            &redport.sockhandle
        } else {
            &self.sockhandle
        }
    }

    fn rx_source_address_mut(&mut self, secondary: bool) -> &mut [i32; MAX_BUFFER_COUNT] {
        if let Some(redport) = self.redport.as_mut().filter(|_| secondary) {
            &mut redport.rx_source_address
        } else {
            &mut self.rx_source_address
        }
    }

    /// Transmit buffer over socket (non blocking)
    ///
    /// # Parameters
    /// - `self`: port context struct
    /// - `index`: index in tx buffer array
    /// - `secondary`: false=primary, true=secondary (if available)
    ///
    /// # Errors
    /// Returns an error on failure to send the data.
    ///
    /// # Returns
    /// Socket send result
    pub fn out_frame(&mut self, index: usize, secondary: bool) -> Result<usize, SendError> {
        if let Some(redport) = self.redport.as_mut().filter(|_| secondary) {
            redport.stack.rx_buffers[index].state = BufferState::Tx;
            let rval = redport
                .sockhandle
                .send(&redport.stack.tx_buffer[index], SendFlags::new());
            if rval.is_err() {
                redport.stack.rx_buffers[index].state = BufferState::Empty;
            }
            rval
        } else {
            self.stack.rx_buffers[index].state = BufferState::Tx;
            let rval = self
                .sockhandle
                .send(&self.stack.tx_buffer[index], SendFlags::new());
            if rval.is_err() {
                self.stack.rx_buffers[index].state = BufferState::Empty;
            }
            rval
        }
    }

    /// Transmit buffer over socket (non blocking)
    ///
    /// # Parameters
    /// - `self`: port context struct
    /// - `index`: index in tx buffer array
    ///
    /// # Errors
    /// Returns an error if:
    /// - No `EthernetHeader` could be created from the `tx_buffers` at `index`.
    /// - No `EthernetHeader` or `EthercatHeader` could be created from `tmp_buffer`
    ///
    /// # Returns
    /// Socket send result
    pub fn out_frame_red(&mut self, index: u8) -> Result<usize, NicdrvError> {
        let mut ehp =
            EthernetHeader::read_from(&mut self.stack.tx_buffer[usize::from(index)].as_slice())?;

        // Rewrite MAC source address 1 to primary
        ehp.source_address_mut()[1] = Network::from_host(PRIMARY_MAC[1]);
        self.stack.tx_buffer[usize::from(index)]
            .as_mut_slice()
            .copy_from_slice(&ehp.bytes());

        // Transmit over primary socket
        let rval = self.out_frame(index.into(), false);

        if let Some(redport) = self.redport.as_mut() {
            let tmp_buffer = &mut self.stack.tx_buffer[self.stack.temp_tx_buf];
            let mut ehp = EthernetHeader::read_from(&mut tmp_buffer.as_slice())?;

            // Use dummy frame for secondary socket transmit (BRD)
            let mut datagram =
                EthercatHeader::read_from(&mut &tmp_buffer[EthernetHeader::size()..])?;

            // Write index to frame
            *datagram.index_mut() = index;

            // Rewrite MAC source address 1 to secondary
            ehp.source_address_mut()[1] = Network::from_host(SECONDARY_MAC[1]);
            tmp_buffer.copy_from_slice(&ehp.bytes());
            tmp_buffer[EthernetHeader::size()..].copy_from_slice(&datagram.bytes());

            // Transmit over secondary socket
            redport.stack.rx_buffers[usize::from(index)].state = BufferState::Tx;
            if redport
                .sockhandle
                .send(tmp_buffer, SendFlags::new())
                .is_err()
            {
                redport.stack.rx_buffers[usize::from(index)].state = BufferState::Empty;
            }
        }
        rval.map_err(CError::from).map_err(NicdrvError::from)
    }

    /// Non blocking read of socket. Put frame in temporary buffer.
    ///
    /// # Parameters
    /// - `self`: port context struct
    /// - `secondary`: false=primary true=secondary stack (if available)
    ///
    /// # Errors
    /// Returns an erro on failure to receive the data
    pub fn receive_packet(&mut self, secondary: bool) -> Result<bool, ReceiveError> {
        if let Some(redport) = self.redport.as_mut().filter(|_| secondary) {
            let bytes_received = redport.sockhandle.receive(
                &mut redport.stack.temp_rx_buf_mut().data,
                ReceiveFlags::new(),
            )?;
            redport.stack.rx_buffers[self.stack.temp_rx_buf].state = if bytes_received == 0 {
                BufferState::Empty
            } else {
                BufferState::Alloc
            };
            Ok(bytes_received > 0)
        } else {
            let bytes_received = self
                .sockhandle
                .receive(&mut self.stack.temp_rx_buf_mut().data, ReceiveFlags::new())?;
            self.stack.rx_buffers[self.stack.temp_rx_buf].state = if bytes_received == 0 {
                BufferState::Empty
            } else {
                BufferState::Alloc
            };
            Ok(bytes_received > 0)
        }
    }

    /// Non blocking receive frame function. Uses RX buffer and index to combine
    /// read frame with transmitted frame. To compensate for received frames that
    /// are out-of-order, all frames are stored in their respective indexed buffer.
    /// If a frame was placed in the buffer previously, the function retrieves it
    /// from that buffer index without calling `receive_packet`. If the requested index
    /// is not already in the buffer, it calls `receive_packet` to fetch it. There are
    /// 3 options now:
    ///  - 1 no frame read, so exit.
    ///  - 2 frame read but other than requested index, store in buffer and exit
    ///  - 3 frame read with matching index, store in buffer, set completed flag
    ///      in buffer status and exit.
    ///
    /// # Parameters
    /// - `self`: port context struct
    /// - `index`: requested index of frame
    /// - `secondary`: false=primary, true=secondary stack (if available)
    ///
    /// # Panics
    /// Panics if:
    /// - The found index is equal to the temporary buffer index
    ///
    /// # Errors
    /// Returns an error if:
    /// - No EthernetHeader could be read from the temorary buffer
    /// - No EthercatHeader could be read from the temporary buffer
    /// - No frame was send
    ///
    /// # Returns
    /// Workcounter if a frame is found with corresponding index, otherwise error
    pub fn inframe(&mut self, index: u8, secondary: bool) -> Result<u16, NicdrvError> {
        let mut rval = Err(NicdrvError::NoFrame);

        // Check if requested index is already in buffer
        if usize::from(index) < MAX_BUFFER_COUNT
            && self.get_stack(secondary).rx_buffers[usize::from(index)].state != BufferState::Rcvd
        {
            let rxbuf = &self.get_stack(secondary).rx_buffers;
            let rxbuf = rxbuf[usize::from(index)].data.as_slice();
            let l = usize::from(rxbuf[0]) + ((usize::from(rxbuf[1]) & 0xF) << 8);

            // Return WKC
            rval = Ok(u16::from(rxbuf[l]) + (u16::from(rxbuf[l + 1]) << 8));
            self.get_stack_mut(secondary).rx_buffers[usize::from(index)].state =
                BufferState::Complete;
        } else if self.receive_packet(secondary).map_err(CError::from)? {
            rval = Err(NicdrvError::OtherFrame);
            let stack = self.get_stack_mut(secondary);
            let temp_buf = stack.temp_rx_buf;
            let ethernetp =
                EthernetHeader::read_from(&mut stack.rx_buffers[temp_buf].data.as_slice())?;
            if ethernetp.ethernet_type() == Network::from_host(ETH_P_ECAT) {
                let ethercatp = EthercatHeader::read_from(
                    &mut &stack.rx_buffers[temp_buf].data[EthernetHeader::size()..],
                )?;
                let l = usize::from(ethercatp.ethercat_length().to_host() & 0x0FFF);
                let index_found = ethercatp.index();

                // Check whether the index is the index we're looking for
                if index_found == index {
                    // Put it in the buffer
                    let stack = self.get_stack_mut(secondary);
                    let (src, rx_buffer) = if usize::from(index) < temp_buf {
                        let (left, right) = stack.rx_buffers.split_at_mut(temp_buf);
                        (&mut right[0], &mut left[usize::from(index)])
                    } else {
                        let (left, right) = stack.rx_buffers.split_at_mut(usize::from(index));
                        (&mut left[temp_buf], &mut right[0])
                    };
                    rx_buffer.data.clear();
                    rx_buffer
                        .data
                        .extend_from_slice(&src.data[EthernetHeader::size()..])
                        .unwrap();

                    // Return WKC
                    rval = Ok(u16::from(rx_buffer.data[l]) + (u16::from(rx_buffer.data[l + 1])));

                    // Mark as completed
                    stack.rx_buffers[usize::from(index)].state = BufferState::Complete;
                    self.rx_source_address_mut(secondary)[usize::from(index)] =
                        ethernetp.source_address()[1].to_host().into();
                } else if usize::from(index_found) < MAX_BUFFER_COUNT
                    && self.get_stack(secondary).rx_buffers[usize::from(index_found)].state
                        == BufferState::Tx
                {
                    // If the index exists and someone is waiting for it

                    let stack = self.get_stack_mut(secondary);

                    // Put it in the buffer array
                    let (src, rx_buffer) = if usize::from(index_found) < temp_buf {
                        let (left, right) = stack.rx_buffers.split_at_mut(temp_buf);
                        (&mut right[0], &mut left[usize::from(index_found)])
                    } else {
                        let (left, right) = stack.rx_buffers.split_at_mut(usize::from(index_found));
                        (&mut left[temp_buf], &mut right[0])
                    };
                    rx_buffer.data.clear();
                    rx_buffer
                        .data
                        .extend_from_slice(&src.data[EthernetHeader::size()..])
                        .unwrap();

                    // Mark as received
                    stack.rx_buffers[usize::from(index_found)].state = BufferState::Rcvd;
                    self.rx_source_address_mut(secondary)[usize::from(index_found)] =
                        ethernetp.source_address()[1].to_host().into();
                }
            }
        }
        rval
    }

    /// Blocking redundant receive frame function. If redundant mode is not active, then
    /// it skips the secondary stack and redundancy functions. In redundant mode, it waits
    /// for both (primary and secondary) frames to come in. The result goes in a decision
    /// tree that decides, depending on the route of the packet and its possible missing arrive,
    /// how to reroute the original packet to get the data in an other try.
    ///
    /// # Parameters
    /// - `self`: port context struct
    /// - `index`: requested index of frame
    /// - `timer`: Absolute timeout time
    ///
    /// # Errors
    /// Returns an error if no frame could be send/received
    ///
    /// # Returns
    /// Workcounter if a frame is found with corresponding index, otherwise `NicdrvError`
    fn wait_in_frame_red(&mut self, index: u8, timer: &OsalTimer) -> Result<u16, NicdrvError> {
        // If not in redundant mode, always assume secondary is ok
        let mut wkc2 = if self.redport.is_none() {
            Ok(0)
        } else {
            Err(NicdrvError::NoFrame)
        };
        let mut wkc = Err(NicdrvError::NoFrame);

        loop {
            // Only read frame if not already in
            if wkc.is_err() {
                wkc = self.inframe(index, false);
            }

            // Only try secondary if in redundant mode and not already in
            if self.redport.is_some() && wkc2.is_err() {
                wkc2 = self.inframe(index, true);
            }

            // Wait for both frames to arrive or timeout
            if (wkc.is_ok() && wkc2.is_ok()) || timer.is_expired() {
                break;
            }
        }

        // Only do redundant functions when in redundant mode
        if self.redport.is_none() {
            return wkc;
        }

        // primrx if the received MAC source on the primary socket
        let primrx = if wkc.is_ok() {
            self.rx_source_address[usize::from(index)]
        } else {
            0
        };

        // Secrx if the received MAC source on psecondary socket
        let secrx = if let Some(redport) = self.redport.as_ref().filter(|_| wkc2.is_ok()) {
            redport.rx_source_address[usize::from(index)]
        } else {
            0
        };

        // Primary socket got secondary frame and secondary socket got primary frame
        if primrx == RX_SECONDARY.into() && secrx == RX_PRIMARY.into() {
            // Copy secondary buffer to primary
            let received_data = self.redport.as_ref().unwrap().stack.rx_buffers[usize::from(index)]
                .data
                .clone();
            let rxbuf = &mut self.stack_mut().rx_buffers[usize::from(index)].data;
            rxbuf.clear();
            rxbuf.extend_from_slice(&received_data).unwrap();
        }

        // Primary socket got nothing or primary frame, and secondary socket got secondary frame.
        // We need to resend TX packet.
        if (primrx == 0 && secrx == RX_SECONDARY.into())
            || (primrx == RX_PRIMARY.into() && secrx == RX_SECONDARY.into())
        {
            // If both primary and secondary have partial connection, retransmit the primary
            // received frame over the secondary socket. The result from the secondary received
            // frame is a combined frame that traversed all slaves in standard order.
            if primrx == RX_PRIMARY.into() && secrx == RX_SECONDARY.into() {
                let received_data = self.stack.rx_buffers[usize::from(index)].data.clone();
                let tx_buf = &mut self.stack.tx_buffers_mut()[usize::from(index)];
                tx_buf.resize(EthernetHeader::size(), 0).unwrap();
                tx_buf.extend_from_slice(&received_data).unwrap();
            }
            let timer2 = OsalTimer::new(TIMEOUT_RETURN);

            // Resend secondary tx
            self.out_frame(usize::from(index), true)
                .map_err(CError::from)?;

            loop {
                wkc2 = self.inframe(index, true);
                if wkc2.is_ok() || timer2.is_expired() {
                    break;
                }
            }
            if wkc2.is_ok() {
                let rx_buf = &mut self.stack.rx_buffers[usize::from(index)].data;
                rx_buf.clear();
                rx_buf
                    .extend_from_slice(
                        self.redport.as_ref().unwrap().stack.rx_buffers[usize::from(index)]
                            .data()
                            .as_slice(),
                    )
                    .unwrap();
            }
        }

        // Return workcounter or error
        wkc
    }

    /// Blocking receive frame function. Calls `wait_in_frame_red`.
    ///
    /// # Parameters
    /// - `self`: Port context struct
    /// - `index`: Requested index of frame
    /// - `timeout`: Timeout duration
    ///
    /// # Errors
    /// Returns an error if no frame was received
    ///
    /// # Returns
    /// Workcounter if a frame is found with corresponding index, otherwise `NicdrvError`
    pub fn wait_in_frame(&mut self, index: u8, timeout: Duration) -> Result<u16, NicdrvError> {
        let timer = OsalTimer::new(timeout);
        self.wait_in_frame_red(index, &timer)
    }

    /// Blocking send and receive frame function. used for non processdata frames.
    /// A datagram is build into a frame and transmitted via this function. It waits
    /// for and answer and returns the workcounter. The function retries if time is
    /// left and the result is WKC=0 or no frame received.
    ///
    /// The function calls `ec_outframe_red` and `ec_waitinframe_red`.
    ///
    /// # Parameters
    /// - `self`: port context struct
    /// - `index`: index of frame
    /// - `timeout`: timeout duration
    ///
    /// # Errors
    /// Returns an error if:
    /// - The `tx_buffer` at index couldn't be used to create an `EthernetHeader`
    /// - The `tmp_buffer` couldn't be used to create an `EthernetHeader` or `EthercatHeader`
    /// - A frame couldn't be send/received
    ///
    /// # Returns
    /// Workcounter of NicdrvError
    pub fn src_confirm(&mut self, index: u8, timeout: Duration) -> Result<u16, NicdrvError> {
        let timer1 = OsalTimer::new(timeout);
        loop {
            // Tx frame on primary and if in redundant mode a dummy on secondary
            self.out_frame_red(index)?;
            let timer2 = OsalTimer::new(timeout.min(TIMEOUT_RETURN));
            let wkc = self.wait_in_frame_red(index, &timer2);
            if wkc.is_ok() || timer1.is_expired() {
                break wkc;
            }
        }
    }
}

/// Initializes the EtherCAT socket
fn initialize_socket(interface_name: &str) -> Result<Socket, CError> {
    use crate::safe_c::{CBool, SocketOption};
    use libc::{
        ifreq, sockaddr_ll, timeval, AF_PACKET, IFF_BROADCAST, IFF_PROMISC, PF_PACKET,
        SIOCGIFFLAGS, SIOCGIFINDEX, SIOCSIFFLAGS, SOCK_RAW,
    };

    // Use a raw socket with packet type ETH_P_ECAT
    let timeout = timeval {
        tv_sec: 0,
        tv_usec: 1,
    };
    let mut socket = Socket::builder(
        PF_PACKET,
        SOCK_RAW,
        Network::from_host(i32::from(ETH_P_ECAT)),
    )?
    .set_option(SocketOption::ReceiveTimeout(timeout))?
    .set_option(SocketOption::SendTimeout(timeout))?
    .set_option(SocketOption::DoNotRoute(CBool::new(true)))?;

    // Connect socket to NIC by name
    let mut interface_name_bytes = interface_name.bytes().map(|value| value as i8);
    #[expect(unsafe_code)]
    let mut interface = ifreq {
        ifr_name: array::from_fn(|_| interface_name_bytes.next().unwrap_or_default()),
        ifr_ifru: unsafe { zeroed() },
    };
    assert_eq!(interface_name_bytes.next(), None, "Interface name too long");
    let interface_name = interface.ifr_name;

    socket = socket.ioctl(SIOCGIFINDEX, ptr::from_mut(&mut interface).cast::<ifreq>())?;
    #[expect(unsafe_code)]
    let ifindex = unsafe { interface.ifr_ifru.ifru_ifindex };
    interface.ifr_name = interface_name;
    interface.ifr_ifru.ifru_flags = 0;

    // Reset flags of NIC interface
    socket = socket.ioctl(SIOCGIFFLAGS, ptr::from_mut(&mut interface).cast::<ifreq>())?;

    // Set flags of NIC interface, here promiscuous and broadcast
    #[expect(unsafe_code)]
    unsafe {
        interface.ifr_ifru.ifru_flags |= (IFF_PROMISC | IFF_BROADCAST) as i16;
    }
    socket = socket.ioctl(SIOCSIFFLAGS, ptr::from_mut(&mut interface).cast::<ifreq>())?;

    // Bind socket to protocol, in this case RAW EtherCAT
    #[expect(unsafe_code)]
    let sll = sockaddr_ll {
        sll_family: AF_PACKET as u16,
        sll_ifindex: ifindex,
        sll_protocol: Network::from_host(ETH_P_ECAT).into_inner(),
        ..unsafe { zeroed() }
    };

    #[expect(unsafe_code)]
    socket
        .bind(
            unsafe {
                ptr::from_ref(&sll)
                    .cast::<libc::sockaddr>()
                    .as_ref()
                    .unwrap()
            },
            size_of::<sockaddr_ll>() as u32,
        )
        .map_err(CError::from)
}
