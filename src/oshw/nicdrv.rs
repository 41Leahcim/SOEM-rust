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

use std::{
    array,
    mem::zeroed,
    ptr::NonNull,
    sync::{
        atomic::{AtomicI32, Ordering},
        Arc, Mutex,
    },
    time::Duration,
};

use crate::{
    ethercat::{
        eoe::host_to_network,
        r#type::{
            ethercat_to_host, Buffer, BufferState, Error as EthercatError, EthercatHeader,
            EthercatHeaderError, EthernetHeader, EthernetHeaderError, ETHERNET_HEADER_SIZE,
            ETH_P_ECAT, MAX_BUF_COUNT, TIMEOUT_RETURN,
        },
    },
    osal::OsalTimer,
    oshw::{htons, ntohs},
    safe_c::{CError, CloseError},
};

#[derive(Debug)]
pub enum NicdrvError {
    C(CError),
    MissingRedportInSecondaryNicSetup,
    EthernetHeaderError(EthernetHeaderError),
    EthercatHeaderError(EthercatHeaderError),
    EthercatError(EthercatError),
}

impl From<CError> for NicdrvError {
    fn from(value: CError) -> Self {
        Self::C(value)
    }
}

impl From<EthernetHeaderError> for NicdrvError {
    fn from(value: EthernetHeaderError) -> Self {
        Self::EthernetHeaderError(value)
    }
}

impl From<EthercatHeaderError> for NicdrvError {
    fn from(value: EthercatHeaderError) -> Self {
        Self::EthercatHeaderError(value)
    }
}

impl From<EthercatError> for NicdrvError {
    fn from(value: EthercatError) -> Self {
        Self::EthercatError(value)
    }
}

#[derive(Debug, PartialEq, Eq)]
enum RedundancyMode {
    /// No redundancy, single NIC mode
    None,

    /// Double redundant NIC connection
    Double,
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

/// Sets all buffer statuses to empty
fn ecx_clear_rx_buffer_status(rx_buffer_status: &mut [BufferState]) {
    rx_buffer_status
        .iter_mut()
        .for_each(|value| *value = BufferState::Empty);
}

/// EtherCAT Pointer structure to Tx and Rx stacks
pub struct Stack<'stack> {
    /// Socket connection used
    pub socket: Arc<AtomicI32>,

    /// Send buffers
    pub tx_buffer: Arc<Mutex<[Buffer; MAX_BUF_COUNT]>>,

    /// Temporary receive buffer
    pub temp_buf: &'stack Mutex<Buffer>,

    /// Rreceive buffers
    pub rx_buffers: Arc<Mutex<[Buffer; MAX_BUF_COUNT]>>,

    /// Receive buffer status fields
    pub rx_buf_stat: Arc<Mutex<[BufferState; MAX_BUF_COUNT]>>,

    /// Received MAC source address (middle word)
    pub rx_source_address: Arc<Mutex<[i32; MAX_BUF_COUNT]>>,
}

/// EtherCAT eXtended Pointer structure to buffers for redundant port
pub struct RedPort<'redport> {
    pub stack: Stack<'redport>,
    pub sockhandle: Arc<AtomicI32>,

    /// Receive buffers
    pub rxbuf: Arc<Mutex<[Buffer; MAX_BUF_COUNT]>>,

    /// Receive buffer status
    pub rxbufstat: Arc<Mutex<[BufferState; MAX_BUF_COUNT]>>,

    /// Receive MAC source address
    pub rx_source_address: Arc<Mutex<[i32; MAX_BUF_COUNT]>>,

    pub temp_in_buf: &'redport Mutex<Buffer>,
}

/// Ethercat eXtended pointer structure to buffers, variables, and mutexes for port instantiation
pub struct Port<'port> {
    pub stack: Stack<'port>,
    pub sockhandle: Arc<AtomicI32>,

    /// Rx buffers
    pub rx_buf: Arc<Mutex<[Buffer; MAX_BUF_COUNT]>>,

    /// Receive buffer status
    pub rx_buf_stat: Arc<Mutex<[BufferState; MAX_BUF_COUNT]>>,

    /// Receive MAC source address
    pub rx_source_address: Arc<Mutex<[i32; MAX_BUF_COUNT]>>,

    /// Temporary receive buffer
    pub temp_in_buf: &'port Mutex<Buffer>,

    /// Temporary receive buffer status
    pub temp_in_buf_stat: BufferState,

    /// Transmit buffers
    pub tx_buffers: Arc<Mutex<[Buffer; MAX_BUF_COUNT]>>,

    /// Temporary send buffer
    pub temp_tx_buffer: &'port Mutex<Buffer>,

    /// last used frame index
    pub last_index: usize,

    /// Current redundancy state
    redstate: RedundancyMode,

    pub redport: Option<RedPort<'port>>,
}

/// Initializes the EtherCAT socket
fn initialize_socket(interface_name: &str) -> Result<i32, CError> {
    use crate::{
        ioctl,
        safe_c::{setsockopt, socket},
    };
    use libc::{
        bind, ifreq, sockaddr_ll, timeval, AF_PACKET, IFF_BROADCAST, IFF_PROMISC, PF_PACKET,
        SIOCGIFFLAGS, SIOCGIFINDEX, SIOCSIFFLAGS, SOCK_RAW, SOL_SOCKET, SO_DONTROUTE, SO_RCVTIMEO,
        SO_SNDTIMEO,
    };

    // Use a raw socket with packet type ETH_P_ECAT
    let socket = socket(
        PF_PACKET,
        SOCK_RAW,
        host_to_network::<u16>(ETH_P_ECAT).into(),
    )?;

    let mut timeout = timeval {
        tv_sec: 0,
        tv_usec: 1,
    };
    let timeout_ptr = NonNull::new((&mut timeout as *mut timeval).cast()).unwrap();
    setsockopt(
        socket,
        SOL_SOCKET,
        SO_RCVTIMEO,
        timeout_ptr,
        size_of::<timeval>() as u32,
    )?;
    setsockopt(
        socket,
        SOL_SOCKET,
        SO_SNDTIMEO,
        timeout_ptr,
        size_of::<timeval>() as u32,
    )?;
    let mut i = 1;
    setsockopt(
        socket,
        SOL_SOCKET,
        SO_DONTROUTE,
        NonNull::new((&mut i as *mut i32).cast()).unwrap(),
        size_of::<i32>() as u32,
    )?;

    // Connect socket to NIC by name
    let mut interface_name_bytes = interface_name.bytes().map(|value| value as i8);
    let mut interface = ifreq {
        ifr_name: array::from_fn(|_| interface_name_bytes.next().unwrap_or_default()),
        ifr_ifru: unsafe { zeroed() },
    };
    assert_eq!(interface_name_bytes.next(), None, "Interface name too long");
    let interface_name = interface.ifr_name;

    ioctl!(socket, SIOCGIFINDEX, (&mut interface as *mut ifreq))?;
    let ifindex = unsafe { interface.ifr_ifru.ifru_ifindex };
    interface.ifr_name = interface_name;
    interface.ifr_ifru.ifru_flags = 0;

    // Reset flags of NIC interface
    ioctl!(socket, SIOCGIFFLAGS, (&mut interface as *mut ifreq))?;

    // Set flags of NIC interface, here promiscuous and broadcast
    unsafe { interface.ifr_ifru.ifru_flags |= (IFF_PROMISC | IFF_BROADCAST) as i16 };
    ioctl!(socket, SIOCSIFFLAGS, (&mut interface as *mut ifreq))?;

    // Bind socket to protocol, in this case RAW EtherCAT
    let sll = sockaddr_ll {
        sll_family: AF_PACKET as u16,
        sll_ifindex: ifindex,
        sll_protocol: htons(ETH_P_ECAT),
        ..unsafe { zeroed() }
    };
    unsafe {
        bind(
            socket,
            (&sll as *const sockaddr_ll).cast(),
            size_of::<sockaddr_ll>() as u32,
        )
    };
    Ok(socket)
}

/// Basic setup connect NIC to socket.
///
/// # Parameters
/// - `port`: port context struct
/// - `interface_name`: Name of NIC device, f.e. "eth0"
/// - `secondary`: if > 0, use secondary stack instead of primary
pub fn setup_nic(
    port: &mut Port,
    interface_name: &str,
    secondary: bool,
) -> Result<(), NicdrvError> {
    let psock;
    if secondary {
        // Check whether a secondary port is available and initialize it
        if let Some(redport) = &mut port.redport {
            redport.sockhandle.store(-1, Ordering::Relaxed);
            port.redstate = RedundancyMode::Double;
            redport.stack.socket = port.sockhandle.clone();
            redport.stack.temp_buf = port.temp_tx_buffer;
            redport.stack.rx_buffers = redport.rxbuf.clone();
            redport.stack.rx_source_address = redport.rx_source_address.clone();
            ecx_clear_rx_buffer_status(&mut redport.rxbufstat.lock().unwrap()[..1]);
            psock = &redport.sockhandle;
        } else {
            // Fail
            return Err(NicdrvError::MissingRedportInSecondaryNicSetup);
        }
    } else {
        port.sockhandle.store(-1, Ordering::Relaxed);
        port.last_index = 0;
        port.redstate = RedundancyMode::None;
        port.stack.socket = port.sockhandle.clone();
        port.stack.tx_buffer = port.tx_buffers.clone();
        port.stack.temp_buf = port.temp_in_buf;
        port.stack.rx_buffers = port.rx_buf.clone();
        port.stack.rx_buf_stat = port.rx_buf_stat.clone();
        port.stack.rx_source_address = port.rx_source_address.clone();
        ecx_clear_rx_buffer_status(&mut port.rx_buf_stat.lock().unwrap()[..1]);
        psock = &port.sockhandle;
    }
    psock.store(
        initialize_socket(interface_name).unwrap(),
        Ordering::Relaxed,
    );

    let header = EthernetHeader::new(PRIMARY_MAC);
    for (tx_buf, rx_buf_stat) in port
        .tx_buffers
        .lock()
        .unwrap()
        .iter_mut()
        .zip(port.rx_buf_stat.lock().unwrap().iter_mut())
    {
        tx_buf.clear();
        tx_buf.extend_from_slice(header.as_ref()).unwrap();
        *rx_buf_stat = BufferState::Empty;
    }
    let mut temp_tx_buffer = port.temp_tx_buffer.lock().unwrap();
    temp_tx_buffer.clear();
    temp_tx_buffer.extend_from_slice(header.as_ref()).unwrap();
    Ok(())
}

/// Close sockets used
///
/// # Parameters
/// - `port`: port context struct
pub fn close_nic(port: &mut Port) -> Result<(), CloseError> {
    use crate::safe_c::close;
    if port.sockhandle.load(Ordering::Relaxed) >= 0 {
        close(port.sockhandle.load(Ordering::Relaxed))?;
        port.sockhandle.store(0, Ordering::Relaxed);
    }
    if let Some(sockhandle) = port
        .redport
        .as_ref()
        .map(|redport| redport.sockhandle.as_ref())
    {
        close(sockhandle.load(Ordering::Relaxed))?;
        sockhandle.store(0, Ordering::Relaxed);
    }
    Ok(())
}

/// Set rx buffer status
///
/// # Parameters
/// - `port`: Port context struct
/// - `index`: index in buffer array
/// - `bufstat`: status to set
pub fn set_buf_stat(port: &mut Port, index: usize, bufstat: BufferState) {
    port.rx_buf_stat.lock().unwrap()[index] = bufstat;
    if let Some(redport) = port
        .redport
        .as_mut()
        .filter(|_| port.redstate != RedundancyMode::None)
    {
        redport.rxbufstat.lock().unwrap()[index] = bufstat;
    }
}

/// Get new frame identifier index and allocate corresponding rx buffer
///
/// # Parameters
/// - `port`: port context struct
///
/// # Returns
/// New index
pub fn get_index(port: &mut Port) -> u8 {
    let mut index = port.last_index + 1;

    if index >= MAX_BUF_COUNT {
        index = 0;
    }
    let mut rx_buf_stat = port.rx_buf_stat.lock().unwrap();
    let index = (index..)
        .take(MAX_BUF_COUNT)
        .zip(rx_buf_stat.iter().skip(index))
        .find(|&(_, rx_buf_stat)| *rx_buf_stat == BufferState::Empty)
        .map(|(index, _)| index)
        .unwrap_or(index);
    rx_buf_stat[index] = BufferState::Alloc;
    if let Some(redport) = port
        .redport
        .as_mut()
        .filter(|_| port.redstate != RedundancyMode::None)
    {
        redport.rxbufstat.lock().unwrap()[index] = BufferState::Alloc;
    }

    index as u8
}

fn get_stack<'a, 'b>(port: &'a Port, stacknumber: i32) -> &'b Stack<'a> {
    if let Some(redport) = port.redport.as_ref().filter(|_| stacknumber == 0) {
        &redport.stack
    } else {
        &port.stack
    }
}

/// Transmit buffer over socket (non blocking)
///
/// # Parameters
/// - `port`: port context struct
/// - `index`: index in tx buffer array
/// - `stacknumber`: 0=primary, 1=secondary (if available)
///
/// # Returns
/// Socket send result
pub fn out_frame(port: &mut Port, index: usize, stacknumber: i32) -> i32 {
    let stack = get_stack(port, stacknumber);
    stack.rx_buf_stat.lock().unwrap()[index] = BufferState::Tx;
    let tx_buffer = stack.tx_buffer.lock().unwrap();
    let buffer: &[u8] = tx_buffer[index].as_ref();
    let rval = unsafe {
        libc::send(
            stack.socket.load(Ordering::Relaxed),
            (buffer as *const [u8]).cast(),
            buffer.len(),
            0,
        )
    };
    if rval == -1 {
        stack.rx_buf_stat.lock().unwrap()[index] = BufferState::Empty;
    }

    rval as i32
}

/// Transmit buffer over socket (non blocking)
///
/// # Parameters
/// - `port`: port context struct
/// - `index`: index in tx buffer array
///
/// # Returns
/// Socket send result
pub fn out_frame_red(port: &mut Port, index: u8) -> Result<i32, NicdrvError> {
    let mut ehp =
        EthernetHeader::try_from(port.tx_buffers.lock().unwrap()[usize::from(index)].as_slice())?;

    // Rewrite MAC source address 1 to primary
    ehp.source_address[1] = htons(PRIMARY_MAC[1]);
    port.tx_buffers.lock().unwrap()[usize::from(index)]
        .as_mut_slice()
        .copy_from_slice(ehp.as_ref());

    // Transmit over primary socket
    let rval = out_frame(port, index.into(), 0);

    if port.redstate != RedundancyMode::None {
        let mut tmp_buffer = port.temp_tx_buffer.lock().unwrap();
        let mut ehp = EthernetHeader::try_from(tmp_buffer.as_slice())?;

        // Use dummy frame for secondary socket transmit (BRD)
        let mut datagram = EthercatHeader::try_from(&tmp_buffer[ETHERNET_HEADER_SIZE..])?;

        // Write index to frame
        datagram.index = index;

        // Rewrite MAC source address 1 to secondary
        ehp.source_address[1] = htons(SECONDARY_MAC[1]);
        tmp_buffer.copy_from_slice(ehp.as_ref());
        tmp_buffer[ETHERNET_HEADER_SIZE..].copy_from_slice(datagram.as_ref());

        // Transmit over secondary socket
        let redport = port.redport.as_ref().unwrap();
        redport.rxbufstat.lock().unwrap()[usize::from(index)] = BufferState::Tx;
        if unsafe {
            libc::send(
                redport.sockhandle.load(Ordering::Relaxed),
                tmp_buffer.as_ptr().cast(),
                tmp_buffer.len(),
                0,
            )
        } == -1
        {
            redport.rxbufstat.lock().unwrap()[usize::from(index)] = BufferState::Empty;
        }
    }
    Ok(rval)
}

/// Non blocking read of socket. Put frame in temporary buffer.
///
/// # Parameters
/// - `port`: port context struct
/// - `stacknumber`: 0=primary 1=secondary stack (if available)
pub fn ecx_receive_packet(port: &mut Port, stacknumber: i32) -> i32 {
    let stack = get_stack(port, stacknumber);
    let mut temp_buf = stack.temp_buf.lock().unwrap();
    let bytesrx = unsafe {
        libc::recv(
            stack.socket.load(Ordering::Relaxed),
            temp_buf.as_mut_ptr().cast(),
            temp_buf.len(),
            0,
        )
    };
    drop(temp_buf);
    port.temp_in_buf_stat = if bytesrx == 0 {
        BufferState::Empty
    } else {
        BufferState::Alloc
    };
    i32::from(bytesrx > 0)
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
/// - `port`: port context struct
/// - `index`: requested index of frame
/// - `stacknumber`: 0=primary, 1=secondary stack (if available)
///
/// # Returns
/// Workcounter if a frame is found with corresponding index, otherwise
/// Error::NoFrame or Error::OtherFrame
pub fn inframe(port: &mut Port, index: u8, stacknumber: i32) -> Result<u16, NicdrvError> {
    let mut rval = Err(EthercatError::NoFrame);

    // Check if requested index is already in buffer
    if usize::from(index) < MAX_BUF_COUNT
        && get_stack(port, stacknumber).rx_buf_stat.lock().unwrap()[usize::from(index)]
            != BufferState::Rcvd
    {
        let rxbuf = get_stack(port, stacknumber).rx_buffers.lock().unwrap();
        let rxbuf = rxbuf[usize::from(index)].as_slice();
        let l = usize::from(rxbuf[0]) + ((usize::from(rxbuf[1]) & 0xF) << 8);

        // Return WKC
        rval = Ok(u16::from(rxbuf[l]) + (u16::from(rxbuf[l + 1]) << 8));
        get_stack(port, stacknumber).rx_buf_stat.lock().unwrap()[usize::from(index)] =
            BufferState::Complete;
    } else if ecx_receive_packet(port, stacknumber) != 0 {
        rval = Err(EthercatError::OtherFramce);
        let ehp = EthernetHeader::try_from(
            get_stack(port, stacknumber)
                .temp_buf
                .lock()
                .unwrap()
                .as_slice(),
        )?;
        if ehp.etype == htons(ETH_P_ECAT) {
            let ecp = EthercatHeader::try_from(
                &get_stack(port, stacknumber).temp_buf.lock().unwrap()[ETHERNET_HEADER_SIZE..],
            )?;
            let l = usize::from(ethercat_to_host(ecp.ethercat_length) & 0x0FFF);
            let index_found = ecp.index;

            // Check whether the index is the index we're looking for
            if index_found == index {
                // Put it in the buffer
                let stack = get_stack(port, stacknumber);
                let rx_buf = &mut stack.rx_buffers.lock().unwrap()[usize::from(index)];
                rx_buf.clear();
                rx_buf
                    .extend_from_slice(&stack.temp_buf.lock().unwrap()[ETHERNET_HEADER_SIZE..])
                    .unwrap();

                // Return WKC
                rval = Ok(u16::from(rx_buf[l]) + (u16::from(rx_buf[l + 1])));

                // Mark as completed
                stack.rx_buf_stat.lock().unwrap()[usize::from(index)] = BufferState::Complete;
                stack.rx_source_address.lock().unwrap()[usize::from(index)] =
                    ntohs(ehp.source_address[1]).into();
            } else if usize::from(index_found) < MAX_BUF_COUNT
                && get_stack(port, stacknumber).rx_buf_stat.lock().unwrap()
                    [usize::from(index_found)]
                    == BufferState::Tx
            {
                // If the index exists and someone is waiting for it

                let stack = get_stack(port, stacknumber);

                // Put it in the buffer array
                let rx_buf = &mut stack.rx_buffers.lock().unwrap()[usize::from(index_found)];
                rx_buf.clear();
                rx_buf
                    .extend_from_slice(&stack.temp_buf.lock().unwrap()[ETHERNET_HEADER_SIZE..])
                    .unwrap();

                // Mark as received
                stack.rx_buf_stat.lock().unwrap()[usize::from(index_found)] = BufferState::Rcvd;
                stack.rx_source_address.lock().unwrap()[usize::from(index_found)] =
                    ntohs(ehp.source_address[1]).into();
            }
        }
    }
    Ok(rval?)
}

/// Blocking redundant receive frame function. If redundant mode is not active, then
/// it skips the secondary stack and redundancy functions. In redundant mode, it waits
/// for both (primary and secondary) frames to come in. The result goes in a decision
/// tree that decides, depending on the route of the packet and its possible missing arrive,
/// how to reroute the original packet to get the data in an other try.
///
/// # Parameters
/// - `port`: port context struct
/// - `index`: requested index of frame
/// - `timer`: Absolute timeout time
///
/// # Returns
/// Workcounter if a frame is found with corresponding index, otherwise `NicdrvError`
pub fn wait_in_frame_red(port: &mut Port, index: u8, timer: OsalTimer) -> Result<u16, NicdrvError> {
    // If not in redundant mode, always assume secondary is ok
    let mut wkc2 = if port.redstate == RedundancyMode::None {
        Ok(0)
    } else {
        Err(NicdrvError::EthercatError(EthercatError::NoFrame))
    };
    let mut wkc = Err(NicdrvError::EthercatError(EthercatError::NoFrame));

    loop {
        // Only read frame if not already in
        if wkc.is_err() {
            wkc = inframe(port, index, 0);
        }

        // Only try secondary if in redundant mode and not already in
        if port.redstate != RedundancyMode::None && wkc2.is_err() {
            wkc2 = inframe(port, index, 1);
        }

        // Wait for both frames to arrive or timeout
        if (wkc.is_ok() && wkc2.is_ok()) || timer.is_expired() {
            break;
        }
    }

    // Only do redundant functions when in redundant mode
    if port.redstate != RedundancyMode::None {
        // primrx if the received MAC source on the primary socket
        let primrx = if wkc.is_ok() {
            port.rx_source_address.lock().unwrap()[usize::from(index)]
        } else {
            0
        };

        // Secrx if the received MAC source on psecondary socket
        let secrx = if let Some(redport) = port.redport.as_ref().filter(|_| wkc2.is_ok()) {
            redport.rx_source_address.lock().unwrap()[usize::from(index)]
        } else {
            0
        };

        // Primary socket got secondary frame and secondary socket got primary frame
        if primrx == RX_SECONDARY.into() && secrx == RX_PRIMARY.into() {
            // Copy secondary buffer to primary
            let rxbuf = &mut port.rx_buf.lock().unwrap()[usize::from(index)];
            rxbuf.clear();
            rxbuf
                .extend_from_slice(
                    port.redport.as_ref().unwrap().rxbuf.lock().unwrap()[usize::from(index)]
                        .as_slice(),
                )
                .unwrap();
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
                let txbuf = &mut port.tx_buffers.lock().unwrap()[usize::from(index)];
                txbuf.resize(ETHERNET_HEADER_SIZE, 0).unwrap();
                txbuf
                    .extend_from_slice(port.rx_buf.lock().unwrap()[usize::from(index)].as_slice())
                    .unwrap();
            }
            let timer2 = OsalTimer::new(TIMEOUT_RETURN);

            // Resend secondary tx
            out_frame(port, usize::from(index), 1);

            loop {
                wkc2 = inframe(port, index, 1);
                if wkc2.is_ok() || timer2.is_expired() {
                    break;
                }
            }
            if wkc2.is_ok() {
                let rx_buf = &mut port.rx_buf.lock().unwrap()[usize::from(index)];
                rx_buf.clear();
                rx_buf
                    .extend_from_slice(
                        port.redport.as_ref().unwrap().rxbuf.lock().unwrap()[usize::from(index)]
                            .as_slice(),
                    )
                    .unwrap();
            }
        }
    }

    // Return workcounter or error
    wkc
}

/// Blocking receive frame function. Calls `wait_in_frame_red`.
///
/// # Parameters
/// - `port`: Port context struct
/// - `index`: Requested index of frame
/// - `timeout`: Timeout duration
///
/// # Returns
/// Workcounter if a frame is found with corresponding index, otherwise `NicdrvError`
pub fn wait_in_frame(port: &mut Port, index: u8, timeout: Duration) -> Result<u16, NicdrvError> {
    let timer = OsalTimer::new(timeout);
    wait_in_frame_red(port, index, timer)
}

/// Blocking send and receive frame function. used for non processdata frames.
/// A datagram is build into a frame and transmitted via this function. It waits
/// for and answer and returns the workcounter. The function retries if time is
/// left and the result is WKC=0 or no frame received.
///
/// The function calls `ec_outframe_red` and `ec_waitinframe_red`.
///
/// # Parameters
/// - `port`: port context struct
/// - `index`: index of frame
/// - `timeout`: timeout duration
///
/// # Returns
/// Workcounter of NicdrvError
pub fn src_confirm(port: &mut Port, index: u8, timeout: Duration) -> Result<u16, NicdrvError> {
    let timer1 = OsalTimer::new(timeout);
    loop {
        // Tx frame on primary and if in redundant mode a dummy on secondary
        out_frame_red(port, index)?;
        let timer2 = OsalTimer::new(timeout.min(TIMEOUT_RETURN));
        let wkc = wait_in_frame_red(port, index, timer2);
        if wkc.is_ok() || timer1.is_expired() {
            break wkc;
        }
    }
}
