use std::{
    net::TcpStream,
    sync::{Arc, Mutex},
    time::Duration,
};

use crate::ethercat::r#type::MAX_BUF_COUNT;

/// EtherCAT Pointer structure to Tx and Rx stacks
pub struct Stack<'stack> {
    /// Socket connection used
    pub socket: &'stack Mutex<TcpStream>,

    /// Send buffers
    pub tx_buffer: &'stack Mutex<[Vec<u8>; MAX_BUF_COUNT]>,

    /// Temporary receive buffer
    pub temp_buf: &'stack Mutex<[u8]>,

    /// Rreceive buffers
    pub rx_buffers: &'stack Mutex<[Vec<u8>; MAX_BUF_COUNT]>,

    /// Receive buffer status fields
    pub rx_buf_stat: &'stack Mutex<[Vec<i32>; MAX_BUF_COUNT]>,

    /// Received MAC source address (middle word)
    pub rx_source_address: &'stack Mutex<[Vec<i32>; MAX_BUF_COUNT]>,
}

/// EtherCAT eXtended Pointer structure to buffers for redundant port
pub struct RedPort<'redport> {
    pub stack: Stack<'redport>,
    pub sockhandle: Arc<Mutex<TcpStream>>,

    /// Receive buffers
    pub rxbuf: Mutex<[Vec<u8>; MAX_BUF_COUNT]>,

    /// Receive buffer status
    pub rxbufstat: Mutex<[Vec<i32>; MAX_BUF_COUNT]>,

    /// Receive MAC source address
    pub rx_source_address: Mutex<[i32; MAX_BUF_COUNT]>,

    pub temp_in_buf: &'redport Mutex<[u8]>,
}

/// Ethercat eXtended pointer structure to buffers, variables, and mutexes for port instantiation
pub struct Port<'port> {
    pub stack: Stack<'port>,
    pub sockhandle: Arc<Mutex<TcpStream>>,

    /// Rx buffers
    pub rx_buf: [Vec<u8>; MAX_BUF_COUNT],

    /// Receive buffer status
    pub rx_buf_stat: [i32; MAX_BUF_COUNT],

    /// Receive MAC source address
    pub rx_source_address: [i32; MAX_BUF_COUNT],

    /// Temporary receive buffer
    pub temp_in_buf: &'port Mutex<[u8]>,

    /// Temporary receive buffer status
    pub temp_in_buf_stat: i32,

    /// Transmit buffers
    pub tx_buffers: [Vec<u8>; MAX_BUF_COUNT],

    /// Temporary receive buffer
    pub temp_tx_buffer: &'port Mutex<[u8]>,

    /// last used frame index
    pub last_index: usize,

    /// Current redundancy state
    pub redstate: i32,

    pub redport: &'port Mutex<RedPort<'port>>,
}

extern "C" {
    /// Primary MAC address
    pub static PRI_MAC: [u16; 3];

    /// Secondary MAC address
    pub static SEC_MAC: [u16; 3];
}

#[cfg(feature = "ec_ver1")]
pub mod ec_ver1 {
    use std::{net::TcpStream, time::Duration};

    /// Setup EtherCAT Network Interface Card
    pub fn ec_setup(input_file_name: &str, secondary: i32) -> i32 {
        todo!()
    }

    /// Close EtherCAT Network Interface Card
    pub fn ec_close() -> i32 {
        todo!()
    }

    pub fn set_buf_stat(index: u8, bufstat: i32) {
        todo!()
    }

    pub fn get_index() -> u8 {
        todo!()
    }

    pub fn out_frame(index: u8, socket: &TcpStream) -> i32 {
        todo!()
    }

    pub fn out_frame_red(index: u8) -> i32 {
        todo!()
    }

    pub fn wait_in_frame(index: u8, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn src_confirm(index: u8, timeout: Duration) -> i32 {
        todo!()
    }
}

pub fn setup_header<T>(header: &mut T) {
    todo!()
}

pub fn setup_nic<const MAX_BUF_COUNT: usize>(
    port: &mut Port,
    input_file_name: &str,
    secondary: i32,
) -> i32 {
    todo!()
}

pub fn close_nic<const MAX_BUF_COUNT: usize>(port: &mut Port) -> i32 {
    todo!()
}

pub fn set_buf_stat<const MAX_BUF_COUNT: usize>(port: &mut Port, index: u8, bufstat: i32) {
    todo!()
}

pub fn get_index<const MAX_BUF_COUNT: usize>(port: &mut Port) -> u8 {
    todo!()
}

pub fn out_frame<const MAX_BUF_COUNT: usize>(
    port: &mut Port,
    index: u8,
    socket: &TcpStream,
) -> i32 {
    todo!()
}

pub fn out_frame_red<const MAX_BUF_COUNT: usize>(port: &mut Port, index: u8) -> i32 {
    todo!()
}

pub fn wait_in_frame<const MAX_BUF_COUNT: usize>(
    port: &mut Port,
    index: u8,
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn src_confirm<const MAX_BUF_COUNT: usize>(
    port: &mut Port,
    index: u8,
    timeout: Duration,
) -> i32 {
    todo!()
}
