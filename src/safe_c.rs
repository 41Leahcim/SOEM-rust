//! Safer abstraction over C function.
//!
//! Improves safety by returning an error on failure and limiting the possible arguments.

use std::{ffi::CStr, ptr};

use libc::{
    socklen_t, EACCES, EAFNOSUPPORT, EBADF, EDQUOT, EFAULT, EINTR, EINVAL, EMFILE, ENFILE, ENOBUFS,
    ENOMEM, ENOPROTOOPT, ENOSPC, ENOTSOCK, EPROTONOSUPPORT,
};

use crate::oshw::Network;

#[derive(Debug)]
pub enum SocketError {
    /// Permission to create a socket of the specified type and/or protocol is denied
    AccessDenied,

    /// The specified address family is not supported by the implementation
    AddressFamilyNotSupported,

    /// Unknown protocol, protocol family not supported, or invalid falgs in type.
    InvalidValue,

    ProcessFileTableOverflow,

    /// The system limit on the total number of open files has been reached
    FileLimitReached,

    /// Insufficient memory is available, the socket can't be created until sufficient resources
    /// are freed.
    InsufficientMemoryAvailable,

    /// The protocol type or the specified protocol is not supported within this domain
    ProtocolNotSupported,

    /// Failed to retrieve error or error not recognized
    InvalidError,
}

#[derive(Debug)]
pub enum SockOptError {
    /// Sockfd is not a valid descriptor
    BadFileDescriptor,

    /// Address pointed to by optval is not a valid part of process address sace.
    /// For getsockopt() the optlen may not be a valid part of process address space.
    Fault,

    /// Optlen invalid in setsockopt(). In some cases this error can occur for an invalid
    /// value in optval (e.g. for IP_ADD_MEMBERSHIP option described in ip
    /// (https://linux.die.net/man/7/ip))
    InvalidValue,

    /// The option is unknown at the level indicated
    NoProtoOpt,

    /// The argument sockfd is a file, not a socket
    NotSock,

    /// Failed to retrieve error or error not recognized
    InvalidError,
}

#[derive(Debug)]
pub enum IoctlError {
    /// Fd is not a valid file descriptor
    BadFileDescriptor,

    /// Argp references an inaccessible memory area
    InaccessibleMemoryArea,

    /// Op or argp is not valid
    InvalidOperation,

    /// Fd is not associated with a character special device or the specified operation doesn't
    /// apply to the kind of object the file descriptor references.
    NotTy,

    /// Failed to retrieve the error or error not recognized
    InvalidError,
}

#[derive(Debug)]
pub enum CloseError {
    /// Fd isn't a valid open file descriptor
    FileDescriptorClosedOrInvalid,

    /// Close call was interrupted by a signal
    Interupted,

    /// An I/O error occured
    IoError,

    /// On NFS, this error is not normally reported against the first write exceeding available
    /// storage space, but instead against a subsequent write, fsync, or close.
    StorageSpaceExceeded,

    /// Failed to retrieve error or error not recognized
    InvalidError,
}

#[derive(Debug)]
pub enum BindError {
    /// The specified address is already in use
    AddressInUse,

    /// The specified address is not available from the local machine
    AddressNotAvailable,

    /// The specified address is not a valid address for the address family of the specified socket
    AddressFamilyNotSupported,

    /// The Socket argment is not a valid file descriptor
    InvalidFileDescriptor,

    /// The socket is already bound to an address, and the protocol doesn't support binding to a
    /// new address, or the socket has been shutdown.
    AlreadyBound,

    /// The socket argument doesn't refer to a socket
    NotSocket,

    /// The type of the specified socket doesn't support binding to an address
    OperationNotSupported,

    /// A component of the path prefix denies search permission, or the requested name requires
    /// writing in a directory with a mode that denies write permission.
    AccessDenied,

    /// The address argument is a null pointer
    AddressArgumentIsNull,

    /// An I/O error occured
    Io,

    /// A loop exists in symbolic links encountered during resolution of pathname in address
    Loop,

    /// A component of a pathname exceeded `libc::NAME_MAX` characters, or an entire pathname
    /// exceeded `libc::PATH_MAX` characters
    NameTooLong,

    /// A component of the pathname doesn't name an existing file or the pathname is empty.
    NameDoesNotExistOrEmpty,

    /// A component of the path prefix of the pathname in address is not a directory
    NotADirectory,

    /// The name would reside on a read-only file system
    ReadOnlyFileSystem,

    /// The socket is already connected
    SocketAlreadyConnected,

    /// Insufficient resources were available to complete call
    InsufficientResources,

    /// Failed to retrieve error or error not recognized
    InvalidError,
}

#[derive(Debug)]
pub enum ReceiveError {
    /// The socket's file descriptor is marked `O_NONBLOCK` and no data is waiting to be received;
    /// or `MSG_OOB` is set and no out-of-band data is available and either the socket's
    /// file descriptor is marked `O_NONBLOCK` or the socket doesn't support blocking to await
    /// out-of-band data.
    WouldBlock,

    /// The socket argument is not a valid file descriptor
    BadFileDescriptor,

    /// The connection was forcibly closed by a peer
    ConnectionReset,

    /// The receive function was interrupted by a signal that was caught, before any data was
    /// available.
    Interrupted,

    /// The MSG_OOB flag is set  and no out-of-band data is available
    NoOutOfBandDataAvailable,

    /// A receive is attempted on a connection-mode socket that is not connected.
    NotConnected,

    /// The socket argument doesn't refer to a socket
    NotSocket,

    /// The specified flags are not supported for this socket type or protocol
    OperationNotSupported,

    /// The connection timed out during connection establishment, or due to transmission timeout
    /// on active connection.
    TimedOut,

    /// An I/O error occured while reading from or writing to the file system
    Io,

    /// Insufficient resources were available in the system to perform the operation
    NoBuffers,

    /// Insufficient memory was available to fullfill the request
    NoMemory,

    /// Failed to retrieve error or error not recognized
    InvalidError,
}

#[derive(Debug)]
pub enum SendError {
    /// The socket's file descriptor is marked `O_NONBLOCK` and the requested operation would block.
    WouldBlock,

    /// The socket argument is not a valid file descriptor
    BadFileDescriptor,

    /// A connection was forcibly closed by a peer
    ConnectionReset,

    /// The socket is not connection-mode and no peer address is set
    DestinationAddressRequired,

    /// A signal interrupted `send` before any data was transmitted
    Interrupted,

    /// The message is too large to be sent all at once, as the socket requires
    MessageSize,

    /// The socket is not connected or hasn't had the peer pre-specified
    NotConnected,

    /// The socket argument doesn't refer to a socket
    NotASocket,

    /// THe socket argument is associated with a socket that doesn't support one or more of the
    /// values set in flags.
    OperationNotSupported,

    /// Socket shut down for writing or the socket is connection-mode and is no longer connected.
    /// In the latter case, and if the socket is of type `SOCK_STREAM`, the SIGPIPE signal is
    /// generated to the calling thread
    SocketShutDownForWritingOrNoLongerConnected,

    /// The calling process doesn't have the appropriate privileges
    AccessDenied,

    /// An I/O error occured while reading from or writing to the file system
    Io,

    /// The local network interface used to reach the destination is down
    NetworkDown,

    /// No route to the network is present
    NetworkUnreachable,

    /// Insufficient resources were available ni the system to perform the operation
    InsufficientResources,

    /// Failed to retrieve error or error not recognized
    InvalidError,
}

#[derive(Debug)]
pub enum CError {
    Socket(SocketError),
    SockOpt(SockOptError),
    Ioctl(IoctlError),
    Bind(BindError),
    Receive(ReceiveError),
    Send(SendError),
    Close(CloseError),
}

impl From<SocketError> for CError {
    fn from(value: SocketError) -> Self {
        Self::Socket(value)
    }
}

impl From<SockOptError> for CError {
    fn from(value: SockOptError) -> Self {
        Self::SockOpt(value)
    }
}

impl From<IoctlError> for CError {
    fn from(value: IoctlError) -> Self {
        Self::Ioctl(value)
    }
}

impl From<BindError> for CError {
    fn from(value: BindError) -> Self {
        Self::Bind(value)
    }
}

impl From<ReceiveError> for CError {
    fn from(value: ReceiveError) -> Self {
        Self::Receive(value)
    }
}

impl From<SendError> for CError {
    fn from(value: SendError) -> Self {
        Self::Send(value)
    }
}

impl From<CloseError> for CError {
    fn from(value: CloseError) -> Self {
        Self::Close(value)
    }
}

#[derive(Clone, Copy)]
pub struct CBool(libc::c_int);

impl CBool {
    pub const fn new(value: bool) -> Self {
        Self(value as libc::c_int)
    }
}

#[derive(Clone, Copy)]
pub enum SocketOption {
    /// Enables debug if true, disables if false
    Debug(CBool),

    /// Permits sending broadcast messages if true and supported
    Broadcast(CBool),

    /// Permits reuse of the address if true
    ReuseAddress(CBool),

    /// Keeps connection active if true and supported
    KeepAlive(CBool),

    /// Lingers on a close (drop) if data is present.
    /// This option controls the action taken when unsent messages queue on a socket and
    /// the socket is dropped.  If set, the system shall block the thread during `Socket::drop`
    /// until it can transmit the data or until the time expires. If linger is not specified,
    /// and the socket is dropped, the system handles the call in a way that allows the calling
    /// thread to continue as quickly as possible.
    Linger(libc::linger),

    /// Leaves received out-of-band data (data marked as urgent) inline if enabled
    OutOfBandInline(CBool),

    /// Sets send buffer size
    SendBufferSize(libc::c_uint),

    /// Sets receive buffer size
    ReceiveBufferSize(libc::c_uint),

    /// Requests that outgoing messages bypass the standard routing facilities. The destination
    /// shall be on a directly-connected network, and messages are directed to the appropriate
    /// network interface according to the destination effect. The effect depends on the protocol.
    DoNotRoute(CBool),

    /// Sets the minimum number of bytes to process for socket input operations. The default value
    /// is 1. If set to a larger value, blocking receive calls normally wait until they have
    /// received the smaller of the low water mark value or the requested amount. (They may return
    /// less than the low water mark if an error occurs, a signal is caught, or the type of data
    /// next in the receive queue is different from returned; For example, out-of-band data).
    ReceiveLowWatermark(libc::c_uint),

    /// Sets the timeout value that specifies the maximum amount of time an input function waits
    /// until completetion. The timeval structure specifies the limit on how long to wait for an
    /// input operation to complete. If a receive operation has blocked for this much time without
    /// receiving additional data, it shall return with a partial count or error if no data is
    /// received. The default for this option is zero, which indicates that a receive operation
    /// shall not time out.
    ReceiveTimeout(libc::timeval),

    /// Sets the minimum number of bytes to process for socket output operations. Non-blocking
    /// output operations shall process no data if flow control doesn´t allow the smaller of the
    /// low water mark value or the entire request to be processed.
    SendLowWatermark(libc::c_uint),

    /// Sets the timeout value specifying the amount of time that an output function blocks
    /// because flow control prevents data from being sent. If a send operation has blocked for
    /// this time, it shall return with a partial count or error if no data was sent. THe default
    /// value is 0, which indicates that a send operation shall not time out.
    SendTimeout(libc::timeval),
}

impl SocketOption {
    pub const fn name(&self) -> libc::c_int {
        match self {
            Self::Debug(_) => libc::SO_DEBUG,
            Self::Broadcast(_) => libc::SO_BROADCAST,
            Self::ReuseAddress(_) => libc::SO_REUSEADDR,
            Self::KeepAlive(_) => libc::SO_KEEPALIVE,
            Self::Linger(_) => libc::SO_LINGER,
            Self::OutOfBandInline(_) => libc::SO_OOBINLINE,
            Self::SendBufferSize(_) => libc::SO_SNDBUF,
            Self::ReceiveBufferSize(_) => libc::SO_RCVBUF,
            Self::DoNotRoute(_) => libc::SO_DONTROUTE,
            Self::ReceiveLowWatermark(_) => libc::SO_RCVLOWAT,
            Self::ReceiveTimeout(_) => libc::SO_RCVTIMEO,
            Self::SendLowWatermark(_) => libc::SO_SNDLOWAT,
            Self::SendTimeout(_) => libc::SO_SNDTIMEO,
        }
    }

    pub const fn value(&self) -> *const libc::c_void {
        match self {
            Self::Debug(enabled)
            | Self::Broadcast(enabled)
            | Self::ReuseAddress(enabled)
            | Self::KeepAlive(enabled)
            | Self::OutOfBandInline(enabled)
            | Self::DoNotRoute(enabled) => ptr::from_ref(&enabled.0).cast(),
            Self::Linger(linger) => ptr::from_ref(linger).cast(),
            Self::SendBufferSize(size) | Self::ReceiveBufferSize(size) => {
                ptr::from_ref(size).cast()
            }
            Self::ReceiveLowWatermark(water_mark) | Self::SendLowWatermark(water_mark) => {
                ptr::from_ref(water_mark).cast()
            }
            Self::ReceiveTimeout(timeval) | Self::SendTimeout(timeval) => {
                ptr::from_ref(timeval).cast()
            }
        }
    }

    pub const fn option_len(&self) -> socklen_t {
        match self {
            Self::Debug(_)
            | Self::Broadcast(_)
            | Self::ReuseAddress(_)
            | Self::KeepAlive(_)
            | Self::OutOfBandInline(_)
            | Self::DoNotRoute(_) => size_of::<libc::c_int>() as socklen_t,
            Self::Linger(_) => size_of::<libc::linger>() as socklen_t,
            Self::SendBufferSize(_)
            | Self::ReceiveBufferSize(_)
            | Self::ReceiveLowWatermark(_)
            | Self::SendLowWatermark(_) => size_of::<libc::c_uint>() as socklen_t,
            Self::ReceiveTimeout(_) | Self::SendTimeout(_) => {
                size_of::<libc::timeval>() as socklen_t
            }
        }
    }
}

pub struct SocketBuilder {
    fd: i32,
}

impl SocketBuilder {
    /// Binds the socket and returns a build socket object.
    ///
    /// # Errors
    /// Returns an error on failure to bind the socket.
    pub fn bind(
        self,
        address: &libc::sockaddr,
        address_length: socklen_t,
    ) -> Result<Socket, BindError> {
        if unsafe { libc::bind(self.fd, ptr::from_ref(address), address_length) } == 0 {
            return Ok(Socket { fd: self.fd });
        }
        Err(match unsafe { *libc::__errno_location() } {
            libc::EADDRINUSE => BindError::AddressInUse,
            libc::EADDRNOTAVAIL => BindError::AddressNotAvailable,
            libc::EAFNOSUPPORT => BindError::AddressFamilyNotSupported,
            libc::EBADF => BindError::InvalidFileDescriptor,
            libc::EINVAL => BindError::AlreadyBound,
            libc::ENOTSOCK => BindError::NotSocket,
            libc::EOPNOTSUPP => BindError::OperationNotSupported,
            libc::EACCES => BindError::AccessDenied,
            libc::EDESTADDRREQ | libc::EISDIR => BindError::AddressArgumentIsNull,
            libc::EIO => BindError::Io,
            libc::ELOOP => BindError::Loop,
            libc::ENAMETOOLONG => BindError::NameTooLong,
            libc::ENOENT => BindError::NameDoesNotExistOrEmpty,
            libc::ENOTDIR => BindError::NotADirectory,
            libc::EROFS => BindError::ReadOnlyFileSystem,
            libc::EISCONN => BindError::SocketAlreadyConnected,
            libc::ENOBUFS => BindError::InsufficientResources,
            _ => BindError::InvalidError,
        })
    }

    /// # Errors
    /// Returns an error if:
    /// - The socket file descriptor is invalid
    /// - The address pointed to by value is not valid
    /// - The option_len is invalid or the value in optval is invalid
    /// - The protocol is invalid
    /// - The socket is a file, not a socket
    /// - An unknown error was stored in ERRNO
    pub fn set_option(self, option: SocketOption) -> Result<Self, SockOptError> {
        if unsafe {
            libc::setsockopt(
                self.fd,
                libc::SOL_SOCKET,
                option.name(),
                option.value(),
                option.option_len(),
            )
        } == 0
        {
            return Ok(self);
        }
        match unsafe { *libc::__errno_location() } {
            EBADF => Err(SockOptError::BadFileDescriptor),
            EFAULT => Err(SockOptError::Fault),
            EINVAL => Err(SockOptError::InvalidValue),
            ENOPROTOOPT | ENOTSOCK => Err(SockOptError::NoProtoOpt),
            _ => Err(SockOptError::InvalidError),
        }
    }

    /// Sets an ioctl option to the specified value.
    ///
    /// # Errors
    /// Returns an error on failure to set the new configuration.
    pub fn ioctl<T>(self, request: libc::c_ulong, value: T) -> Result<Self, IoctlError> {
        if unsafe { libc::ioctl(self.fd, request, value) } != -1 {
            return Ok(self);
        }
        Err(match unsafe { *libc::__errno_location() } {
            libc::EBADF => IoctlError::BadFileDescriptor,
            libc::EFAULT => IoctlError::InaccessibleMemoryArea,
            libc::EINVAL => IoctlError::InvalidOperation,
            libc::ENOTTY => IoctlError::NotTy,
            _ => IoctlError::InvalidError,
        })
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ReceiveFlags(libc::c_int);

impl ReceiveFlags {
    pub const fn new() -> Self {
        Self(0)
    }

    /// Enables peeking at incoming messages. The data is treated as unread and the next receive
    /// or similar function shall still return this data.
    #[must_use]
    pub const fn peek(mut self) -> Self {
        self.0 |= libc::MSG_PEEK;
        self
    }

    /// Enables requesting out-of-band data. The significance and semantics of out-of-band data
    /// are protocol-specific.
    #[must_use]
    pub const fn out_of_band(mut self) -> Self {
        self.0 |= libc::MSG_OOB;
        self
    }

    /// Enables requesting that the function blocks until the full amount of data can be returned,
    /// if the socket is a `SOCK_STREAM` socket. The function may return the smaller amount of
    /// data if the socket is a message based socket, a signal is caught, the connection is
    /// terminated, `MSG_PEEK` was specified or if an error is pending for the socket.
    #[must_use]
    pub const fn wait_all(mut self) -> Self {
        self.0 |= libc::MSG_WAITALL;
        self
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SendFlags(libc::c_int);

impl SendFlags {
    pub const fn new() -> Self {
        Self(0)
    }

    /// Terminates a record if supported by the protocol
    #[must_use]
    pub const fn end_of_record(mut self) -> Self {
        self.0 |= libc::MSG_EOR;
        self
    }

    /// Send out-of-band data on sockets that support out-of-band communication. THe significance
    /// of out-of-band data are protocol-specific.
    #[must_use]
    pub const fn out_of_band(mut self) -> Self {
        self.0 |= libc::MSG_OOB;
        self
    }
}

pub struct Socket {
    fd: i32,
}

impl Socket {
    /// # Errors
    /// Returns an error if:
    /// - Permission to create the socket is denied
    /// - The specified address family is not supported
    /// - The requested protocol is not supported
    /// - The system reached the maximum number of open files/interfaces
    /// - There is not enough memory available for the socket to be created
    /// - An unknown error was stored in ERRNO
    pub fn builder(
        socket_family: i32,
        socket_type: i32,
        protocol: Network<i32>,
    ) -> Result<SocketBuilder, SocketError> {
        match unsafe { libc::socket(socket_family, socket_type, protocol.into_inner()) } {
            -1 => match unsafe { *libc::__errno_location() } {
                EACCES => Err(SocketError::AccessDenied),
                EAFNOSUPPORT => Err(SocketError::AddressFamilyNotSupported),
                EINVAL => Err(SocketError::InvalidValue),
                EMFILE => Err(SocketError::ProcessFileTableOverflow),
                ENFILE => Err(SocketError::FileLimitReached),
                ENOBUFS | ENOMEM => Err(SocketError::InsufficientMemoryAvailable),
                EPROTONOSUPPORT => Err(SocketError::ProtocolNotSupported),
                _ => Err(SocketError::InvalidError),
            },
            fd => Ok(SocketBuilder { fd }),
        }
    }

    /// Sends the buffer data over the socket.
    ///
    /// # Errors
    /// Returns an error if:
    /// - The data can't be send without blocking
    /// - The connection was reset
    /// - A destination address is required for the socket protocol.
    /// - Sending the data was interrupted
    /// - The message is too large
    /// - An IO error occured
    /// - The network is down
    /// - The peer's network is unreachable
    /// - Not enough resources are available to send the data
    pub fn send(&self, buffer: &[u8], flags: SendFlags) -> Result<usize, SendError> {
        #[expect(
            clippy::cast_sign_loss,
            reason = "Value checked for being positive before casting to unsigned"
        )]
        match unsafe { libc::send(self.fd, buffer.as_ptr().cast(), buffer.len(), flags.0) } {
            bytes_send @ 0.. => Ok(bytes_send as usize),
            _ => Err(match unsafe { *libc::__errno_location() } {
                libc::EWOULDBLOCK => SendError::WouldBlock,
                libc::EBADF => SendError::BadFileDescriptor,
                libc::ECONNRESET => SendError::ConnectionReset,
                libc::EDESTADDRREQ => SendError::DestinationAddressRequired,
                libc::EINTR => SendError::Interrupted,
                libc::EMSGSIZE => SendError::MessageSize,
                libc::ENOTCONN => SendError::NotConnected,
                libc::ENOTSOCK => SendError::NotASocket,
                libc::EPIPE => SendError::SocketShutDownForWritingOrNoLongerConnected,
                libc::EACCES => SendError::AccessDenied,
                libc::EIO => SendError::Io,
                libc::ENETDOWN => SendError::NetworkDown,
                libc::ENETUNREACH => SendError::NetworkUnreachable,
                libc::ENOBUFS => SendError::InsufficientResources,
                _ => SendError::InvalidError,
            }),
        }
    }

    /// Receives data over the socket.
    ///
    /// # Errors
    /// Returns an error if:
    /// - Receiving any data would block
    /// - The connection was reset
    /// - Receiving the data was interrupted
    /// - No out-of-band data was available
    /// - There is no connection
    /// - The passed combination of receive flags is not supported
    /// - The connection timed out
    /// - An IO error occured
    /// - There are no receive buffers available
    /// - There is not enough memory available to receive the message
    pub fn receive(&self, buffer: &mut [u8], flags: ReceiveFlags) -> Result<usize, ReceiveError> {
        #[expect(
            clippy::cast_sign_loss,
            reason = "Value checked for being positive before casting to unsigned"
        )]
        match unsafe { libc::recv(self.fd, ptr::from_mut(buffer).cast(), buffer.len(), flags.0) } {
            bytes @ 0.. => Ok(bytes as usize),
            _ => Err(match unsafe { *libc::__errno_location() } {
                libc::EWOULDBLOCK => ReceiveError::WouldBlock,
                libc::EBADF => ReceiveError::BadFileDescriptor,
                libc::ECONNRESET => ReceiveError::ConnectionReset,
                libc::EINTR => ReceiveError::Interrupted,
                libc::EINVAL => ReceiveError::NoOutOfBandDataAvailable,
                libc::ENOTCONN => ReceiveError::NotConnected,
                libc::ENOTSOCK => ReceiveError::OperationNotSupported,
                libc::ETIMEDOUT => ReceiveError::TimedOut,
                libc::EIO => ReceiveError::Io,
                libc::ENOBUFS => ReceiveError::NoBuffers,
                libc::ENOMEM => ReceiveError::NoMemory,
                _ => ReceiveError::InvalidError,
            }),
        }
    }
}

impl Drop for Socket {
    /// # Errors
    /// Returns an error if:
    /// - Fd isn't a valid open file descriptor
    /// - Close call was interrupted by a signal
    /// - An IO error occured
    /// - On NFS, the storage space was exceeded
    /// - An unknown error was stored in ERRNO
    fn drop(&mut self) {
        if unsafe { libc::close(self.fd) } == 0 {
            return;
        }
        match unsafe { *libc::__errno_location() } {
            EBADF => Err::<(), _>(CloseError::FileDescriptorClosedOrInvalid),
            EINTR => Err(CloseError::Interupted),
            ENOSPC | EDQUOT => Err(CloseError::StorageSpaceExceeded),
            _ => Err(CloseError::InvalidError),
        }
        .unwrap();
    }
}

pub struct Interface {
    pub index: u32,
    pub name: String,
}

pub struct InterfaceIterator {
    interface: *mut libc::if_nameindex,
}

impl Iterator for InterfaceIterator {
    type Item = Interface;

    fn next(&mut self) -> Option<Self::Item> {
        let interface = unsafe { self.interface.as_ref()? };
        if interface.if_index == 0 {
            self.interface = core::ptr::null_mut();
            return None;
        }
        let interface = Interface {
            index: interface.if_index,
            name: unsafe { CStr::from_ptr(interface.if_name) }
                .to_string_lossy()
                .into_owned(),
        };
        Some(interface)
    }
}

pub fn if_nameindex() -> InterfaceIterator {
    InterfaceIterator {
        interface: unsafe { libc::if_nameindex() },
    }
}
