use std::ptr::NonNull;

use libc::{
    socklen_t, EACCES, EAFNOSUPPORT, EBADF, EDQUOT, EFAULT, EINTR, EINVAL, EMFILE, ENFILE, ENOBUFS,
    ENOMEM, ENOPROTOOPT, ENOSPC, ENOTSOCK, EPROTONOSUPPORT,
};

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

pub fn socket(socket_family: i32, socket_type: i32, protocol: i32) -> Result<i32, SocketError> {
    match unsafe { libc::socket(socket_family, socket_type, protocol) } {
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
        fd => Ok(fd),
    }
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

pub fn setsockopt(
    socket: i32,
    level: i32,
    name: i32,
    value: NonNull<libc::c_void>,
    option_len: socklen_t,
) -> Result<(), SockOptError> {
    if unsafe { libc::setsockopt(socket, level, name, value.as_ptr(), option_len) } == 0 {
        return Ok(());
    }
    match unsafe { *libc::__errno_location() } {
        EBADF => Err(SockOptError::BadFileDescriptor),
        EFAULT => Err(SockOptError::Fault),
        EINVAL => Err(SockOptError::InvalidValue),
        ENOPROTOOPT => Err(SockOptError::NoProtoOpt),
        ENOTSOCK => Err(SockOptError::NoProtoOpt),
        _ => Err(SockOptError::InvalidError),
    }
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

#[macro_export]
macro_rules! ioctl {
    ($fd: expr, $op: expr, ($($x:expr),+ $(,)?)) => {{
        #[allow(clippy::macro_metavars_in_unsafe, unsafe_code)]
        let value = if unsafe{libc::ioctl($fd, $op, $($x),+)} != -1{
            Ok(())
        }else{
            use $crate::safe_c::IoctlError;
            Err(match unsafe{*libc::__errno_location()}{
                libc::EBADF => IoctlError::BadFileDescriptor,
                libc::EFAULT => IoctlError::InaccessibleMemoryArea,
                libc::EINVAL => IoctlError::InvalidOperation,
                libc::ENOTTY => IoctlError::NotTy,
                _ => IoctlError::InvalidError
            })
        };
        value
    }};
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

pub fn close(fd: i32) -> Result<(), CloseError> {
    if unsafe { libc::close(fd) } == 0 {
        return Ok(());
    }
    match unsafe { *libc::__errno_location() } {
        EBADF => Err(CloseError::FileDescriptorClosedOrInvalid),
        EINTR => Err(CloseError::Interupted),
        ENOSPC | EDQUOT => Err(CloseError::StorageSpaceExceeded),
        _ => Err(CloseError::InvalidError),
    }
}

#[derive(Debug)]
pub enum CError {
    Socket(SocketError),
    SockOpt(SockOptError),
    Ioctl(IoctlError),
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

impl From<CloseError> for CError {
    fn from(value: CloseError) -> Self {
        Self::Close(value)
    }
}
