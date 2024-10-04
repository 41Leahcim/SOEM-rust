use std::{any::Any, time::Duration};

use crate::oshw::nicdrv::Port;

pub fn setup_datagram(
    port: &mut Port,
    frame: &mut [Box<dyn Any>],
    com: u8,
    index: u8,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
) -> i32 {
    todo!()
}

pub fn add_datagram(
    port: &mut Port,
    frame: &mut [Box<dyn Any>],
    com: u8,
    index: u8,
    more: bool,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
) -> u16 {
    todo!()
}

pub fn bwr(
    port: &mut Port,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn brd(
    port: &mut Port,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn aprd(
    port: &mut Port,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn armw(
    port: &mut Port,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn frmw(
    port: &mut Port,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn aprdw(port: &mut Port, adp: u16, ado: u16, timeout: Duration) -> u16 {
    todo!()
}

pub fn fprd(
    port: &mut Port,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn fprdw(port: &mut Port, adp: u16, ado: u16, timeout: Duration) -> u16 {
    todo!()
}

pub fn apwrw(port: &mut Port, adp: u16, ado: u16, data: u16, timeout: Duration) -> i32 {
    todo!()
}

pub fn apwr(
    port: &mut Port,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn fpwrw(port: &mut Port, adp: u16, ado: u16, data: u16, timeout: Duration) -> i32 {
    todo!()
}

pub fn fpwr(
    port: &mut Port,
    adp: u16,
    ado: u16,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn lrw(
    port: &mut Port,
    logical_address: u32,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn lrd(
    port: &mut Port,
    logical_address: u32,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn lwr(
    port: &mut Port,
    logical_address: u32,
    data: &mut [Box<dyn Any>],
    timeout: Duration,
) -> i32 {
    todo!()
}

pub fn lrwdc(
    port: &mut Port,
    logical_address: u32,
    data: &mut [Box<dyn Any>],
    dc_rs: u16,
    dc_time: &mut [i64],
    timeout: Duration,
) -> i32 {
    todo!()
}

#[cfg(feature = "ec_ver1")]
pub mod ec_ver1 {
    use std::{any::Any, time::Duration};

    pub fn setup_datagram(
        frame: &mut [Box<dyn Any>],
        com: u8,
        index: u8,
        adp: u16,
        ado: u16,
        data: &mut [Box<dyn Any>],
    ) -> i32 {
        todo!()
    }

    pub fn add_datagram(
        frame: &mut [Box<dyn Any>],
        com: u8,
        index: u8,
        more: bool,
        adp: u16,
        ado: u16,
        data: &mut [Box<dyn Any>],
    ) -> u16 {
        todo!()
    }

    pub fn bwr(adp: u16, ado: u16, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn brd(adp: u16, ado: u16, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn aprd(adp: u16, ado: u16, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn frmw(adp: u16, ado: u16, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn aprdw(adp: u16, ado: u16, timeout: Duration) -> u16 {
        todo!()
    }

    pub fn fprd(adp: u16, ado: u16, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn fprdw(adp: u16, ado: u16, timeout: Duration) -> u16 {
        todo!()
    }

    pub fn apwrw(adp: u16, ado: u16, data: u16, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn apwr(adp: u16, ado: u16, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn fpwrw(adp: u16, ado: u16, data: u16, timeout: Duration) -> i32 {
        todo!()
    }

    pub fn fpwr(adp: u16, ado: u16, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn lrw(logical_address: u32, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn lrd(logical_address: u32, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn lwr(logical_address: u32, data: &mut [Box<dyn Any>], timeout: Duration) -> i32 {
        todo!()
    }

    pub fn lrw_dc(
        logical_address: u32,
        data: &mut [Box<dyn Any>],
        dc_rs: u16,
        dc_time: &mut [i64],
        timeout: Duration,
    ) -> i32 {
        todo!()
    }
}
