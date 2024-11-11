#![warn(
    clippy::todo,
    clippy::missing_const_for_fn,
    clippy::pedantic,
    clippy::use_self
)]
#![expect(
    clippy::doc_markdown,
    clippy::too_many_lines,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap,
    clippy::module_name_repetitions,
    clippy::must_use_candidate,
    dead_code
)]

pub mod ethercat;
pub mod osal;
pub mod oshw;
#[expect(unsafe_code)]
pub mod safe_c;
