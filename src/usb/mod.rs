//! USB Device support

use crate::gpio;

pub use usb_device;

mod bus;
pub use self::bus::UsbBus;

mod devicedesc;
use self::devicedesc::Descriptors;

/// USB D-
pub type DmPad = gpio::Pb10<gpio::Sf>;

/// USB D+
pub type DpPad = gpio::Pb11<gpio::Sf>;
