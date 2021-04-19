//! Two-wire Interface (TWI) (Atmel I2C)
//! Loosely based off of atsamd-hal SERCOM I2C v1
// TODO Handle multiple TWI modules (e.g. TWI1 and TWI2)
// TODO Handle different pins
use crate::clock::{get_master_clock_frequency, Enabled, Twi0Clock};
use crate::gpio::{Pa3, Pa4, PfA};
use crate::pac::TWI0;
use core::marker::PhantomData;
use embedded_time::rate::Hertz;
use hal::blocking::i2c::{Read, Write, WriteRead};

/// I2C error
#[derive(Debug)]
pub enum Error {
    AddressError,
    ArbitrationLost,
    Nack,
    Overrun,
}

/// TWI Master Mode
pub struct TwiMaster {
    twi: TWI0,
    twd: PhantomData<Pa3<PfA>>,
    twck: PhantomData<Pa4<PfA>>,
}

// TODO
// - Error detection
// - Error recovery
// http://ww1.microchip.com/downloads/en/DeviceDoc/90003181A.pdf

/// (ckdiv, chdiv, cldiv)
fn clk_dividers(baud: Hertz) -> (u8, u8, u8) {
    let pclk = get_master_clock_frequency();
    // t_low  = ((CLDIV x 2^CKDIV) + 4 x t_periph
    // t_high = ((CHDIV x 2^CKDIV) + 4 x t_periph

    // - Calculations taken from ASF -
    let low_level_time_limit = Hertz(384000);
    let twi_clk_divider = 2;
    let twi_clk_calc_argu = 4;
    let twi_clk_div_max = 0xFF;
    let twi_clk_div_min = 7;

    // Low level time not less than 1.3us of I2C Fast Mode.
    let mut ckdiv: u8 = 0;
    if baud > low_level_time_limit {
        // Low level of time fixed for 1.3us.
        let mut cldiv = pclk.0 / (low_level_time_limit.0 * twi_clk_divider) - twi_clk_calc_argu;
        let mut chdiv = pclk.0 / ((baud.0 + (baud.0 - low_level_time_limit.0)) * twi_clk_divider)
            - twi_clk_calc_argu;

        // cldiv must fit in 8 bits, ckdiv must fit in 3 bits
        while (cldiv > twi_clk_div_max) && (ckdiv < twi_clk_div_min) {
            // Increase clock divider
            ckdiv += 1;
            // Divide cldiv value
            cldiv /= twi_clk_divider;
        }
        // chdiv must fit in 8 bits, ckdiv must fit in 3 bits
        while (chdiv > twi_clk_div_max) && (ckdiv < twi_clk_div_min) {
            // Increase clock divider
            ckdiv += 1;
            // Divide cldiv value
            chdiv /= twi_clk_divider;
        }

        (ckdiv, chdiv as u8, cldiv as u8)
    } else {
        let mut c_lh_div = pclk.0 / (baud.0 * twi_clk_divider) - twi_clk_calc_argu;

        // cldiv must fit in 8 bits, ckdiv must fit in 3 bits
        while (c_lh_div > twi_clk_div_max) && (ckdiv < twi_clk_div_min) {
            // Increase clock divider
            ckdiv += 1;
            // Divide cldiv value
            c_lh_div /= twi_clk_divider;
        }

        (ckdiv, c_lh_div as u8, c_lh_div as u8)
    }
}

impl TwiMaster {
    /// Initialize TWI as master
    pub fn new(
        twi: TWI0,
        _clock: Twi0Clock<Enabled>,
        _twd: Pa3<PfA>,
        _twck: Pa4<PfA>,
        baud: Hertz,
    ) -> TwiMaster {
        // Reset TWI
        twi.cr.write_with_zero(|w| w.swrst().set_bit());

        // Setup TWI/I2C Clock
        let (ckdiv, chdiv, cldiv) = clk_dividers(baud);
        twi.cwgr.write_with_zero(|w| unsafe {
            w.ckdiv()
                .bits(ckdiv)
                .chdiv()
                .bits(chdiv)
                .cldiv()
                .bits(cldiv)
        });

        // Disable slave and master modes
        twi.cr
            .write_with_zero(|w| w.msdis().set_bit().svdis().set_bit());

        // Enable master mode CR
        twi.cr.write_with_zero(|w| w.msen().set_bit());

        TwiMaster {
            twi,
            twd: PhantomData,
            twck: PhantomData,
        }
    }

    pub fn set_baud(&mut self, baud: Hertz) {
        let (ckdiv, chdiv, cldiv) = clk_dividers(baud);
        self.twi.cwgr.write_with_zero(|w| unsafe {
            w.ckdiv()
                .bits(ckdiv)
                .chdiv()
                .bits(chdiv)
                .cldiv()
                .bits(cldiv)
        });
    }

    fn start_tx_write(&mut self, addr: u8) -> Result<(), Error> {
        // Wait for idle
        // TODO Is this the best way to check?
        while !self.twi.sr.read().txcomp().bit_is_set() {}

        // Set address
        self.twi
            .mmr
            .modify(|_, w| unsafe { w.mread().clear_bit().dadr().bits(addr) });

        // Generate start condition
        self.twi.cr.write_with_zero(|w| w.start().set_bit());

        // Wait for transmission to complete
        while !self.twi.sr.read().txrdy().bit_is_set() {}

        self.status_to_err()
    }

    fn status_to_err(&mut self) -> Result<(), Error> {
        let status = self.twi.sr.read();
        if status.arblst().bit_is_set() {
            return Err(Error::ArbitrationLost);
        }
        if status.ovre().bit_is_set() {
            return Err(Error::Overrun);
        }
        if status.nack().bit_is_set() {
            return Err(Error::Nack);
        }

        Ok(())
    }

    fn start_tx_read(&mut self, addr: u8) -> Result<(), Error> {
        // Wait for idle
        // TODO Is this the best way to check?
        while !self.twi.sr.read().txcomp().bit_is_set() {}

        // Set address
        self.twi
            .mmr
            .modify(|_, w| unsafe { w.mread().set_bit().dadr().bits(addr) });

        // Generate start condition
        self.twi.cr.write_with_zero(|w| w.start().set_bit());

        // Wait for transmission to complete
        loop {
            let sr = self.twi.sr.read();
            // Check for arbitration
            if sr.arblst().bit_is_set() {
                return Err(Error::ArbitrationLost);
            }
            if sr.txrdy().bit_is_set() {
                break;
            }
        }

        self.status_to_err()
    }

    fn stop(&mut self) {
        self.twi.cr.write_with_zero(|w| w.stop().set_bit());
    }

    fn send_bytes(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for b in bytes {
            self.twi
                .thr
                .write_with_zero(|w| unsafe { w.txdata().bits(*b) });

            // TODO is this sufficient?
            loop {
                let sr = self.twi.sr.read();
                if sr.txrdy().bit_is_set() {
                    break;
                }
            }

            self.status_to_err()?;
        }
        Ok(())
    }

    fn read_one(&mut self) -> u8 {
        while !self.twi.sr.read().rxrdy().bit_is_set() {}
        self.twi.rhr.read().rxdata().bits()
    }

    fn fill_buffer(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        // Some manual iterator gumph because we need to ack bytes after the first.
        let mut iter = buffer.iter_mut();
        *iter.next().expect("buffer len is at least 1") = self.read_one();

        // TODO Make sure we don't have to send ACK/NACK (atsam4 TWI)
        loop {
            match iter.next() {
                None => break,
                Some(dest) => {
                    *dest = self.read_one();
                }
            }
        }

        // TODO - Can we even send a NACK?
        // arrange to send nack on next command to
        // stop slave from transmitting more data
        //self.i2cm().ctrlb.modify(|_, w| w.ackact().set_bit());

        Ok(())
    }

    fn do_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        self.start_tx_write(addr)?;
        self.send_bytes(bytes)
    }

    fn do_read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        self.start_tx_read(addr)?;
        self.fill_buffer(buffer)
    }

    fn do_write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        self.start_tx_write(addr)?;
        self.send_bytes(bytes)?;
        self.start_tx_read(addr)?;
        self.fill_buffer(buffer)
    }
}

impl Write for TwiMaster {
    type Error = Error;

    /// Sends bytes to slave with address `addr`
    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Self::Error> {
        let res = self.do_write(addr, bytes);
        self.stop();
        res
    }
}

impl Read for TwiMaster {
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Self::Error> {
        let res = self.do_read(addr, buffer);
        self.stop();
        res
    }
}

impl WriteRead for TwiMaster {
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Self::Error> {
        let res = self.do_write_read(addr, bytes, buffer);
        self.stop();
        res
    }
}
