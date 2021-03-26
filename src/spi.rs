//! SPI Implementation
use crate::clock::{get_master_clock_frequency, Enabled, SpiClock};
use crate::gpio::{Pa12, Pa13, Pa14, PfA};
use crate::pac::SPI;
use core::marker::PhantomData;
use embedded_hal::spi;
use embedded_time::rate::Hertz;

/// SPI Error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Underrun occurred (slave mode only)
    Underrun,
    /// Mode fault occurred
    ModeFault,
    /// SPI Disabled
    SpiDisabled,
    /// Invalid Chip Select
    InvalidCs(u8),
    /// Fixed Mode Set
    FixedModeSet,
    /// Variable Mode Set
    VariableModeSet,
    /// PCS read unexpected (data, pcs)
    UnexpectedPcs(u16, u8),
}

/// Chip Select Active Settings
/// This enum controls:
///  CNSAAT -> Chip Select Not Active After Transfer
///  CSAAT -> Chip Select Active After Transfer
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChipSelectActive {
    /// csaat = 1, csnaat = 0
    ActiveAfterTransfer,
    /// csaat = 0, csnaat = 0
    ActiveOnConsecutiveTransfers,
    /// csaat = 0, csnaat = 1
    InactiveAfterEachTransfer,
}

/// Transfer Width
/// NOTE: Transfer Widths larger than 8-bits require using 16-bit with send/read
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BitWidth {
    Width8Bit = 0,
    Width9Bit = 1,
    Width10Bit = 2,
    Width11Bit = 3,
    Width12Bit = 4,
    Width13Bit = 5,
    Width14Bit = 6,
    Width15Bit = 7,
    Width16Bit = 8,
}

/// Peripheral Select Mode
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PeripheralSelectMode {
    /// Fixed Peripheral Select Mode (ps = 0, pcsdec = 0)
    Fixed,
    /// Variable Peripheral Select Mode (ps = 1, pcsdec = 0)
    Variable,
    /// Chip Select Decode (Variable) (ps = 1, pcsdec = 1)
    ChipSelectDecode,
}

/// SPI Chip Select Settings
///
/// CPOL -> MODE
/// NCPHA -> MODE
/// CSNAAT -> CS not active after transfer (ignored if CSAAT = 1) => csa
/// CSAAT -> CS active after transfer => csa
/// BITS -> 8bit through 16bits
/// SCBR -> Serial clock rate (SCBR = f_periph / SPCK bit rate) (0 forbidden)
/// DLYBS -> Delay before SPCK (DLYBS x f_periph)
/// DLYBCT -> Delay between consecutive transfers (DLYBCT x f_periph / 32)
#[derive(Clone, PartialEq, Eq)]
pub struct ChipSelectSettings {
    mode: spi::Mode,
    csa: ChipSelectActive,
    scbr: u8,
    dlybs: u8,
    dlybct: u8,
    bits: BitWidth,
}

impl ChipSelectSettings {
    /// mode:   SPI Mode
    /// csa:    Chip Select behaviour after transfer
    /// bits:   SPI bit width
    /// baud:   SPI speed in Hertz
    /// dlybs:  Cycles to delay from CS to first valid SPCK
    ///         0 is half the SPCK clock period
    ///         Otherwise dlybs = Delay Before SPCK x f_periph
    /// dlybct: Cycles to delay between consecutive transfers
    ///         0 is no delay
    ///         Otherwise dlybct = Delay between consecutive transfers x f_periph / 32
    pub fn new(
        mode: spi::Mode,
        csa: ChipSelectActive,
        bits: BitWidth,
        baud: Hertz,
        dlybs: u8,
        dlybct: u8,
    ) -> ChipSelectSettings {
        let pclk = get_master_clock_frequency();

        // Calculate baud divider
        // (f_periph + baud - 1) / baud
        // TODO: Assert if scbr is less than 1
        let scbr = ((pclk.0 + baud.0 - 1) / baud.0) as u8;

        ChipSelectSettings {
            mode,
            csa,
            scbr,
            dlybs,
            dlybct,
            bits,
        }
    }
}

pub struct SpiMaster {
    spi: SPI,
    clock: PhantomData<SpiClock<Enabled>>,
    miso: PhantomData<Pa12<PfA>>,
    mosi: PhantomData<Pa13<PfA>>,
    spck: PhantomData<Pa14<PfA>>,
    cs: u8,
    lastxfer: bool,
}

impl SpiMaster {
    /// Initialize SPI as Master
    /// PSM - Peripheral Select Mode
    /// WDRBT - Wait Data Read Before Transfer Enabled
    /// LLB - Local Loopback
    /// DLYBCS - Delay between chip selects = DLYBCS / f_periph
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        spi: SPI,
        _clock: SpiClock<Enabled>,
        _miso: Pa12<PfA>,
        _mosi: Pa13<PfA>,
        _spck: Pa14<PfA>,
        psm: PeripheralSelectMode,
        wdrbt: bool,
        llb: bool,
        dlybcs: u8,
    ) -> SpiMaster {
        // Disable SPI
        spi.cr.write_with_zero(|w| w.spidis().set_bit());

        // Software reset SPI (this will reset SPI into Slave Mode)
        spi.cr.write_with_zero(|w| w.swrst().set_bit());

        // Enable SPI
        spi.cr.write_with_zero(|w| w.spien().set_bit());

        // Determine peripheral select mode
        let (ps, pcsdec) = match psm {
            PeripheralSelectMode::Fixed => (false, false),
            PeripheralSelectMode::Variable => (true, false),
            PeripheralSelectMode::ChipSelectDecode => (true, true),
        };

        // Setup SPI Master
        // Master Mode
        // Variable Peripheral Select (more flexible and less initial options to set)
        // Mode Fault Detection Enabled
        spi.mr.write_with_zero(|w| unsafe {
            w.mstr()
                .set_bit()
                .ps()
                .bit(ps)
                .pcsdec()
                .bit(pcsdec)
                .modfdis()
                .clear_bit()
                .wdrbt()
                .bit(wdrbt)
                .llb()
                .bit(llb)
                .dlybcs()
                .bits(dlybcs) // unsafe
        });

        SpiMaster {
            spi,
            clock: PhantomData,
            miso: PhantomData,
            mosi: PhantomData,
            spck: PhantomData,
            cs: 0,           // Default to NPCS0
            lastxfer: false, // Reset to false on each call to send()
        }
    }

    /// Apply settings to a specific channel
    /// Uses cs 0..3 for spi channel settings
    /// When using pcsdec (Chip Decode Select)
    ///  csr0 -> 0..3
    ///  csr1 -> 4..7
    ///  csr2 -> 8..11
    ///  csr3 -> 12..14
    pub fn cs_setup(&mut self, cs: u8, settings: ChipSelectSettings) -> Result<(), Error> {
        // Lookup cs when using pcsdec
        let cs = if self.spi.mr.read().pcsdec().bit_is_set() {
            match cs {
                0..=3 => 0,
                4..=7 => 1,
                8..=11 => 2,
                12..=14 => 3,
                _ => {
                    return Err(Error::InvalidCs(cs));
                }
            }

        // Otherwise validate the cs
        } else if cs > 3 {
            return Err(Error::InvalidCs(cs));
        } else {
            cs
        };

        let cpol = match settings.mode.polarity {
            spi::Polarity::IdleLow => false,
            spi::Polarity::IdleHigh => true,
        };
        let ncpha = match settings.mode.phase {
            spi::Phase::CaptureOnFirstTransition => true,
            spi::Phase::CaptureOnSecondTransition => false,
        };
        let (csaat, csnaat) = match settings.csa {
            ChipSelectActive::ActiveAfterTransfer => (true, false),
            ChipSelectActive::ActiveOnConsecutiveTransfers => (false, false),
            ChipSelectActive::InactiveAfterEachTransfer => (false, true),
        };
        self.spi.csr[cs as usize].write_with_zero(|w| unsafe {
            w.cpol()
                .bit(cpol)
                .ncpha()
                .bit(ncpha)
                .csnaat()
                .bit(csnaat)
                .csaat()
                .bit(csaat)
                .bits_()
                .bits(settings.bits as u8) // unsafe (can be made safe with some work)
                .scbr()
                .bits(settings.scbr) // unsafe
                .dlybs()
                .bits(settings.dlybs) // unsafe
                .dlybct()
                .bits(settings.dlybct) // unsafe
        });

        Ok(())
    }

    /// Select ChipSelect for next read/write FullDuplex trait functions
    /// Works around limitations in the embedded-hal trait
    /// Valid cs:
    ///  0 -> 3 (as long as NPCS0..3 are configured)
    ///  0 -> 15 (uses NPCS0..3 as the input to a 4 to 16 mux), pcsdec must be enabled
    pub fn cs_select(&mut self, cs: u8) -> Result<(), Error> {
        // Map cs to id
        let pcs_id = match cs {
            0 => 0b0000, // xxx0 => NPCS[3:0] = 1110
            1 => 0b0001, // xx01 => NPCS[3:0] = 1101
            2 => 0b0011, // x011 => NPCS[3:0] = 1011
            3 => 0b0111, // 0111 => NPCS[3:0] = 0111
            _ => 0b1111, // Forbidden
        };

        // Fixed mode
        if self.spi.mr.read().ps().bit_is_clear() {
            self.spi.mr.modify(|_, w| unsafe { w.pcs().bits(pcs_id) });

        // Variable Mode
        } else {
            // Check for pcsdec
            if self.spi.mr.read().pcsdec().bit_is_set() {
                if cs > 15 {
                    return Err(Error::InvalidCs(cs));
                }
                self.cs = cs;
            } else {
                if cs > 3 {
                    return Err(Error::InvalidCs(cs));
                }
                // Map cs to id
                self.cs = pcs_id;
            }
        }
        Ok(())
    }

    /// lastxfer set
    /// Fixed Mode
    ///  Sets lastxfer register
    /// Variable Mode
    ///  Use to set lastxfer for the next call to send()
    pub fn lastxfer(&mut self, lastxfer: bool) {
        // Fixed mode
        if self.spi.mr.read().ps().bit_is_clear() {
            self.spi.cr.write_with_zero(|w| w.lastxfer().set_bit());
        // Variable Mode
        } else {
            self.lastxfer = lastxfer;
        }
    }
}

impl spi::FullDuplex<u8> for SpiMaster {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let sr = self.spi.sr.read();

        // Check for errors (return error)
        // Check for data to read (and read it)
        // Return WouldBlock if no data available
        Err(if sr.ovres().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().bit_is_set() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.spiens().bit_is_clear() {
            nb::Error::Other(Error::SpiDisabled)
        } else if sr.rdrf().bit_is_set() {
            let rdr = self.spi.rdr.read();

            // In variable mode, verify pcs is what we expect
            if self.spi.mr.read().ps().bit_is_set() && rdr.pcs() != self.cs {
                nb::Error::Other(Error::UnexpectedPcs(rdr.rd().bits(), rdr.pcs().bits()))
            } else {
                return Ok(rdr.rd().bits() as u8);
            }
        } else {
            nb::Error::WouldBlock
        })
    }

    fn send(&mut self, byte: u8) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        // Check for errors (return error)
        // Make sure buffer is empty (then write if available)
        // Return WouldBlock if buffer is full
        Err(if sr.ovres().bit_is_set() {
            nb::Error::Other(Error::Overrun)
        } else if sr.modf().bit_is_set() {
            nb::Error::Other(Error::ModeFault)
        } else if sr.spiens().bit_is_clear() {
            nb::Error::Other(Error::SpiDisabled)
        } else if sr.tdre().bit_is_set() {
            // Fixed Mode
            if self.spi.mr.read().ps().bit_is_clear() {
                self.spi
                    .tdr
                    .write_with_zero(|w| unsafe { w.td().bits(byte as u16) });

            // Variable Mode
            } else {
                // NOTE: Uses self.cs to write the pcs register field
                self.spi.tdr.write_with_zero(|w| unsafe {
                    w.td()
                        .bits(byte as u16)
                        .pcs()
                        .bits(self.cs)
                        .lastxfer()
                        .bit(self.lastxfer)
                });
            }
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
}
