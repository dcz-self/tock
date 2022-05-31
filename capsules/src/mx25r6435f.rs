//! SyscallDriver for the MX25R6435F flash chip.
//!
//! <http://www.macronix.com/en-us/products/NOR-Flash/Serial-NOR-Flash/Pages/spec.aspx?p=MX25R6435F>
//!
//! From the datasheet:
//!
//! > MX25R6435F is 64Mb bits Serial NOR Flash memory, which is configured as
//! > 8,388,608 x 8 internally. When it is in four I/O mode, the structure
//! > becomes 16,777,216 bits x 4 or 33,554,432 bits x 2. MX25R6435F feature a
//! > serial peripheral interface and software protocol allowing operation on a
//! > simple 3-wire bus while it is in single I/O mode. The three bus signals
//! > are a clock input (SCLK), a serial data input (SI), and a serial data
//! > output (SO). Serial access to the device is enabled by CS# input.
//!
//! Usage
//! -----
//!
//! ```rust
//! # use kernel::static_init;
//! # use capsules::virtual_alarm::VirtualMuxAlarm;
//!
//! // Create a SPI device for this chip.
//! let mx25r6435f_spi = static_init!(
//!     capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52::spi::SPIM>,
//!     capsules::virtual_spi::VirtualSpiMasterDevice::new(mux_spi, &nrf5x::gpio::PORT[17])
//! );
//! // Create an alarm for this chip.
//! let mx25r6435f_virtual_alarm = static_init!(
//!     VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
//!     VirtualMuxAlarm::new(mux_alarm)
//! );
//! mx25r6435f_virtual_alarm.setup();
//!
//! // Setup the actual MX25R6435F driver.
//! let mx25r6435f = static_init!(
//!     capsules::mx25r6435f::MX25R6435F<
//!         'static,
//!         capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52::spi::SPIM>,
//!         nrf5x::gpio::GPIOPin,
//!     >,
//!     capsules::mx25r6435f::MX25R6435F::new(
//!         mx25r6435f_spi,
//!         &mut capsules::mx25r6435f::TXBUFFER,
//!         &mut capsules::mx25r6435f::RXBUFFER,
//!         Some(&nrf5x::gpio::PORT[22]),
//!         Some(&nrf5x::gpio::PORT[23])
//!     )
//! );
//! mx25r6435f_spi.set_client(mx25r6435f);
//! mx25r6435f_virtual_alarm.set_client(mx25r6435f);
//! ```

use core::cell::Cell;
use kernel::debug;
use kernel::hil;
use kernel::hil::block_storage::AddressRange;
use kernel::hil::time::ConvertTicks;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::cells::TakeCell;
use kernel::ErrorCode;

pub static mut TXBUFFER: [u8; PAGE_SIZE + 4] = [0; PAGE_SIZE + 4];
pub static mut RXBUFFER: [u8; PAGE_SIZE + 4] = [0; PAGE_SIZE + 4];

const SPI_SPEED: u32 = 8000000;
pub const SECTOR_SIZE: usize = 4096;
pub const PAGE_SIZE: usize = 256;

// TODO: remove alias
type Mx25r6435fSector = [u8];

#[allow(dead_code)]
enum Opcodes {
    WREN = 0x06, // Write Enable
    WRDI = 0x04, // Write Disable
    SE = 0x20,   // Sector Erase
    READ = 0x03, // Normal Read
    PP = 0x02,   // Page Program (write)
    RDID = 0x9f, // Read Identification
    RDSR = 0x05, // Read Status Register
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    Idle,

    ReadSector {
        page_index: u32,
    },

    EraseSectorWriteEnable {
        sector_index: u32,
    },
    EraseSectorErase,
    EraseSectorCheckDone,
    EraseSectorDone,

    WriteSectorWriteEnable {
        region: WriteRegion,
        /// Offset into region
        block_offset: u32,
    },
    WriteSectorWrite {
        region: WriteRegion,
        /// Offset into region
        block_offset: u32,
    },
    WriteSectorCheckDone {
        region: WriteRegion,
        /// Offset into region
        block_offset: u32,
    },
    WriteSectorWaitDone {
        region: WriteRegion,
        /// Offset into region
        block_offset: u32,
    },

    ReadId,
}

pub struct MX25R6435F<
    'a,
    S: hil::spi::SpiMasterDevice + 'a,
    P: hil::gpio::Pin + 'a,
    A: hil::time::Alarm<'a> + 'a,
> {
    spi: &'a S,
    alarm: &'a A,
    state: Cell<State>,
    write_protect_pin: Option<&'a P>,
    hold_pin: Option<&'a P>,
    txbuffer: TakeCell<'static, [u8]>,
    rxbuffer: TakeCell<'static, [u8]>,
    client: OptionalCell<&'a dyn hil::block_storage::Client<PAGE_SIZE, SECTOR_SIZE>>,
    client_sector: TakeCell<'static, Mx25r6435fSector>,
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice + 'a,
        P: hil::gpio::Pin + 'a,
        A: hil::time::Alarm<'a> + 'a,
    > MX25R6435F<'a, S, P, A>
{
    pub fn new(
        spi: &'a S,
        alarm: &'a A,
        txbuffer: &'static mut [u8],
        rxbuffer: &'static mut [u8],
        write_protect_pin: Option<&'a P>,
        hold_pin: Option<&'a P>,
    ) -> MX25R6435F<'a, S, P, A> {
        MX25R6435F {
            spi: spi,
            alarm: alarm,
            state: Cell::new(State::Idle),
            write_protect_pin: write_protect_pin,
            hold_pin: hold_pin,
            txbuffer: TakeCell::new(txbuffer),
            rxbuffer: TakeCell::new(rxbuffer),
            client: OptionalCell::empty(),
            client_sector: TakeCell::empty(),
        }
    }

    /// Setup SPI for this chip
    fn configure_spi(&self) -> Result<(), ErrorCode> {
        self.hold_pin.map(|pin| {
            pin.set();
        });
        self.spi.configure(
            hil::spi::ClockPolarity::IdleLow,
            hil::spi::ClockPhase::SampleLeading,
            SPI_SPEED,
        )
    }

    /// Requests the readout of a 24-bit identification number.
    /// This command will cause a debug print when succeeded.
    pub fn read_identification(&self) -> Result<(), ErrorCode> {
        self.configure_spi()?;

        self.txbuffer
            .take()
            .map_or(Err(ErrorCode::RESERVE), |txbuffer| {
                self.rxbuffer
                    .take()
                    .map_or(Err(ErrorCode::RESERVE), move |rxbuffer| {
                        txbuffer[0] = Opcodes::RDID as u8;

                        self.state.set(State::ReadId);
                        if let Err((err, txbuffer, rxbuffer)) =
                            self.spi.read_write_bytes(txbuffer, Some(rxbuffer), 4)
                        {
                            self.txbuffer.replace(txbuffer);
                            self.rxbuffer.replace(rxbuffer.unwrap());
                            Err(err)
                        } else {
                            Ok(())
                        }
                    })
            })
    }

    fn enable_write(&self) -> Result<(), ErrorCode> {
        self.write_protect_pin.map(|pin| {
            pin.set();
        });
        self.txbuffer
            .take()
            .map_or(Err(ErrorCode::RESERVE), |txbuffer| {
                txbuffer[0] = Opcodes::WREN as u8;
                if let Err((err, txbuffer, _)) = self.spi.read_write_bytes(txbuffer, None, 1) {
                    self.txbuffer.replace(txbuffer);
                    Err(err)
                } else {
                    Ok(())
                }
            })
    }

    fn erase_sector(&self, sector_index: u32) -> Result<(), ErrorCode> {
        self.configure_spi()?;
        self.state.set(State::EraseSectorWriteEnable {
            sector_index,
        });
        self.enable_write()
    }

    fn read_page(
        &self,
        page_index: u32,
        sector: &'static mut Mx25r6435fSector,
    ) -> Result<(), (ErrorCode, &'static mut Mx25r6435fSector)> {
        match self.configure_spi() {
            Ok(()) => {
                let retval = self
                    .txbuffer
                    .take()
                    .map_or(Err(ErrorCode::RESERVE), |txbuffer| {
                        self.rxbuffer
                            .take()
                            .map_or(Err(ErrorCode::RESERVE), move |rxbuffer| {
                                let address = page_index * PAGE_SIZE as u32;
                                // Setup the read instruction
                                txbuffer[0] = Opcodes::READ as u8;
                                txbuffer[1] = (address >> 16) as u8;
                                txbuffer[2] = (address >> 8) as u8;
                                txbuffer[3] = (address >> 0) as u8;

                                // Call the SPI driver to kick things off.
                                self.state.set(State::ReadSector {
                                    page_index,
                                });
                                if let Err((err, txbuffer, rxbuffer)) = self.spi.read_write_bytes(
                                    txbuffer,
                                    Some(rxbuffer),
                                    (PAGE_SIZE + 4) as usize,
                                ) {
                                    self.txbuffer.replace(txbuffer);
                                    self.rxbuffer.replace(rxbuffer.unwrap());
                                    Err(err)
                                } else {
                                    Ok(())
                                }
                            })
                    });

                match retval {
                    Ok(()) => {
                        self.client_sector.replace(sector);
                        Ok(())
                    }
                    Err(ecode) => Err((ecode, sector)),
                }
            }
            Err(error) => Err((error, sector)),
        }
    }

    fn write_pages(
        &self,
        region: &WriteRegion,
        buffer: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut Mx25r6435fSector)> {
        if region.get_length_bytes() as usize > buffer.len() {
            Err((ErrorCode::SIZE, buffer))
        } else {
            match self.configure_spi() {
                Ok(()) => {
                    self.state.set(State::WriteSectorWriteEnable {
                        region: *region,
                        block_offset: 0,
                    });
                    let retval = self.enable_write();

                    match retval {
                        Ok(()) => {
                            self.client_sector.replace(buffer);
                            Ok(())
                        }
                        Err(ecode) => Err((ecode, buffer)),
                    }
                }
                Err(error) => Err((error, buffer)),
            }
        }
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice + 'a,
        P: hil::gpio::Pin + 'a,
        A: hil::time::Alarm<'a> + 'a,
    > hil::spi::SpiMasterClient for MX25R6435F<'a, S, P, A>
{
    fn read_write_done(
        &self,
        write_buffer: &'static mut [u8],
        read_buffer: Option<&'static mut [u8]>,
        len: usize,
        read_write_status: Result<(), ErrorCode>,
    ) {
        match self.state.get() {
            State::ReadId => {
                self.txbuffer.replace(write_buffer);
                read_buffer.map(|read_buffer| {
                    debug!(
                        "id 0x{:02x}{:02x}{:02x}",
                        read_buffer[1], read_buffer[2], read_buffer[3]
                    );
                    self.rxbuffer.replace(read_buffer);
                });
            }
            State::ReadSector {
                page_index: _
            } => {
                self.client_sector.take().map(|sector| {
                    read_buffer.map(move |read_buffer| {
                        // Copy read in bytes to user page
                        for i in 0..PAGE_SIZE {
                            // Skip the command and address bytes (hence the +4).
                            sector[i] = read_buffer[i + 4];
                        }

                        self.state.set(State::Idle);
                        self.txbuffer.replace(write_buffer);
                        self.rxbuffer.replace(read_buffer);

                        self.client.map(move |client| {
                            client.read_complete(sector, Ok(()));
                        });
                    });
                });
            }
            State::EraseSectorWriteEnable {
                sector_index,
            } => {
                self.state.set(State::EraseSectorErase);
                let address = sector_index * SECTOR_SIZE as u32;
                write_buffer[0] = Opcodes::SE as u8;
                write_buffer[1] = (address >> 16) as u8;
                write_buffer[2] = (address >> 8) as u8;
                write_buffer[3] = (address >> 0) as u8;

                // TODO verify SPI return value
                let _ = self.spi.read_write_bytes(write_buffer, None, 4);
            }
            State::EraseSectorErase => {
                self.state.set(State::EraseSectorCheckDone);
                self.txbuffer.replace(write_buffer);
                // Datasheet says erase takes 58 ms on average. So we wait that
                // long.
                let delay = self.alarm.ticks_from_ms(58);
                self.alarm.set_alarm(self.alarm.now(), delay);
            }
            State::EraseSectorCheckDone => {
                read_buffer.map(move |read_buffer| {
                    let status = read_buffer[1];

                    // Check the status byte to see if the erase is done or not.
                    if status & 0x01 == 0x01 {
                        // Erase is still in progress.
                        let _ = self
                            .spi
                            .read_write_bytes(write_buffer, Some(read_buffer), 2);
                    } else {
                        // Erase has finished, so jump to the next state.
                        self.state.set(State::EraseSectorDone);
                        self.rxbuffer.replace(read_buffer);
                        self.read_write_done(write_buffer, None, len, read_write_status);
                    }
                });
            }
            State::EraseSectorDone => {
                // No need to disable write, chip does it automatically.
                self.state.set(State::Idle);
                self.txbuffer.replace(write_buffer);
                self.client.map(|client| {
                    client.erase_complete(Ok(()));
                });
            }
            State::WriteSectorWriteEnable { region, block_offset } => {
                if region.length_blocks == block_offset {
                    // Finished
                    self.state.set(State::Idle);
                    self.txbuffer.replace(write_buffer);
                    self.client.map(|client| {
                        self.client_sector.take().map(|sector| {
                            client.write_complete(sector, Ok(()));
                        });
                    });
                } else {
                    self.state.set(State::WriteSectorWrite {
                        region, block_offset,
                    });
                    // Need to write enable before each PP
                    write_buffer[0] = Opcodes::WREN as u8;
                    // TODO verify SPI return value
                    let _ = self.spi.read_write_bytes(write_buffer, None, 1);
                }
            }
            State::WriteSectorWrite { region, block_offset } => {
                self.state.set(State::WriteSectorCheckDone{ region, block_offset });
                let address = u64::from(region.index + block_offset) as u32;
                write_buffer[0] = Opcodes::PP as u8;
                write_buffer[1] = (address >> 16) as u8;
                write_buffer[2] = (address >> 8) as u8;
                write_buffer[3] = (address >> 0) as u8;

                self.client_sector.map(|sector| {
                    for i in 0..PAGE_SIZE {
                        write_buffer[i + 4] = sector[(block_offset as usize * PAGE_SIZE)..][i];
                    }
                });

                let _ = self
                    .spi
                    .read_write_bytes(write_buffer, None, PAGE_SIZE);
            }
            State::WriteSectorCheckDone { region, block_offset } => {
                self.state.set(State::WriteSectorWaitDone { region, block_offset });
                self.txbuffer.replace(write_buffer);
                // Datasheet says write page takes 3.2 ms on average. So we wait
                // that long.
                let delay = self.alarm.ticks_from_us(3200);
                self.alarm.set_alarm(self.alarm.now(), delay);
            }
            State::WriteSectorWaitDone { region, block_offset } => {
                read_buffer.map(move |read_buffer| {
                    let status = read_buffer[1];

                    // Check the status byte to see if the write is done or not.
                    if status & 0x01 == 0x01 {
                        // Write is still in progress.
                        let _ = self
                            .spi
                            .read_write_bytes(write_buffer, Some(read_buffer), 2);
                    } else {
                        // Write has finished, so go back to writing.
                        self.state.set(State::WriteSectorWriteEnable{
                            region,
                            block_offset: block_offset + 1,
                        });
                        self.rxbuffer.replace(read_buffer);
                        self.read_write_done(write_buffer, None, len, read_write_status);
                    }
                });
            }
            _ => {}
        }
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice + 'a,
        P: hil::gpio::Pin + 'a,
        A: hil::time::Alarm<'a> + 'a,
    > hil::time::AlarmClient for MX25R6435F<'a, S, P, A>
{
    fn alarm(&self) {
        // After the timer expires we still have to check that the erase/write
        // operation has finished.
        self.txbuffer.take().map(|write_buffer| {
            self.rxbuffer.take().map(move |read_buffer| {
                write_buffer[0] = Opcodes::RDSR as u8;
                let _ = self
                    .spi
                    .read_write_bytes(write_buffer, Some(read_buffer), 2);
            });
        });
    }
}

pub type WriteRegion = hil::block_storage::Region<PAGE_SIZE>;
pub type EraseRegion = hil::block_storage::Region<SECTOR_SIZE>;

impl<
        'a,
        S: hil::spi::SpiMasterDevice + 'a,
        P: hil::gpio::Pin + 'a,
        A: hil::time::Alarm<'a> + 'a,
    > hil::block_storage::BlockStorage<PAGE_SIZE, SECTOR_SIZE> for MX25R6435F<'a, S, P, A>
{
    fn read_range(
        &self,
        _range: &AddressRange,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        Err((ErrorCode::NOSUPPORT, buf))
    }

    fn read(
        &self,
        region: &WriteRegion,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        let errs = if buf.len() < region.get_length_bytes() as usize {
            Err(ErrorCode::INVAL)
        } else if AddressRange::from(*region).get_end_address() > self.get_size().unwrap() {
            Err(ErrorCode::INVAL)
        } else if region.length_blocks != 1 {
            Err(ErrorCode::NOSUPPORT)
        } else {
            Ok(())
        };

        match errs {
            Ok(()) => self.read_page(region.index.0, buf),
            Err(e) => Err((e, buf)),
        }
    }

    fn write(
        &self,
        region: &WriteRegion,
        buf: &'static mut [u8],
    ) -> Result<(), (ErrorCode, &'static mut [u8])> {
        let errs = if buf.len() < region.get_length_bytes() as usize {
            Err(ErrorCode::INVAL)
        } else if AddressRange::from(*region).get_end_address() > self.get_size().unwrap() {
            Err(ErrorCode::INVAL)
        } else if region.length_blocks != 1 {
            Err(ErrorCode::NOSUPPORT)
        } else {
            Ok(())
        };

        match errs {
            Ok(()) => self.write_pages(region, buf),
            Err(e) => Err((e, buf)),
        }
    }

    fn erase(&self, region: &EraseRegion) -> Result<(), ErrorCode> {
        if AddressRange::from(*region).get_end_address() > self.get_size().unwrap() {
            Err(ErrorCode::INVAL)
        } else if region.length_blocks != 1 {
            Err(ErrorCode::NOSUPPORT)
        } else {
            self.erase_sector(region.index.0)
        }
    }

    /// Returns the size of the device in bytes.
    fn get_size(&self) -> Result<u64, ErrorCode> {
        // TODO: it's probably a good idea to discover the size in advance,
        // for the sake of compatible devices.
        Ok(8 * 1024 * 1024)
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice + 'a,
        P: hil::gpio::Pin + 'a,
        A: hil::time::Alarm<'a> + 'a,
        C: hil::block_storage::Client<PAGE_SIZE, SECTOR_SIZE>,
    > hil::block_storage::HasClient<'a, C> for MX25R6435F<'a, S, P, A>
{
    fn set_client(&self, client: &'a C) {
        self.client.set(client);
    }
}
