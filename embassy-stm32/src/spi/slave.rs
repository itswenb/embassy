use core::ptr;

use core::marker::PhantomData;
use embassy_embedded_hal::SetConfig;
use embassy_futures::join::join;
use embassy_hal_internal::{into_ref, PeripheralRef};
use embedded_hal_02::spi::{Mode, Phase, Polarity, MODE_0};
use embedded_hal_nb::nb;

use super::{finish_dma, set_rxdmaen, set_txdmaen, flush_rx_fifo, RxDma, TxDma};
use super::{
    rx_ready, tx_ready, word_impl, BitOrder, CsPin, Error, Info, Instance, MisoPin, MosiPin, RegsExt, SckPin,
    SealedWord, Word,
};
use crate::dma::ChannelAndRequest;
use crate::gpio::{AfType, AnyPin, OutputType, Pull, SealedPin as _, Speed};
use crate::pac::spi::{vals, Spi as Regs};
use crate::{rcc, Peripheral};
use crate::mode::{Async, Blocking, Mode as PeriMode};

/// SPI slave configuration.
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// SPI mode.
    pub mode: Mode,
    /// Bit order.
    pub bit_order: BitOrder,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: BitOrder::MsbFirst,
        }
    }
}

impl Config {
    fn raw_phase(&self) -> vals::Cpha {
        match self.mode.phase {
            Phase::CaptureOnSecondTransition => vals::Cpha::SECONDEDGE,
            Phase::CaptureOnFirstTransition => vals::Cpha::FIRSTEDGE,
        }
    }

    fn raw_polarity(&self) -> vals::Cpol {
        match self.mode.polarity {
            Polarity::IdleHigh => vals::Cpol::IDLEHIGH,
            Polarity::IdleLow => vals::Cpol::IDLELOW,
        }
    }

    fn raw_byte_order(&self) -> vals::Lsbfirst {
        match self.bit_order {
            BitOrder::LsbFirst => vals::Lsbfirst::LSBFIRST,
            BitOrder::MsbFirst => vals::Lsbfirst::MSBFIRST,
        }
    }
}

/// SPI slave driver.
///
/// This driver provides blocking software-driven read and write methods. The driver can be turned
/// into an asynchronous one by providing DMA peripherals using `Self::dma_ringbuffered`.
///
/// For SPI buses with high-frequency clocks you must use the asynchronous driver, as the chip is
/// not fast enough to drive the SPI in software.
pub struct SpiSlave<'d, M: PeriMode> {
    pub(crate) info: &'static Info,
    sck: Option<PeripheralRef<'d, AnyPin>>,
    mosi: Option<PeripheralRef<'d, AnyPin>>,
    miso: Option<PeripheralRef<'d, AnyPin>>,
    cs: Option<PeripheralRef<'d, AnyPin>>,
    tx_dma: Option<ChannelAndRequest<'d>>,
    rx_dma: Option<ChannelAndRequest<'d>>,
    _phantom: PhantomData<M>,
    current_word_size: word_impl::Config,
}

impl<'d, M: PeriMode> SpiSlave<'d, M> {
    fn new_inner<T: Instance>(
        _peri: impl Peripheral<P = T> + 'd,
        sck: Option<PeripheralRef<'d, AnyPin>>,
        mosi: Option<PeripheralRef<'d, AnyPin>>,
        miso: Option<PeripheralRef<'d, AnyPin>>,
        cs: Option<PeripheralRef<'d, AnyPin>>,
        tx_dma: Option<ChannelAndRequest<'d>>,
        rx_dma: Option<ChannelAndRequest<'d>>,
        config: Config,
    ) -> Self {
        let cpha = config.raw_phase();
        let cpol = config.raw_polarity();

        let lsbfirst = config.raw_byte_order();

        rcc::enable_and_reset::<T>();

        let info = T::info();
        let regs = info.regs;

        #[cfg(any(spi_v1, spi_f1))]
        {
            regs.cr1().modify(|w| {
                w.set_cpha(cpha);
                w.set_cpol(cpol);

                w.set_mstr(vals::Mstr::SLAVE);
                w.set_ssm(cs.is_none());

                w.set_lsbfirst(lsbfirst);
                w.set_crcen(false);
                w.set_bidimode(vals::Bidimode::UNIDIRECTIONAL);
                if miso.is_none() {
                    w.set_rxonly(vals::Rxonly::OUTPUTDISABLED);
                }
                w.set_dff(<u8 as SealedWord>::CONFIG)
            });
        }
        #[cfg(spi_v2)]
        {
            regs.cr2().modify(|w| {
                let (ds, frxth) = <u8 as SealedWord>::CONFIG;
                w.set_frxth(frxth);
                w.set_ds(ds);
            });
            regs.cr1().modify(|w| {
                w.set_cpha(cpha);
                w.set_cpol(cpol);

                w.set_mstr(vals::Mstr::SLAVE);
                w.set_ssm(cs.is_none());

                w.set_lsbfirst(lsbfirst);
                w.set_crcen(false);
                w.set_bidimode(vals::Bidimode::UNIDIRECTIONAL);
            });
        }
        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        {
            regs.ifcr().write(|w| w.0 = 0xffff_ffff);
            regs.cfg2().modify(|w| {
                w.set_cpha(cpha);
                w.set_cpol(cpol);
                w.set_lsbfirst(lsbfirst);

                w.set_master(vals::Master::SLAVE);
                w.set_ssm(cs.is_none());

                w.set_comm(vals::Comm::FULLDUPLEX);
                w.set_ssom(vals::Ssom::ASSERTED);
                w.set_midi(0);
                w.set_mssi(0);
                w.set_afcntr(true);
                w.set_ssiop(vals::Ssiop::ACTIVEHIGH);
            });
            regs.cfg1().modify(|w| {
                w.set_crcen(false);
                w.set_dsize(<u8 as SealedWord>::CONFIG);
                w.set_fthlv(vals::Fthlv::ONEFRAME);
            });
            regs.cr2().modify(|w| {
                w.set_tsize(0);
            });
            regs.cr1().modify(|w| {
                w.set_ssi(false);
            });
        }
        
        Self {
            info,
            sck,
            mosi,
            miso,
            cs,
            tx_dma,
            rx_dma,
            _phantom: PhantomData,
            current_word_size: <u8 as SealedWord>::CONFIG,
        }
    }

    fn set_word_size(&mut self, word_size: word_impl::Config) {
        if self.current_word_size == word_size {
            return;
        }

        #[cfg(any(spi_v1, spi_f1))]
        {
            self.info.regs.cr1().modify(|reg| {
                reg.set_spe(false);
                reg.set_dff(word_size)
            });
            self.info.regs.cr1().modify(|reg| {
                reg.set_spe(true);
            });
        }
        #[cfg(spi_v2)]
        {
            self.info.regs.cr1().modify(|w| {
                w.set_spe(false);
            });
            self.info.regs.cr2().modify(|w| {
                w.set_frxth(word_size.1);
                w.set_ds(word_size.0);
            });
            self.info.regs.cr1().modify(|w| {
                w.set_spe(true);
            });
        }
        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        {
            self.info.regs.cr1().modify(|w| {
                w.set_csusp(true);
            });
            while self.info.regs.sr().read().eot() {}
            self.info.regs.cr1().modify(|w| {
                w.set_spe(false);
            });
            self.info.regs.cfg1().modify(|w| {
                w.set_dsize(word_size);
            });
            self.info.regs.cr1().modify(|w| {
                w.set_csusp(false);
                w.set_spe(true);
            });
        }

        self.current_word_size = word_size;
    }

    /// Write a word to the SPI.
    pub fn write_blocking<W: Word>(&mut self, word: W) -> nb::Result<(), Error> {
        self.info.regs.cr1().modify(|w| w.set_spe(true));
        self.set_word_size(W::CONFIG);

        let _ = transfer_word(self.info.regs, word)?;

        Ok(())
    }

    /// Read a word from the SPI.
    pub fn read_blocking<W: Word>(&mut self) -> nb::Result<W, Error> {
        self.info.regs.cr1().modify(|w| w.set_spe(true));
        self.set_word_size(W::CONFIG);

        transfer_word(self.info.regs, W::default())
    }

    /// Bidirectionally transfer by writing a word to SPI while simultaneously reading a word from
    /// the SPI during the same clock cycle.
    pub fn transfer_blocking<W: Word>(&mut self, word: W) -> nb::Result<W, Error> {
        self.info.regs.cr1().modify(|w| w.set_spe(true));
        self.set_word_size(W::CONFIG);

        transfer_word(self.info.regs, word)
    }
}

impl<'d> SpiSlave<'d, Blocking> {
    /// Create a new SPI slave driver
    pub fn new_blocking<T: Instance>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, sck, mosi, miso);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        mosi.set_as_af(mosi.af_num(), AfType::input(Pull::None));
        miso.set_as_af(miso.af_num(), AfType::output(OutputType::PushPull, Speed::VeryHigh));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            None,
            None,
            None,
            config,
        )
    }

    /// Create a new SPI slave driver with hardware managed chip select
    pub fn new_blocking_hardware_cs<Cs, T: Instance>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
        cs: impl Peripheral<P = Cs> + 'd,
        config: Config,
    ) -> Self
    where
        Cs: CsPin<T>,
    {
        into_ref!(peri, sck, mosi, miso, cs);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        mosi.set_as_af(mosi.af_num(), AfType::input(Pull::None));
        miso.set_as_af(miso.af_num(), AfType::output(OutputType::PushPull, Speed::VeryHigh));
        cs.set_as_af(cs.af_num(), AfType::input(Pull::None));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            Some(cs.map_into()),
            None,
            None,
            config,
        )
    }
}

impl<'d> SpiSlave<'d, Async> {
    /// Create a new SPI slave driver
    pub fn new<T: Instance>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Self {
        into_ref!(peri, sck, mosi, miso, tx_dma, rx_dma);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        mosi.set_as_af(mosi.af_num(), AfType::input(Pull::None));
        miso.set_as_af(miso.af_num(), AfType::output(OutputType::PushPull, Speed::VeryHigh));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            None,
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }
    pub fn new_hardware_cs<Cs, T: Instance>(
        peri: impl Peripheral<P = T> + 'd,
        sck: impl Peripheral<P = impl SckPin<T>> + 'd,
        mosi: impl Peripheral<P = impl MosiPin<T>> + 'd,
        miso: impl Peripheral<P = impl MisoPin<T>> + 'd,
        cs: impl Peripheral<P = Cs> + 'd,
        tx_dma: impl Peripheral<P = impl TxDma<T>> + 'd,
        rx_dma: impl Peripheral<P = impl RxDma<T>> + 'd,
        config: Config,
    ) -> Self
    where
        Cs: CsPin<T>,
    {
        into_ref!(peri, sck, mosi, miso, cs);

        sck.set_as_af(sck.af_num(), AfType::input(Pull::None));
        mosi.set_as_af(mosi.af_num(), AfType::input(Pull::None));
        miso.set_as_af(miso.af_num(), AfType::output(OutputType::PushPull, Speed::VeryHigh));
        cs.set_as_af(cs.af_num(), AfType::input(Pull::None));

        Self::new_inner(
            peri,
            Some(sck.map_into()),
            Some(mosi.map_into()),
            Some(miso.map_into()),
            Some(cs.map_into()),
            new_dma!(tx_dma),
            new_dma!(rx_dma),
            config,
        )
    }

    /// SPI write, using DMA.
    pub async fn write<W: Word>(&mut self, data: &[W]) -> Result<(), Error> {
        if data.is_empty() {
            return Ok(());
        }

        self.set_word_size(W::CONFIG);
        self.info.regs.cr1().modify(|w| {
            w.set_spe(false);
        });

        let tx_dst = self.info.regs.tx_ptr();
        let tx_f = unsafe { self.tx_dma.as_mut().unwrap().write(data, tx_dst, Default::default()) };

        set_txdmaen(self.info.regs, true);
        self.info.regs.cr1().modify(|w| {
            w.set_spe(true);
        });
        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        self.info.regs.cr1().modify(|w| {
            w.set_cstart(true);
        });

        tx_f.await;

        finish_dma(self.info.regs);

        Ok(())
    }

    async fn transfer_inner<W: Word>(&mut self, read: *mut [W], write: *const [W]) -> Result<(), Error> {
        assert_eq!(read.len(), write.len());
        if read.len() == 0 {
            return Ok(());
        }

        self.set_word_size(W::CONFIG);
        self.info.regs.cr1().modify(|w| {
            w.set_spe(false);
        });

        // SPIv3 clears rxfifo on SPE=0
        #[cfg(not(any(spi_v3, spi_v4, spi_v5)))]
        flush_rx_fifo(self.info.regs);

        set_rxdmaen(self.info.regs, true);

        let rx_src = self.info.regs.rx_ptr();
        let rx_f = unsafe { self.rx_dma.as_mut().unwrap().read_raw(rx_src, read, Default::default()) };

        let tx_dst = self.info.regs.tx_ptr();
        let tx_f = unsafe {
            self.tx_dma
                .as_mut()
                .unwrap()
                .write_raw(write, tx_dst, Default::default())
        };

        set_txdmaen(self.info.regs, true);
        self.info.regs.cr1().modify(|w| {
            w.set_spe(true);
        });
        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        self.info.regs.cr1().modify(|w| {
            w.set_cstart(true);
        });

        join(tx_f, rx_f).await;

        finish_dma(self.info.regs);

        Ok(())
    }

    pub async fn transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        self.transfer_inner(read, write).await
    }

    pub async fn transfer_in_place<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        self.transfer_inner(data, data).await
    }
}

impl<'d, M: PeriMode> Drop for SpiSlave<'d, M> {
    fn drop(&mut self) {
        self.sck.as_ref().map(|x| x.set_as_disconnected());
        self.mosi.as_ref().map(|x| x.set_as_disconnected());
        self.miso.as_ref().map(|x| x.set_as_disconnected());
        self.cs.as_ref().map(|x| x.set_as_disconnected());

        self.info.rcc.disable();
    }
}

impl<'d, M: PeriMode> SetConfig for SpiSlave<'d, M> {
    type Config = Config;
    type ConfigError = ();
    fn set_config(&mut self, _config: &Self::Config) -> Result<(), ()> {
        unimplemented!()
    }
}

fn transfer_word<W: Word>(regs: Regs, tx_word: W) -> nb::Result<W, Error> {
    // To keep the tx and rx FIFO queues in the SPI peripheral synchronized, a word must be
    // simultaneously sent and received, even when only sending or receiving.
    if !tx_ready(regs, true)? || !rx_ready(regs)? {
        return Err(nb::Error::WouldBlock);
    }

    unsafe {
        ptr::write_volatile(regs.tx_ptr(), tx_word);

        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        regs.cr1().modify(|reg| reg.set_cstart(true));
    }

    let rx_word = unsafe { ptr::read_volatile(regs.rx_ptr()) };
    Ok(rx_word)
}
