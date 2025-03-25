use super::{check_error_flags, flush_rx_fifo, set_rxdmaen, SpiSlave};
use super::{Error, RegsExt, Word};
use crate::dma::ReadableRingBuffer;
use crate::mode::Mode as PeriMode;

/// SPI Slave Rx With Ring Buffer
#[cfg(not(gpdma))]
pub struct SpiSlaveRingBufferedRx<'d, T: PeriMode, W: Word> {
    _inner: SpiSlave<'d, T>,
    rx_ring_buffer: ReadableRingBuffer<'d, W>,
}

impl<'d, T: PeriMode, W> SpiSlaveRingBufferedRx<'d, T, W>
where
    W: Word,
{
    /// Read data from the ring buffer
    pub async fn read(&mut self, buf: &mut [W]) -> Result<(), Error> {
        loop {
            match self.rx_ring_buffer.read_exact(buf).await {
                Ok(_) => {
                    let sr = self._inner.info.regs.sr().read();
                    check_error_flags(sr, true)?;
                    return Ok(());
                }
                Err(crate::dma::ringbuffer::Error::Overrun) => {
                    self.handle_overrun();
                }
                Err(crate::dma::ringbuffer::Error::DmaUnsynced) => {
                    return Err(Error::Framing);
                }
            }
        }
    }

    fn handle_overrun(&mut self) {
        self._inner.info.regs.cr1().modify(|w| w.set_spe(false));

        flush_rx_fifo(self._inner.info.regs);

        self.rx_ring_buffer.clear();

        self._inner.info.regs.cr1().modify(|w| w.set_spe(true));

        self.rx_ring_buffer.start();
    }
}

impl<'d, M: PeriMode> SpiSlave<'d, M> {
    /// Into SPI RingBuffered Rx
    pub fn into_ringbuffered_rx<W: Word>(mut self, rxdma_buffer: &'d mut [W]) -> SpiSlaveRingBufferedRx<'d, M, W> {
        if rxdma_buffer.is_empty() {
            panic!("rxdma_buffer cannot be empty");
        }

        // 设置 SPI word size
        self.set_word_size(W::CONFIG);

        // 禁用 SPI 以进行 DMA 配置
        self.info.regs.cr1().modify(|w| w.set_spe(false));

        flush_rx_fifo(self.info.regs);

        // 启用 RX DMA
        set_rxdmaen(self.info.regs, true);

        // 获取 RX DMA 通道
        let rx_dma = match self.rx_dma.as_ref() {
            Some(dma) => dma,
            None => panic!("RX DMA not initialized"),
        };

        let rx_request = rx_dma.request;
        let rx_src = self.info.regs.rx_ptr();

        let mut rx_ring_buffer = unsafe {
            ReadableRingBuffer::new(
                rx_dma.channel.clone_unchecked(),
                rx_request,
                rx_src,
                rxdma_buffer,
                crate::dma::TransferOptions {
                    half_transfer_ir: true,
                    priority: crate::dma::Priority::VeryHigh,
                    ..Default::default()
                },
            )
        };

        // 重新启用 SPI
        self.info.regs.cr1().modify(|w| w.set_spe(true));

        // 对 SPIv3/v4/v5 启用 `cstart`
        #[cfg(any(spi_v3, spi_v4, spi_v5))]
        self.info.regs.cr1().modify(|w| w.set_cstart(true));

        // 启动 DMA 读取
        rx_ring_buffer.start();

        SpiSlaveRingBufferedRx {
            _inner: self,
            rx_ring_buffer,
        }
    }
}
