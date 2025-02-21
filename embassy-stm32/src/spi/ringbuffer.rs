use super::{check_error_flags, flush_rx_fifo, set_rxdmaen, SpiSlave};
use super::{Error, RegsExt, Word};
use crate::dma::ReadableRingBuffer;
use crate::mode::Mode as PeriMode;

#[cfg(not(gpdma))]
/// SPI Slave With RingBuffered Rx
pub struct SpiSlaveRingBufferedRx<'d, T: PeriMode, W: Word> {
    _inner: SpiSlave<'d, T>,
    rx_ring_buffer: ReadableRingBuffer<'d, W>,
}

impl<'d, T: PeriMode, W> SpiSlaveRingBufferedRx<'d, T, W>
where
    W: Word,
{
    /// 异步等待读取 exact 个元素，优化过 OverRun 处理
    pub async fn read(&mut self, buf: &mut [W]) -> Result<(), Error> {
        loop {
            match self.rx_ring_buffer.read_exact(buf).await {
                Ok(_) => {
                    // 可选：检查 SPI 状态寄存器中的错误标志
                    let sr = self._inner.info.regs.sr().read();
                    check_error_flags(sr, true)?;
                    return Ok(());
                }
                Err(crate::dma::ringbuffer::Error::Overrun) => {
                    // 发生 Overrun，清空 RX FIFO 并重启 DMA
                    self.handle_overrun();
                }
                Err(crate::dma::ringbuffer::Error::DmaUnsynced) => {
                    return Err(Error::Framing);
                }
            }
        }
    }

    /// 处理 OverRun 错误
    fn handle_overrun(&mut self) {
        // 禁用 SPI 以清除 Overrun
        self._inner.info.regs.cr1().modify(|w| w.set_spe(false));

        // 清空 RX FIFO
        flush_rx_fifo(self._inner.info.regs);

        // 重新启动 DMA
        self.rx_ring_buffer.clear();

        // 重新启用 SPI
        self._inner.info.regs.cr1().modify(|w| w.set_spe(true));

        // 重新启动 DMA 传输
        self.rx_ring_buffer.start();
    }
}

impl<'d, M: PeriMode> SpiSlave<'d, M> {
    /// Into SPI RingBuffered Rx (优化版)
    pub fn into_ringbuffered_rx<W: Word>(mut self, rxdma_buffer: &'d mut [W]) -> SpiSlaveRingBufferedRx<'d, M, W> {
        self.set_word_size(W::CONFIG);

        // 禁用 SPI 仅用于初始化阶段，之后保持使能状态
        self.info.regs.cr1().modify(|w| w.set_spe(false));

        // 先使能 RX DMA，再配置 DMA 环形缓冲区
        set_rxdmaen(self.info.regs, true);
        let rx_dma = self.rx_dma.as_ref().unwrap();
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
                    priority: crate::dma::Priority::VeryHigh, // 优先级调至最高
                    ..Default::default()
                },
            )
        };

        // 重新启动 SPI
        self.info.regs.cr1().modify(|w| w.set_spe(true));

        rx_ring_buffer.start();

        SpiSlaveRingBufferedRx {
            _inner: self,
            rx_ring_buffer,
        }
    }
}
