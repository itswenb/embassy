use super::{check_error_flags, set_rxdmaen, RxDma, SpiSlave, TxDma};
use super::{Error, Instance, RegsExt, Word};
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
    /// 启动连续接收模式（在 dma_ringbuffered 中已启动 DMA 循环传输）
    /// 该方法从 DMA 环形缓冲区中读取数据到用户提供的 buf 中，
    /// 返回读取的元素数量。如果可读数据不足则返回 Error::Overrun。
    pub fn read_next(&mut self, buf: &mut [W]) -> Result<usize, Error> {
        match self.rx_ring_buffer.read(buf) {
            Ok((read_count, _)) => Ok(read_count),
            Err(_) => {
                // 如果发生数据溢出，则先清空缓冲区，返回错误
                self.rx_ring_buffer.clear();
                Err(Error::Overrun)
            }
        }
    }

    /// 异步等待读取 exact 个元素
    pub async fn read_exact(&mut self, buf: &mut [W]) -> Result<(), Error> {
        self.rx_ring_buffer.read_exact(buf).await.map_err(|_| Error::Overrun)?;

        // 可选：检查 SPI 状态寄存器中的错误标志
        let sr = self._inner.info.regs.sr().read();
        check_error_flags(sr, true)?;

        Ok(())
    }
}

impl<'d, M: PeriMode> SpiSlave<'d, M> {
    /// Into SPI RingBuffered Rx
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
                    priority: crate::dma::Priority::High,
                    ..Default::default()
                },
            )
        };
        self.info.regs.cr1().modify(|w| w.set_spe(true));

        rx_ring_buffer.start();

        SpiSlaveRingBufferedRx {
            _inner: self,
            rx_ring_buffer,
        }
    }
}
