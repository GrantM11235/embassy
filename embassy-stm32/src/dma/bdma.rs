#![macro_use]

use core::future::{poll_fn, Future};
use core::ops::ControlFlow;
use core::pin::Pin;
use core::sync::atomic::{fence, Ordering};
use core::task::{Context, Poll};

use atomic_polyfill::AtomicU8;
use embassy_cortex_m::interrupt::Priority;
use embassy_hal_common::{into_ref, Peripheral, PeripheralRef};
use embassy_sync::waitqueue::AtomicWaker;

use super::word::{Word, WordSize};
use super::Dir;
use crate::_generated::BDMA_CHANNEL_COUNT;
use crate::interrupt::{Interrupt, InterruptExt};
use crate::pac;
use crate::pac::bdma::vals;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {}

impl Default for TransferOptions {
    fn default() -> Self {
        Self {}
    }
}

impl From<WordSize> for vals::Size {
    fn from(raw: WordSize) -> Self {
        match raw {
            WordSize::OneByte => Self::BITS8,
            WordSize::TwoBytes => Self::BITS16,
            WordSize::FourBytes => Self::BITS32,
        }
    }
}

impl From<Dir> for vals::Dir {
    fn from(raw: Dir) -> Self {
        match raw {
            Dir::MemoryToPeripheral => Self::FROMMEMORY,
            Dir::PeripheralToMemory => Self::FROMPERIPHERAL,
        }
    }
}

struct State {
    ch_wakers: [AtomicWaker; BDMA_CHANNEL_COUNT],
    foo: [AtomicU8; BDMA_CHANNEL_COUNT],
}

impl State {
    const fn new() -> Self {
        const AW: AtomicWaker = AtomicWaker::new();
        const FOO: AtomicU8 = AtomicU8::new(0);
        Self {
            ch_wakers: [AW; BDMA_CHANNEL_COUNT],
            foo: [FOO; BDMA_CHANNEL_COUNT],
        }
    }
}

static STATE: State = State::new();

/// safety: must be called only once
pub(crate) unsafe fn init(irq_priority: Priority) {
    foreach_interrupt! {
        ($peri:ident, bdma, $block:ident, $signal_name:ident, $irq:ident) => {
            let irq = crate::interrupt::$irq::steal();
            irq.set_priority(irq_priority);
            irq.enable();
        };
    }
    crate::_generated::init_bdma();
}

foreach_dma_channel! {
    ($channel_peri:ident, BDMA1, bdma, $channel_num:expr, $index:expr, $dmamux:tt) => {
        // BDMA1 in H7 doesn't use DMAMUX, which breaks
    };
    ($channel_peri:ident, $dma_peri:ident, bdma, $channel_num:expr, $index:expr, $dmamux:tt) => {
        impl sealed::Channel for crate::peripherals::$channel_peri {
            fn regs(&self) -> pac::bdma::Dma {
                pac::$dma_peri
            }
            fn num(&self) -> usize {
                $channel_num
            }
            fn index(&self) -> usize {
                $index
            }
            fn on_irq() {
                unsafe { on_irq_inner(pac::$dma_peri, $channel_num, $index) }
            }
        }

        impl Channel for crate::peripherals::$channel_peri {}
    };
}

/// Safety: Must be called with a matching set of parameters for a valid dma channel
pub(crate) unsafe fn on_irq_inner(dma: pac::bdma::Dma, channel_num: usize, index: usize) {
    let isr = dma.isr().read();
    let cr = dma.ch(channel_num).cr();

    if isr.teif(channel_num) {
        panic!("DMA: error on BDMA@{:08x} channel {}", dma.0 as u32, channel_num);
    }
    if cr.read().htie() {
        // Circular mode
        let first = isr.htif(channel_num);
        let second = isr.tcif(channel_num);
        if first || second {
            STATE.foo[index].fetch_or(first as u8 | ((second as u8) << 1), Ordering::SeqCst);
            dma.ifcr().write(|reg| {
                reg.set_htif(channel_num, first);
                reg.set_tcif(channel_num, second);
            });
            STATE.ch_wakers[index].wake();
        }
    } else if isr.tcif(channel_num) && cr.read().tcie() {
        cr.write(|_| ()); // Disable channel interrupts with the default value.
        STATE.ch_wakers[index].wake();
    }
}

#[cfg(any(bdma_v2, dmamux))]
pub type Request = u8;
#[cfg(not(any(bdma_v2, dmamux)))]
pub type Request = ();

#[cfg(dmamux)]
pub trait Channel: sealed::Channel + Peripheral<P = Self> + 'static + super::dmamux::MuxChannel {}
#[cfg(not(dmamux))]
pub trait Channel: sealed::Channel + Peripheral<P = Self> + 'static {}

pub(crate) mod sealed {
    use super::*;

    pub trait Channel {
        fn regs(&self) -> pac::bdma::Dma;
        fn num(&self) -> usize;
        fn index(&self) -> usize;
        fn on_irq();
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a, C: Channel> {
    channel: PeripheralRef<'a, C>,
}

impl<'a, C: Channel> Transfer<'a, C> {
    pub unsafe fn new_read<W: Word>(
        channel: impl Peripheral<P = C> + 'a,
        request: Request,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Self {
        Self::new_read_raw(channel, request, peri_addr, buf, options)
    }

    pub unsafe fn new_read_raw<W: Word>(
        channel: impl Peripheral<P = C> + 'a,
        request: Request,
        peri_addr: *mut W,
        buf: *mut [W],
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        let (ptr, len) = super::slice_ptr_parts_mut(buf);
        assert!(len > 0 && len <= 0xFFFF);

        Self::new_inner(
            channel,
            request,
            Dir::PeripheralToMemory,
            peri_addr as *const u32,
            ptr as *mut u32,
            len,
            true,
            W::size(),
            options,
        )
    }

    pub unsafe fn new_write<W: Word>(
        channel: impl Peripheral<P = C> + 'a,
        request: Request,
        buf: &'a [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        Self::new_write_raw(channel, request, buf, peri_addr, options)
    }

    pub unsafe fn new_write_raw<W: Word>(
        channel: impl Peripheral<P = C> + 'a,
        request: Request,
        buf: *const [W],
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        let (ptr, len) = super::slice_ptr_parts(buf);
        assert!(len > 0 && len <= 0xFFFF);

        Self::new_inner(
            channel,
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            ptr as *mut u32,
            len,
            true,
            W::size(),
            options,
        )
    }

    pub unsafe fn new_write_repeated<W: Word>(
        channel: impl Peripheral<P = C> + 'a,
        request: Request,
        repeated: &'a W,
        count: usize,
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Self {
        into_ref!(channel);

        Self::new_inner(
            channel,
            request,
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            repeated as *const W as *mut u32,
            count,
            false,
            W::size(),
            options,
        )
    }

    unsafe fn new_inner(
        channel: PeripheralRef<'a, C>,
        _request: Request,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: WordSize,
        _options: TransferOptions,
    ) -> Self {
        let ch = channel.regs().ch(channel.num());

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        #[cfg(bdma_v2)]
        critical_section::with(|_| channel.regs().cselr().modify(|w| w.set_cs(channel.num(), _request)));

        let mut this = Self { channel };
        this.clear_irqs();

        #[cfg(dmamux)]
        super::dmamux::configure_dmamux(&mut *this.channel, _request);

        ch.par().write_value(peri_addr as u32);
        ch.mar().write_value(mem_addr as u32);
        ch.ndtr().write(|w| w.set_ndt(mem_len as u16));
        ch.cr().write(|w| {
            w.set_psize(data_size.into());
            w.set_msize(data_size.into());
            if incr_mem {
                w.set_minc(vals::Inc::ENABLED);
            } else {
                w.set_minc(vals::Inc::DISABLED);
            }
            w.set_dir(dir.into());
            w.set_teie(true);
            w.set_tcie(true);
            w.set_en(true);
        });

        this
    }

    fn clear_irqs(&mut self) {
        unsafe {
            self.channel.regs().ifcr().write(|w| {
                w.set_tcif(self.channel.num(), true);
                w.set_teif(self.channel.num(), true);
            })
        }
    }

    pub fn request_stop(&mut self) {
        let ch = self.channel.regs().ch(self.channel.num());

        // Disable the channel. Keep the IEs enabled so the irqs still fire.
        unsafe {
            ch.cr().write(|w| {
                w.set_teie(true);
                w.set_tcie(true);
            })
        }
    }

    pub fn is_running(&mut self) -> bool {
        let ch = self.channel.regs().ch(self.channel.num());
        unsafe { ch.cr().read() }.en()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub fn get_remaining_transfers(&self) -> u16 {
        let ch = self.channel.regs().ch(self.channel.num());
        unsafe { ch.ndtr().read() }.ndt()
    }

    pub fn blocking_wait(mut self) {
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);

        core::mem::forget(self);
    }
}

impl<'a, C: Channel> Drop for Transfer<'a, C> {
    fn drop(&mut self) {
        self.request_stop();
        while self.is_running() {}

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }
}

impl<'a, C: Channel> Unpin for Transfer<'a, C> {}
impl<'a, C: Channel> Future for Transfer<'a, C> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        STATE.ch_wakers[self.channel.index()].register(cx.waker());

        if self.is_running() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

pub struct CircRead<'a, C: Channel, W: Word, const N: usize> {
    channel: PeripheralRef<'a, C>,
    buffer: *mut [[W; N]; 2],
}

enum Half {
    First,
    Second,
}

impl<'a, C: Channel, W: Word, const N: usize> CircRead<'a, C, W, N> {
    pub unsafe fn new(channel: impl Peripheral<P = C> + 'a, peri_addr: *mut W, buffer: &'a mut [[W; N]; 2]) -> Self {
        into_ref!(channel);

        let data_size = W::size();

        let ch = channel.regs().ch(channel.num());

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        // #[cfg(bdma_v2)]
        // critical_section::with(|_| channel.regs().cselr().modify(|w| w.set_cs(channel.num(), _request)));

        let mut this = Self { channel, buffer };
        this.clear_irqs();

        // #[cfg(dmamux)]
        // super::dmamux::configure_dmamux(&mut *this.channel, _request);

        unsafe {
            ch.par().write_value(peri_addr as u32);
            ch.mar().write_value(this.buffer as u32);
            ch.ndtr().write(|w| w.set_ndt(N as u16 * 2));
            ch.cr().write(|w| {
                w.set_psize(data_size.into());
                w.set_msize(data_size.into());
                w.set_minc(vals::Inc::ENABLED);
                w.set_circ(vals::Circ::ENABLED);
                w.set_dir(Dir::PeripheralToMemory.into());
                w.set_teie(true);
                w.set_htie(true);
                w.set_tcie(true);
                w.set_en(true);
            });
        }

        this
    }

    fn clear_irqs(&mut self) {
        unsafe {
            self.channel.regs().ifcr().write(|w| {
                w.set_tcif(self.channel.num(), true);
                w.set_teif(self.channel.num(), true);
            })
        }
    }

    async fn wait(&self) -> Half {
        poll_fn(|cx| {
            let foo = STATE.foo[self.channel.index()].load(Ordering::SeqCst);

            match foo {
                0 => {
                    STATE.ch_wakers[self.channel.index()].register(cx.waker());
                    Poll::Pending
                }
                1 => Poll::Ready(Half::First),
                2 => Poll::Ready(Half::Second),
                _ => panic!("Overrun"),
            }
        })
        .await
    }

    fn mark_half_done(&self, half: Half) {
        let mask = match half {
            Half::First => 1,
            Half::Second => 2,
        };

        let prev = STATE.foo[self.channel.index()].fetch_and(!mask, Ordering::SeqCst);

        if prev != mask {
            panic!("Overrun");
        }
    }

    async fn do_once<R>(&mut self, f: impl FnOnce(*mut [W; N]) -> R) -> R {
        let half = self.wait().await;

        let ptr = self.buffer;
        let ptr = ptr as *mut [W; N];
        let ptr = match half {
            Half::First => ptr,
            Half::Second => unsafe { ptr.add(1) },
        };

        let res = f(ptr);

        self.mark_half_done(half);

        res
    }

    pub async fn do_while<R>(&mut self, mut f: impl FnMut(*mut [W; N]) -> ControlFlow<R>) -> R {
        loop {
            let res = self.do_once(&mut f).await;

            if let ControlFlow::Break(res) = res {
                unsafe { self.channel.regs().ch(self.channel.num()).cr().write(|_| {}) }; // Disable channel interrupts with the default value.
                return res;
            }
        }
    }
}
