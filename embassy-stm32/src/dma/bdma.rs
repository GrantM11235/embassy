#![macro_use]

use core::cell::UnsafeCell;
use core::future::{poll_fn, Future};
use core::mem::MaybeUninit;
use core::ops::ControlFlow;
use core::pin::Pin;
use core::sync::atomic::{fence, Ordering};
use core::task::{Context, Poll};

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

struct Foo(UnsafeCell<MaybeUninit<*mut (dyn FnMut() + Send)>>);

impl Foo {
    const UNINIT: Self = Self::uninit();

    const fn uninit() -> Self {
        Self(UnsafeCell::new(MaybeUninit::uninit()))
    }

    unsafe fn set<'a>(&self, closure: &'a mut (dyn FnMut() + Send + 'a)) {
        // erase closure lifetime
        let closure: &mut (dyn FnMut() + Send + 'static) = core::mem::transmute(closure);
        (*self.0.get()).write(closure);
    }

    unsafe fn run(&self) {
        let closure = &mut *(*self.0.get()).assume_init();
        closure()
    }
}

unsafe impl Sync for Foo {}

struct State {
    ch_wakers: [AtomicWaker; BDMA_CHANNEL_COUNT],
    foo: [Foo; BDMA_CHANNEL_COUNT],
}

impl State {
    const fn new() -> Self {
        const AW: AtomicWaker = AtomicWaker::new();
        Self {
            ch_wakers: [AW; BDMA_CHANNEL_COUNT],
            foo: [Foo::UNINIT; BDMA_CHANNEL_COUNT],
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

    let enabled_interrupts = cr.read();

    if enabled_interrupts.htie() {
        // Circular mode
        fence(Ordering::Acquire);
        STATE.foo[index].run();
    } else if isr.tcif(channel_num) && enabled_interrupts.tcie() {
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

#[derive(Clone, Copy)]
pub(crate) enum Half {
    First,
    Second,
}

fn choose_half<T>(buffer: SendPtr<[T; 2]>, half: Half) -> SendPtr<T> {
    let ptr = buffer.0;
    let ptr = ptr as *mut T;
    let ptr = match half {
        Half::First => ptr,
        Half::Second => unsafe { ptr.add(1) },
    };
    SendPtr(ptr)
}

pub(crate) async unsafe fn circ_dma_read<'a, C: Channel, W: Word, const N: usize>(
    channel: impl Peripheral<P = C> + 'a,
    peri_addr: *mut W,
    buffer: &'a mut [[W; N]; 2],
    mut f: impl FnMut(SendPtr<[W; N]>) -> ControlFlow<()> + Send + 'a,
    after_enable: impl FnOnce(),
) {
    into_ref!(channel);

    let buffer: *mut [[W; N]; 2] = buffer;

    let send_ptr = SendPtr(buffer);

    let waker = AtomicWaker::new();

    let dma = channel.regs();
    let channel_num = channel.num();

    let mut closure = || {
        circ_read_isr(dma, channel_num, &mut f, send_ptr, &waker);
    };

    // let mut dyn_closure: &mut (dyn FnMut(Half) -> ControlFlow<()> + Send + 'a) = &mut closure;

    let transfer = CircRead::new(channel, peri_addr, buffer, &mut closure);

    after_enable();

    poll_fn(|cx| {
        waker.register(cx.waker());

        if transfer.is_running() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    })
    .await;

    // transfer.await;
}

unsafe fn circ_read_isr<W: Word, const N: usize>(
    dma: pac::bdma::Dma,
    channel_num: usize,
    f: &mut (impl FnMut(SendPtr<[W; N]>) -> ControlFlow<()> + Send),
    send_ptr: SendPtr<[[W; N]; 2]>,
    waker: &AtomicWaker,
) {
    fence(Ordering::Acquire);
    let cr = dma.ch(channel_num).cr();
    let isr = dma.isr().read();

    let first = isr.htif(channel_num);
    let second = isr.tcif(channel_num);

    let half = match (first, second) {
        (true, true) => panic!("overrun before closure"),
        (true, false) => Half::First,
        (false, true) => Half::Second,
        (false, false) => return,
    };

    dma.ifcr().write(|reg| match half {
        Half::First => reg.set_htif(channel_num, true),
        Half::Second => reg.set_tcif(channel_num, true),
    });

    let res = f(choose_half(send_ptr, half));

    let isr_again = dma.isr().read();
    let overran = match half {
        Half::First => isr_again.tcif(channel_num),
        Half::Second => isr_again.htif(channel_num),
    };
    if overran {
        panic!("overrun after closure, isr: {}", isr_again.0);
    }

    if res.is_break() {
        fence(Ordering::Release);
        cr.write(|_| ()); // Disable channel interrupts with the default value.

        waker.wake();
    }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct CircRead<'a, C: Channel> {
    channel: PeripheralRef<'a, C>,
}

impl<'a, C: Channel> CircRead<'a, C> {
    pub(crate) unsafe fn new<W: Word, const N: usize>(
        channel: impl Peripheral<P = C> + 'a,
        peri_addr: *mut W,
        buffer: *mut [[W; N]; 2],
        f: &'a mut (dyn FnMut() + Send + 'a),
    ) -> Self {
        into_ref!(channel);

        let ch = channel.regs().ch(channel.num());

        // #[cfg(bdma_v2)]
        // critical_section::with(|_| channel.regs().cselr().modify(|w| w.set_cs(channel.num(), _request)));

        // clear irqs
        channel.regs().ifcr().write(|w| {
            w.set_gif(channel.num(), true);
        });

        // #[cfg(dmamux)]
        // super::dmamux::configure_dmamux(&mut *this.channel, _request);

        STATE.foo[channel.index()].set(f);

        fence(Ordering::Release);

        unsafe {
            ch.par().write_value(peri_addr as u32);
            ch.mar().write_value(buffer as *mut _ as u32);
            ch.ndtr().write(|w| w.set_ndt(N as u16 * 2));
            ch.cr().write(|w| {
                w.set_psize(W::size().into());
                w.set_msize(W::size().into());
                w.set_minc(vals::Inc::ENABLED);
                w.set_circ(vals::Circ::ENABLED);
                w.set_dir(Dir::PeripheralToMemory.into());
                w.set_teie(true);
                w.set_htie(true);
                w.set_tcie(true);
                w.set_en(true);
            });
        }

        Self { channel }
    }

    pub fn is_running(&self) -> bool {
        let ch = self.channel.regs().ch(self.channel.num());
        unsafe { ch.cr().read() }.en()
    }
}

impl<'a, C: Channel> Drop for CircRead<'a, C> {
    fn drop(&mut self) {
        if self.is_running() {
            panic!("circular transfers must not be dropped while running")
        }
        fence(Ordering::Acquire);
    }
}

// impl<'a, C: Channel> Unpin for CircRead<'a, C> {}
// impl<'a, C: Channel> Future for CircRead<'a, C> {
//     type Output = ();
//     fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
//         STATE.ch_wakers[self.channel.index()].register(cx.waker());

//         if self.is_running() {
//             Poll::Pending
//         } else {
//             Poll::Ready(())
//         }
//     }
// }

#[derive(Clone, Copy)]
pub struct SendPtr<T>(pub *mut T);
unsafe impl<T> Send for SendPtr<T> {}
unsafe impl<T> Sync for SendPtr<T> {}
