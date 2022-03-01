macro_rules! dma_num {
    (DMA) => {
        1
    };
    (DMA1) => {
        1
    };
    (DMA2) => {
        2
    };
    (BDMA) => {
        1
    };
    (BDMA1) => {
        1
    };
    (BDMA2) => {
        2
    };
}

#[cfg(bdma)]
pub(crate) mod bdma;
#[cfg(dma)]
pub(crate) mod dma;
#[cfg(dmamux)]
mod dmamux;

#[cfg(dmamux)]
pub use dmamux::*;

use atomic_polyfill::{AtomicU8, Ordering};
use core::future::Future;
use core::marker::PhantomData;
use core::mem;
use core::pin::Pin;
use core::task::Waker;
use core::task::{Context, Poll};
use embassy::util::Unborrow;
use embassy_hal_common::unborrow;

#[cfg(feature = "unstable-pac")]
pub mod low_level {
    pub use super::transfers::*;
}

pub(crate) use transfers::*;

#[cfg(any(bdma_v2, dma_v2, dmamux))]
pub type Request = u8;
#[cfg(not(any(bdma_v2, dma_v2, dmamux)))]
pub type Request = ();

pub enum Half {
    First,
    Second,
}

const STATE_BITS_PER_CHANNEL: usize = 2;

struct DmaState<const N: usize>([AtomicU8; N]);

impl<const N: usize> DmaState<N> {
    const fn new() -> Self {
        const ZERO: AtomicU8 = AtomicU8::new(0);
        Self([ZERO; N])
    }

    fn get_byte(&self, index: u8) -> &AtomicU8 {
        &self.0[index as usize / (8 / STATE_BITS_PER_CHANNEL)]
    }

    fn get_offset(index: u8) -> usize {
        (index as usize * STATE_BITS_PER_CHANNEL) % 8
    }

    fn get(&self, index: u8) -> u8 {
        // TODO: Relax ordering?
        let byte = self.get_byte(index).load(Ordering::SeqCst);
        let shifted = byte >> Self::get_offset(index);
        shifted & 0b11
    }

    fn get_available_half(&self, index: u8) -> Option<Half> {
        let state = self.get(index);
        let first = state & 0b01 != 0;
        let second = state & 0b10 != 0;

        match (first, second) {
            (false, false) => None,
            (true, false) => Some(Half::First),
            (false, true) => Some(Half::Second),
            (true, true) => panic!("Buffer overrun"),
        }
    }

    unsafe fn set_available_half(&self, index: u8, half: Half) {
        let mut foo = match half {
            Half::First => 0b01,
            Half::Second => 0b10,
        };
        foo <<= Self::get_offset(index);
        // TODO: Relax ordering?
        self.get_byte(index).fetch_or(foo, Ordering::SeqCst);
    }

    unsafe fn mark_half_done(&self, index: u8, half: Half) {
        let mut foo = match half {
            Half::First => 0b01,
            Half::Second => 0b10,
        };
        foo <<= Self::get_offset(index);
        // TODO: Relax ordering?
        self.get_byte(index).fetch_and(!foo, Ordering::SeqCst);
    }
}

pub(crate) mod sealed {
    use super::*;

    pub trait Word {}

    pub trait Channel {
        /// Starts this channel for writing a stream of words.
        ///
        /// Safety:
        /// - `buf` must point to a valid buffer for DMA reading.
        /// - `buf` must be alive for the entire duration of the DMA transfer.
        /// - `reg_addr` must be a valid peripheral register address to write to.
        unsafe fn start_write<W: super::Word>(
            &mut self,
            request: Request,
            buf: *const [W],
            reg_addr: *mut W,
        );

        /// Starts this channel for writing a word repeatedly.
        ///
        /// Safety:
        /// - `reg_addr` must be a valid peripheral register address to write to.
        unsafe fn start_write_repeated<W: super::Word>(
            &mut self,
            request: Request,
            repeated: W,
            count: usize,
            reg_addr: *mut W,
        );

        /// Starts this channel for reading a stream of words.
        ///
        /// Safety:
        /// - `buf` must point to a valid buffer for DMA writing.
        /// - `buf` must be alive for the entire duration of the DMA transfer.
        /// - `reg_addr` must be a valid peripheral register address to read from.
        unsafe fn start_read<W: super::Word>(
            &mut self,
            request: Request,
            reg_addr: *const W,
            buf: *mut [W],
        );

        unsafe fn start_circ_read<W: super::Word, const N: usize>(
            &mut self,
            request: Request,
            reg_addr: *const W,
            buf: *mut [[W; N]; 2],
        );

        unsafe fn start_circ_write<W: super::Word, const N: usize>(
            &mut self,
            request: Request,
            reg_addr: *mut W,
            buf: *const [[W; N]; 2],
        );

        fn get_available_half(&self) -> Option<Half>;

        unsafe fn mark_half_done(&self, half: Half);

        /// Requests the channel to stop.
        /// NOTE: The channel does not immediately stop, you have to wait
        /// for `is_running() = false`.
        fn request_stop(&mut self);

        /// Returns whether this channel is running or stopped.
        ///
        /// The channel stops running when it either completes or is manually stopped.
        fn is_running(&self) -> bool;

        /// Returns the total number of remaining transfers.
        fn remaining_transfers(&mut self) -> u16;

        /// Sets the waker that is called when this channel stops (either completed or manually stopped)
        fn set_waker(&mut self, waker: &Waker);
    }
}

pub enum WordSize {
    OneByte,
    TwoBytes,
    FourBytes,
}
pub trait Word: sealed::Word {
    fn bits() -> WordSize;
}

impl sealed::Word for u8 {}
impl Word for u8 {
    fn bits() -> WordSize {
        WordSize::OneByte
    }
}

impl sealed::Word for u16 {}
impl Word for u16 {
    fn bits() -> WordSize {
        WordSize::TwoBytes
    }
}

impl sealed::Word for u32 {}
impl Word for u32 {
    fn bits() -> WordSize {
        WordSize::FourBytes
    }
}

mod transfers {
    use core::ops::ControlFlow;

    use super::*;

    #[allow(unused)]
    pub fn read<'a, W: Word>(
        channel: impl Unborrow<Target = impl Channel> + 'a,
        request: Request,
        reg_addr: *mut W,
        buf: &'a mut [W],
    ) -> impl Future<Output = ()> + 'a {
        assert!(buf.len() > 0 && buf.len() <= 0xFFFF);
        unborrow!(channel);

        unsafe { channel.start_read::<W>(request, reg_addr, buf) };

        Transfer::new(channel)
    }

    #[allow(unused)]
    pub fn circ_read<'a, W: Word, R: 'a, const N: usize>(
        channel: impl Unborrow<Target = impl Channel> + 'a,
        request: Request,
        reg_addr: *mut W,
        buffer: &'a mut [[W; N]; 2],
        closure: impl FnMut(&mut [W; N]) -> ControlFlow<R> + 'a,
    ) -> impl Future<Output = R> + 'a {
        assert!(N > 0 && 2 * N <= 0xFFFF);
        unborrow!(channel);

        unsafe { channel.start_circ_read(request, reg_addr, buffer) };

        CircTransfer::new(channel, buffer, closure)
    }

    #[allow(unused)]
    pub fn write<'a, W: Word>(
        channel: impl Unborrow<Target = impl Channel> + 'a,
        request: Request,
        buf: &'a [W],
        reg_addr: *mut W,
    ) -> impl Future<Output = ()> + 'a {
        assert!(buf.len() > 0 && buf.len() <= 0xFFFF);
        unborrow!(channel);

        unsafe { channel.start_write::<W>(request, buf, reg_addr) };

        Transfer::new(channel)
    }

    #[allow(unused)]
    pub fn write_repeated<'a, W: Word>(
        channel: impl Unborrow<Target = impl Channel> + 'a,
        request: Request,
        repeated: W,
        count: usize,
        reg_addr: *mut W,
    ) -> impl Future<Output = ()> + 'a {
        unborrow!(channel);

        unsafe { channel.start_write_repeated::<W>(request, repeated, count, reg_addr) };

        Transfer::new(channel)
    }

    #[allow(unused)]
    pub fn circ_write<'a, W: Word, R: 'a, const N: usize>(
        channel: impl Unborrow<Target = impl Channel> + 'a,
        request: Request,
        reg_addr: *mut W,
        buffer: &'a mut [[W; N]; 2],
        closure: impl FnMut(&mut [W; N]) -> ControlFlow<R> + 'a,
    ) -> impl Future<Output = R> + 'a {
        assert!(N > 0 && 2 * N <= 0xFFFF);
        unborrow!(channel);

        unsafe { channel.start_circ_write(request, reg_addr, buffer) };

        CircTransfer::new(channel, buffer, closure)
    }

    pub(crate) struct Transfer<'a, C: Channel> {
        channel: C,
        _phantom: PhantomData<&'a mut C>,
    }

    impl<'a, C: Channel> Transfer<'a, C> {
        pub(crate) fn new(channel: impl Unborrow<Target = C> + 'a) -> Self {
            unborrow!(channel);
            Self {
                channel,
                _phantom: PhantomData,
            }
        }
    }

    impl<'a, C: Channel> Drop for Transfer<'a, C> {
        fn drop(&mut self) {
            self.channel.request_stop();
            while self.channel.is_running() {}
        }
    }

    impl<'a, C: Channel> Unpin for Transfer<'a, C> {}
    impl<'a, C: Channel> Future for Transfer<'a, C> {
        type Output = ();
        fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            self.channel.set_waker(cx.waker());
            if self.channel.is_running() {
                Poll::Pending
            } else {
                Poll::Ready(())
            }
        }
    }

    #[pin_project::pin_project(PinnedDrop)]
    pub(crate) struct CircTransfer<'a, C, W, F, R, const N: usize>
    where
        C: Channel,
        F: FnMut(&mut [W; N]) -> ControlFlow<R>,
    {
        channel: C,
        buffer: *mut [[W; N]; 2],
        closure: F,
        _phantom: PhantomData<&'a mut C>,
    }

    impl<'a, C, W, F, R, const N: usize> CircTransfer<'a, C, W, F, R, N>
    where
        C: Channel,
        F: FnMut(&mut [W; N]) -> ControlFlow<R>,
    {
        pub(crate) fn new(
            channel: impl Unborrow<Target = C> + 'a,
            buffer: *mut [[W; N]; 2],
            closure: F,
        ) -> Self {
            unborrow!(channel);
            Self {
                channel,
                buffer,
                closure,
                _phantom: PhantomData,
            }
        }
    }

    #[pin_project::pinned_drop]
    impl<'a, C, W, F, R, const N: usize> PinnedDrop for CircTransfer<'a, C, W, F, R, N>
    where
        C: Channel,
        F: FnMut(&mut [W; N]) -> ControlFlow<R>,
    {
        fn drop(self: Pin<&mut Self>) {
            let this = self.project();
            this.channel.request_stop();
            while this.channel.is_running() {}
        }
    }

    impl<'a, C, W, F, R, const N: usize> Future for CircTransfer<'a, C, W, F, R, N>
    where
        C: Channel,
        F: FnMut(&mut [W; N]) -> ControlFlow<R>,
    {
        type Output = R;
        fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
            self.channel.set_waker(cx.waker());
            match self.channel.get_available_half() {
                Some(half) => {
                    let ptr = match half {
                        Half::First => self.buffer as *mut [W; N],
                        Half::Second => unsafe { (self.buffer as *mut [W; N]).add(N) },
                    };

                    let buf: &mut [W; N] = unsafe { &mut *ptr };
                    let res = (self.closure)(buf);

                    drop(buf);

                    unsafe {
                        self.channel.mark_half_done(half);
                    }

                    match res {
                        ControlFlow::Continue(()) => Poll::Pending,
                        ControlFlow::Break(res) => {
                            self.channel.request_stop();
                            while self.channel.is_running() {}
                            Poll::Ready(res)
                        }
                    }
                }
                None => Poll::Pending,
            }
        }
    }
}

pub trait Channel: sealed::Channel + Unborrow<Target = Self> + 'static {}

pub struct NoDma;

unsafe impl Unborrow for NoDma {
    type Target = NoDma;

    unsafe fn unborrow(self) -> Self::Target {
        self
    }
}

// safety: must be called only once at startup
pub(crate) unsafe fn init() {
    #[cfg(bdma)]
    bdma::init();
    #[cfg(dma)]
    dma::init();
    #[cfg(dmamux)]
    dmamux::init();
}

// TODO: replace transmutes with core::ptr::metadata once it's stable
#[allow(unused)]
pub(crate) fn slice_ptr_parts<T>(slice: *const [T]) -> (usize, usize) {
    unsafe { mem::transmute(slice) }
}

#[allow(unused)]
pub(crate) fn slice_ptr_parts_mut<T>(slice: *mut [T]) -> (usize, usize) {
    unsafe { mem::transmute(slice) }
}
