#![macro_use]

use core::sync::atomic::{fence, Ordering};
use core::task::Waker;

use atomic_polyfill::AtomicU8;
use embassy::interrupt::{Interrupt, InterruptExt};
use embassy::waitqueue::AtomicWaker;

use crate::dma::Request;
use crate::generated::BDMA_CHANNEL_COUNT;
use crate::pac;
use crate::pac::bdma::{regs, vals};

use super::{Word, WordSize};

impl From<WordSize> for vals::Size {
    fn from(raw: WordSize) -> Self {
        match raw {
            WordSize::OneByte => Self::BITS8,
            WordSize::TwoBytes => Self::BITS16,
            WordSize::FourBytes => Self::BITS32,
        }
    }
}

enum Half {
    First,
    Second,
}

const STATE_BITS_PER_CHANNEL: usize = 2;

const STATE_BYTES: usize = ((BDMA_CHANNEL_COUNT * STATE_BITS_PER_CHANNEL) + 7) / 8;

struct DmaState([AtomicU8; STATE_BYTES]);

impl DmaState {
    const fn new() -> Self {
        const ZERO: AtomicU8 = AtomicU8::new(0);
        Self([ZERO; STATE_BYTES])
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
            (true, true) => buffer_overrun(),
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

struct State {
    ch_wakers: [AtomicWaker; BDMA_CHANNEL_COUNT],
    dma_state: DmaState,
}

impl State {
    const fn new() -> Self {
        const AW: AtomicWaker = AtomicWaker::new();
        Self {
            ch_wakers: [AW; BDMA_CHANNEL_COUNT],
            dma_state: DmaState::new(),
        }
    }
}

static STATE: State = State::new();

pub(crate) unsafe fn on_irq() {
    // TODO: clean up this interrupt handler so that it doesn't check every single channel
    foreach_peripheral! {
        (bdma, BDMA1) => {
            // BDMA1 in H7 doesn't use DMAMUX, which breaks
        };
        (bdma, $dma:ident) => {
            use pac::$dma;
            let isr_cached = pac::$dma.isr().read();
            foreach_dma_channel! {
                ($channel_peri:ident, $dma, bdma, $channel_num:expr, $index:expr, $dmamux:tt) => {
                    handle_channel($dma, dma_num!($dma), $channel_num, $index, isr_cached);
                };
            }
        };
    }
}

unsafe fn handle_channel(
    dma: pac::bdma::Dma,
    dma_num: u8,
    channel_num: u8,
    index: u8,
    isr_cached: regs::Isr,
) {
    let cr = dma.ch(channel_num as usize).cr();

    if isr_cached.teif(channel_num as usize) {
        panic!("BDMA: error on BDMA {} channel {}", dma_num, channel_num);
    }

    let cr_cached = cr.read();

    // The half transfer interrupt is enabled if and only if we are in circular mode
    if cr_cached.htie() {
        // Circular mode
        match (
            isr_cached.htif(channel_num as usize),
            isr_cached.tcif(channel_num as usize),
        ) {
            (false, false) => {
                // Neither half is done, do nothing
            }
            (true, false) => {
                // First half is done
                if STATE.dma_state.get_available_half(index).is_some() {
                    buffer_overrun();
                }
                STATE.dma_state.set_available_half(index, Half::First);
            }
            (false, true) => {
                // Second half is done
                if STATE.dma_state.get_available_half(index).is_some() {
                    buffer_overrun();
                }
                STATE.dma_state.set_available_half(index, Half::Second);
            }
            (true, true) => {
                buffer_overrun();
            }
        }
    } else {
        // Regular mode
        // TODO: Why do we need to check tcie?
        if isr_cached.tcif(channel_num as usize) && cr_cached.tcie() {
            cr.write(|_| ()); // Disable channel interrupts with the default value.
            STATE.ch_wakers[index as usize].wake();
        }
    }
}

fn buffer_overrun() -> ! {
    panic!("Buffer overrun");
}

/// safety: must be called only once
pub(crate) unsafe fn init() {
    foreach_interrupt! {
        ($peri:ident, bdma, $block:ident, $signal_name:ident, $irq:ident) => {
            crate::interrupt::$irq::steal().enable();
        };
    }
    crate::generated::init_bdma();
}

foreach_dma_channel! {
    ($channel_peri:ident, BDMA1, bdma, $channel_num:expr, $index:expr, $dmamux:tt) => {
        // BDMA1 in H7 doesn't use DMAMUX, which breaks
    };
    ($channel_peri:ident, $dma_peri:ident, bdma, $channel_num:expr, $index:expr, $dmamux:tt) => {
        impl crate::dma::sealed::Channel for crate::peripherals::$channel_peri {

            unsafe fn start_write<W: Word>(&mut self, _request: Request, buf: *const[W], reg_addr: *mut W) {
                let (ptr, len) = super::slice_ptr_parts(buf);
                low_level_api::start_transfer(
                    pac::$dma_peri,
                    $channel_num,
                    #[cfg(any(bdma_v2, dmamux))]
                    _request,
                    vals::Dir::FROMMEMORY,
                    reg_addr as *const u32,
                    ptr as *mut u32,
                    len,
                    true,
                    vals::Size::from(W::bits()),
                    false,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
                );
            }


            unsafe fn start_write_repeated<W: Word>(&mut self, _request: Request, repeated: W, count: usize, reg_addr: *mut W) {
                let buf = [repeated];
                low_level_api::start_transfer(
                    pac::$dma_peri,
                    $channel_num,
                    #[cfg(any(bdma_v2, dmamux))]
                    _request,
                    vals::Dir::FROMMEMORY,
                    reg_addr as *const u32,
                    buf.as_ptr() as *mut u32,
                    count,
                    false,
                    vals::Size::from(W::bits()),
                    false,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
                )
            }

            unsafe fn start_read<W: Word>(&mut self, _request: Request, reg_addr: *const W, buf: *mut [W]) {
                let (ptr, len) = super::slice_ptr_parts_mut(buf);
                low_level_api::start_transfer(
                    pac::$dma_peri,
                    $channel_num,
                    #[cfg(any(bdma_v2, dmamux))]
                    _request,
                    vals::Dir::FROMPERIPHERAL,
                    reg_addr as *const u32,
                    ptr as *mut u32,
                    len,
                    true,
                    vals::Size::from(W::bits()),
                    false,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_REGS,
                    #[cfg(dmamux)]
                    <Self as super::dmamux::sealed::MuxChannel>::DMAMUX_CH_NUM,
                );
            }

            fn request_stop(&mut self){
                unsafe {low_level_api::request_stop(pac::$dma_peri, $channel_num);}
            }

            fn is_running(&self) -> bool {
                unsafe {low_level_api::is_running(pac::$dma_peri, $channel_num)}
            }
            fn remaining_transfers(&mut self) -> u16 {
                unsafe {low_level_api::get_remaining_transfers(pac::$dma_peri, $channel_num)}
            }

            fn set_waker(&mut self, waker: &Waker) {
                unsafe { low_level_api::set_waker($index, waker) }
            }
        }

        impl crate::dma::Channel for crate::peripherals::$channel_peri {}
    };
}

mod low_level_api {
    use super::*;

    pub unsafe fn start_transfer(
        dma: pac::bdma::Dma,
        channel_number: u8,
        #[cfg(any(bdma_v2, dmamux))] request: Request,
        dir: vals::Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: bool,
        data_size: vals::Size,
        circular: bool,
        #[cfg(dmamux)] dmamux_regs: pac::dmamux::Dmamux,
        #[cfg(dmamux)] dmamux_ch_num: u8,
    ) {
        let ch = dma.ch(channel_number as _);

        reset_status(dma, channel_number);

        #[cfg(dmamux)]
        super::super::dmamux::configure_dmamux(dmamux_regs, dmamux_ch_num, request);

        #[cfg(bdma_v2)]
        critical_section::with(|_| {
            dma.cselr()
                .modify(|w| w.set_cs(channel_number as _, request))
        });

        // "Preceding reads and writes cannot be moved past subsequent writes."
        fence(Ordering::SeqCst);

        ch.par().write_value(peri_addr as u32);
        ch.mar().write_value(mem_addr as u32);
        ch.ndtr().write(|w| w.set_ndt(mem_len as u16));
        ch.cr().write(|w| {
            w.set_psize(data_size);
            w.set_msize(data_size);
            if incr_mem {
                w.set_minc(vals::Inc::ENABLED);
            } else {
                w.set_minc(vals::Inc::DISABLED);
            }
            if circular {
                w.set_circ(vals::Circ::ENABLED);
            } else {
                w.set_circ(vals::Circ::DISABLED);
            }
            w.set_dir(dir);
            w.set_teie(true);
            w.set_htie(circular);
            w.set_tcie(true);
            w.set_en(true);
        });
    }

    pub unsafe fn request_stop(dma: pac::bdma::Dma, channel_number: u8) {
        reset_status(dma, channel_number);

        let ch = dma.ch(channel_number as _);

        // Disable the channel and interrupts with the default value.
        ch.cr().write(|_| ());

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        fence(Ordering::SeqCst);
    }

    pub unsafe fn is_running(dma: pac::bdma::Dma, ch: u8) -> bool {
        let ch = dma.ch(ch as _);
        ch.cr().read().en()
    }

    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub unsafe fn get_remaining_transfers(dma: pac::bdma::Dma, ch: u8) -> u16 {
        // get a handle on the channel itself
        let ch = dma.ch(ch as _);
        // read the remaining transfer count. If this is zero, the transfer completed fully.
        ch.ndtr().read().ndt()
    }

    /// Sets the waker for the specified DMA channel
    pub unsafe fn set_waker(state_number: usize, waker: &Waker) {
        STATE.ch_wakers[state_number].register(waker);
    }

    pub unsafe fn reset_status(dma: pac::bdma::Dma, channel_number: u8) {
        dma.ifcr().write(|w| {
            w.set_tcif(channel_number as _, true);
            w.set_htif(channel_number as _, true);
            w.set_teif(channel_number as _, true);
        });
    }
}
