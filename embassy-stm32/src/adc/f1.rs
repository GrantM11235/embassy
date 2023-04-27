use embassy_hal_common::into_ref;
use embedded_hal_02::blocking::delay::DelayUs;

use crate::adc::{Adc, AdcPin, Instance, SampleTime};
use crate::rcc::get_freqs;
use crate::time::Hertz;
use crate::{dma, Peripheral};

pub const VDDA_CALIB_MV: u32 = 3300;
pub const ADC_MAX: u32 = (1 << 12) - 1;
// No calibration data for F103, voltage should be 1.2v
pub const VREF_INT: u32 = 1200;

pub struct Vref;
impl<T: Instance> AdcPin<T> for Vref {}
impl<T: Instance> super::sealed::AdcPin<T> for Vref {
    fn channel(&self) -> u8 {
        17
    }
}

pub struct Temperature;
impl<T: Instance> AdcPin<T> for Temperature {}
impl<T: Instance> super::sealed::AdcPin<T> for Temperature {
    fn channel(&self) -> u8 {
        16
    }
}

impl<'d, T: Instance> Adc<'d, T> {
    pub fn new(adc: impl Peripheral<P = T> + 'd, delay: &mut impl DelayUs<u32>) -> Self {
        into_ref!(adc);
        T::enable();
        T::reset();
        unsafe {
            T::regs().cr2().modify(|reg| reg.set_adon(true));
        }

        // 11.4: Before starting a calibration, the ADC must have been in power-on state (ADON bit = ‘1’)
        // for at least two ADC clock cycles
        delay.delay_us((1_000_000 * 2) / Self::freq().0 + 1);

        unsafe {
            // Reset calibration
            T::regs().cr2().modify(|reg| reg.set_rstcal(true));
            while T::regs().cr2().read().rstcal() {
                // spin
            }

            // Calibrate
            T::regs().cr2().modify(|reg| reg.set_cal(true));
            while T::regs().cr2().read().cal() {
                // spin
            }
        }

        // One cycle after calibration
        delay.delay_us((1_000_000) / Self::freq().0 + 1);

        Self {
            adc,
            sample_time: Default::default(),
        }
    }

    fn freq() -> Hertz {
        unsafe { get_freqs() }.adc
    }

    pub fn sample_time_for_us(&self, us: u32) -> SampleTime {
        match us * Self::freq().0 / 1_000_000 {
            0..=1 => SampleTime::Cycles1_5,
            2..=7 => SampleTime::Cycles7_5,
            8..=13 => SampleTime::Cycles13_5,
            14..=28 => SampleTime::Cycles28_5,
            29..=41 => SampleTime::Cycles41_5,
            42..=55 => SampleTime::Cycles55_5,
            56..=71 => SampleTime::Cycles71_5,
            _ => SampleTime::Cycles239_5,
        }
    }

    pub fn enable_vref(&self, _delay: &mut impl DelayUs<u32>) -> Vref {
        unsafe {
            T::regs().cr2().modify(|reg| {
                reg.set_tsvrefe(true);
            })
        }
        Vref {}
    }

    pub fn enable_temperature(&self) -> Temperature {
        unsafe {
            T::regs().cr2().modify(|reg| {
                reg.set_tsvrefe(true);
            })
        }
        Temperature {}
    }

    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;
    }

    /// Perform a single conversion.
    fn convert(&mut self) -> u16 {
        unsafe {
            T::regs().cr2().modify(|reg| {
                reg.set_adon(true);
                reg.set_swstart(true);
            });
            while T::regs().cr2().read().swstart() {}
            while !T::regs().sr().read().eoc() {}

            T::regs().dr().read().0 as u16
        }
    }

    pub fn read(&mut self, pin: &mut impl AdcPin<T>) -> u16 {
        unsafe {
            Self::set_channel_sample_time(pin.channel(), self.sample_time);
            T::regs().cr1().modify(|reg| {
                reg.set_scan(false);
                reg.set_discen(false);
            });
            T::regs().sqr1().modify(|reg| reg.set_l(0));

            T::regs().cr2().modify(|reg| {
                reg.set_cont(false);
                reg.set_exttrig(true);
                reg.set_swstart(false);
                reg.set_extsel(crate::pac::adc::vals::Extsel::SWSTART);
            });
        }

        // Configure the channel to sample
        unsafe { T::regs().sqr3().write(|reg| reg.set_sq(0, pin.channel())) }
        self.convert()
    }

    unsafe fn set_channel_sample_time(ch: u8, sample_time: SampleTime) {
        let sample_time = sample_time.into();
        if ch <= 9 {
            T::regs().smpr2().modify(|reg| reg.set_smp(ch as _, sample_time));
        } else {
            T::regs().smpr1().modify(|reg| reg.set_smp((ch - 10) as _, sample_time));
        }
    }

    fn regs(&self) -> crate::pac::adc::Adc {
        T::regs()
    }
}

use crate::peripherals::{ADC1, DMA1_CH1};

impl<'d> Adc<'d, ADC1> {
    pub fn dma_read<'a>(
        &'a mut self,
        pin: &'a mut impl AdcPin<ADC1>,
        buffer: &'a mut [u16],
        dma: &'a mut DMA1_CH1,
    ) -> dma::Transfer<'a, DMA1_CH1> {
        unsafe {
            self.regs().cr1().modify(|reg| reg.set_discen(false));
            Self::set_channel_sample_time(pin.channel(), self.sample_time);
            self.regs().sqr3().write(|reg| reg.set_sq(0, pin.channel()));
            self.regs().cr2().modify(|reg| reg.set_dma(true));
        }

        let transfer = unsafe {
            let peri_addr = self.regs().dr().ptr() as *mut u16;
            dma::Transfer::new_read(dma, (), peri_addr, buffer, Default::default())
        };

        unsafe {
            self.regs().cr2().modify(|reg| reg.set_cont(true));
            self.regs().cr2().modify(|reg| reg.set_adon(true));
        }

        transfer
    }

    pub async fn circ_dma_read<'a, const N: usize>(
        &'a mut self,
        pin: &'a mut impl AdcPin<ADC1>,
        buffer: &'a mut [[u16; N]; 2],
        channel: &'a mut DMA1_CH1,
        f: impl FnMut(crate::dma::SendPtr<[u16; N]>) -> core::ops::ControlFlow<()> + Send + 'a,
    ) {
        unsafe {
            self.regs().cr1().modify(|reg| reg.set_discen(false));
            Self::set_channel_sample_time(pin.channel(), self.sample_time);
            self.regs().sqr3().write(|reg| reg.set_sq(0, pin.channel()));
            self.regs().cr2().modify(|reg| reg.set_dma(true));

            let peri_addr = self.regs().dr().ptr() as *mut u16;

            let after_enable = || {
                self.regs().cr2().modify(|reg| reg.set_cont(true));
                self.regs().cr2().modify(|reg| reg.set_adon(true));
            };

            dma::circ_dma_read(channel, peri_addr, buffer, f, after_enable).await;
        }
    }
}
