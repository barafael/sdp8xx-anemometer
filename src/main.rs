#![no_std]
#![no_main]

use panic_halt as _;

use core::fmt::Write;

use stm32f0xx_hal::timers::Timer;
use stm32f0xx_hal::{pac, prelude::*};

use cortex_m::{interrupt::free as disable_interrupts, Peripherals};

use sdp8xx::*;

use panic_halt as _;

use cortex_m_rt::entry;

use stm32f0xx_hal::delay::Delay;
use stm32f0xx_hal::serial::Serial;

use nb::block;

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let mut flash = dp.FLASH;
        let mut rcc = dp.RCC.configure().freeze(&mut flash);

        let gpioa = dp.GPIOA.split(&mut rcc);

        let (
            mut i2cbb1_scl,
            mut i2cbb1_sda,
            mut i2cbb2_scl,
            mut i2cbb2_sda,
            mut led,
            mut i2cbb3_scl,
            mut i2cbb3_sda,
            tx_pin,
            rx_pin,
        ) = disable_interrupts(|cs| {
            (
                gpioa.pa0.into_open_drain_output(cs),
                gpioa.pa1.into_open_drain_output(cs),
                gpioa.pa2.into_open_drain_output(cs),
                gpioa.pa3.into_open_drain_output(cs),
                gpioa.pa5.into_push_pull_output(cs),
                gpioa.pa6.into_open_drain_output(cs),
                gpioa.pa7.into_open_drain_output(cs),
                gpioa.pa9.into_alternate_af1(cs),
                gpioa.pa10.into_alternate_af1(cs),
            )
        });
        disable_interrupts(|cs| {
            i2cbb1_scl.internal_pull_up(cs, true);
            i2cbb1_sda.internal_pull_up(cs, true);

            i2cbb2_scl.internal_pull_up(cs, true);
            i2cbb2_sda.internal_pull_up(cs, true);

            i2cbb3_scl.internal_pull_up(cs, true);
            i2cbb3_sda.internal_pull_up(cs, true);
        });

        let mut delay = Delay::new(cp.SYST, &rcc);

        let mut serial = Serial::usart1(dp.USART1, (tx_pin, rx_pin), 9600.bps(), &mut rcc);

        match writeln!(serial, "Detecting sensors") {
            Ok(_) => {}
            Err(_) => {}
        }

        let timer_i2cbb1 = Timer::tim2(dp.TIM2, 200.khz(), &mut rcc);
        let timer_i2cbb2 = Timer::tim3(dp.TIM3, 200.khz(), &mut rcc);
        let timer_i2cbb3 = Timer::tim1(dp.TIM1, 200.khz(), &mut rcc);

        // Configure I2C with 100kHz rate
        let i2cbb1 = bitbang_hal::i2c::I2cBB::new(i2cbb1_scl, i2cbb1_sda, timer_i2cbb1);
        let i2cbb2 = bitbang_hal::i2c::I2cBB::new(i2cbb2_scl, i2cbb2_sda, timer_i2cbb2);
        let i2cbb3 = bitbang_hal::i2c::I2cBB::new(i2cbb3_scl, i2cbb3_sda, timer_i2cbb3);

        let mut sdp8xx1 = Sdp8xx::new(i2cbb1, 0x25, delay.clone());
        let mut sdp8xx2 = Sdp8xx::new(i2cbb2, 0x25, delay.clone());
        let mut sdp8xx3 = Sdp8xx::new(i2cbb3, 0x25, delay.clone());

        let mut arr: [f32; 3] = [0f32; 3];
        loop {
            if let Ok(m) = sdp8xx1.trigger_differential_pressure_sample() {
                arr[0] = m.get_differential_pressure();
            }
            if let Ok(m) = sdp8xx2.trigger_differential_pressure_sample() {
                arr[1] = m.get_differential_pressure();
            }
            if let Ok(m) = sdp8xx3.trigger_differential_pressure_sample() {
                arr[2] = m.get_differential_pressure();
            }

            let view = &arr as *const _ as *const u8;
            let slice =
                unsafe { core::slice::from_raw_parts(view, core::mem::size_of::<[f32; 3]>()) };
            for byte in slice {
                block!(serial.write(*byte)).ok();
            }
            block!(serial.write(b'\n')).ok();

            let _ = led.set_high();
            delay.delay_ms(100u32);
            let _ = led.set_low();
            delay.delay_ms(100u32);
        }
    }
    loop {}
}
