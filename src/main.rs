#![no_std]
#![no_main]

use panic_halt as _;

#[cfg(feature = "println_debug")]
use rtt_target::{rprintln, rtt_init_print};

use stm32f0xx_hal::timers::Timer;

use stm32f0xx_hal::{pac, prelude::*};

use cortex_m::{interrupt::free as disable_interrupts, Peripherals};

use sdp8xx::*;

use panic_halt as _;

use cortex_m_rt::entry;

use stm32f0xx_hal::{delay::Delay};

#[entry]
fn main() -> ! {
    #[cfg(feature = "println_debug")]
    rtt_init_print!();

    if let (Some(dp), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let mut flash = dp.FLASH;
        let mut rcc = dp.RCC.configure().freeze(&mut flash);

        // Configure pins for I2C
        let timer_i2cbb1 = Timer::tim2(dp.TIM2, 200.khz(), &mut rcc);
        let timer_i2cbb2 = Timer::tim3(dp.TIM3, 200.khz(), &mut rcc);
        let timer_i2cbb3 = Timer::tim6(dp.TIM6, 200.khz(), &mut rcc);

        let mut delay = Delay::new(cp.SYST, &rcc);

        let gpioa = dp.GPIOA.split(&mut rcc);

        let gpiob = dp.GPIOB.split(&mut rcc);

        let gpiof = dp.GPIOF.split(&mut rcc);

        let (mut i2cbb1_sda, mut i2cbb1_scl, mut i2cbb2_sda, mut i2cbb2_scl, pa4, mut led, mut i2cbb3_sda, mut i2cbb3_scl, i2c1_scl, i2c1_sda) = disable_interrupts(|cs| {
            (
                gpioa.pa0.into_open_drain_output(cs),
                gpioa.pa1.into_open_drain_output(cs),

                gpioa.pa2.into_open_drain_output(cs),
                gpioa.pa3.into_open_drain_output(cs),

                gpioa.pa4.into_push_pull_output(cs),

                gpioa.pa5.into_push_pull_output(cs),

                gpioa.pa6.into_open_drain_output(cs),
                gpioa.pa7.into_open_drain_output(cs),

                gpioa.pa9.into_alternate_af1(cs),
                gpioa.pa10.into_alternate_af1(cs)
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

        // Configure I2C with 100kHz rate
        let i2cbb1 = bitbang_hal::i2c::I2cBB::new(i2cbb1_scl, i2cbb1_sda, timer_i2cbb1);
        let i2cbb2 = bitbang_hal::i2c::I2cBB::new(i2cbb2_scl, i2cbb2_sda, timer_i2cbb2);
        let i2cbb3 = bitbang_hal::i2c::I2cBB::new(i2cbb3_scl, i2cbb3_sda, timer_i2cbb3);

        let mut sdp8xx1 = Sdp8xx::new(i2cbb1, 0x25, delay.clone());
        let mut sdp8xx2 = Sdp8xx::new(i2cbb2, 0x25, delay.clone());
        let mut sdp8xx3 = Sdp8xx::new(i2cbb3, 0x25, delay.clone());

        #[cfg(feature = "println_debug")]
        rprintln!("Detecting sensors");
        if let Ok(product_id) = sdp8xx1.read_product_id() {
            #[cfg(feature = "println_debug")]
            rprintln!("Sensor 1 Product ID: {:x?}", product_id);
        }
        if let Ok(product_id) = sdp8xx2.read_product_id() {
            #[cfg(feature = "println_debug")]
            rprintln!("Sensor 2 Product ID: {:x?}", product_id);
        }
        if let Ok(product_id) = sdp8xx3.read_product_id() {
            #[cfg(feature = "println_debug")]
            rprintln!("Sensor 3 Product ID: {:x?}", product_id);
        }

        loop {
            if let Ok(m) = sdp8xx1.trigger_differential_pressure_sample() {
                #[cfg(feature = "println_debug")]
                rprintln!("1: {:?}", m);
            }
            if let Ok(m) = sdp8xx2.trigger_differential_pressure_sample() {
                #[cfg(feature = "println_debug")]
                rprintln!("2: {:?}", m);
            }
            if let Ok(m) = sdp8xx3.trigger_differential_pressure_sample() {
                #[cfg(feature = "println_debug")]
                rprintln!("3: {:?}", m);
            }
            let _ = led.set_high();
            delay.delay_ms(500u32);
            let _ = led.set_low();
            delay.delay_ms(500u32);
        }
    }
    loop {}
}
