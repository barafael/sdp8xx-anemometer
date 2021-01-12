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

                gpioa.pa5.into_open_drain_output(cs),

                gpioa.pa6.into_open_drain_output(cs),
                gpioa.pa7.into_open_drain_output(cs),

                gpioa.pa9.into_alternate_af1(cs),
                gpioa.pa10.into_alternate_af1(cs)
            )
        });
        disable_interrupts(|cs| {
            i2cbb1_scl.internal_pull_up(cs, true);
            i2cbb1_sda.internal_pull_up(cs, true);
        });

        // Configure I2C with 100kHz rate
        let i2c = bitbang_hal::i2c::I2cBB::new(i2cbb1_scl, i2cbb1_sda, timer_i2cbb1);

        let mut sdp8xx = Sdp8xx::new(i2c, 0x25, delay.clone());

        if let Ok(product_id) = sdp8xx.read_product_id() {
            #[cfg(feature = "println_debug")]
            rprintln!("Product ID: {:x?}", product_id);
        } else {
            #[cfg(feature = "println_debug")]
            rprintln!("Failed to read product ID!");
        }
        delay.delay_ms(1000u32);

        #[cfg(feature = "println_debug")]
        rprintln!("Sending sensor to sleep.");
        let woken_up = if let Ok(sleeping) = sdp8xx.go_to_sleep() {
            delay.delay_ms(1000u16);
            sleeping.wake_up_poll()
        } else {
            #[cfg(feature = "println_debug")]
            rprintln!("Failed to send sensor to sleep!");
            loop {}
        };
        let mut sdp8xx = match woken_up {
            Ok(s) => s,
            Err(e) => {
                #[cfg(feature = "println_debug")]
                rprintln!("Failed to wake up sensor! {:?}", e);
                loop {}
            }
        };
        #[cfg(feature = "println_debug")]
        rprintln!("Woke up sensor!");

        //rprintln!("Taking 10 triggered samples");
        for _ in 0..=10 {
            if let Ok(m) = sdp8xx.read_sample_triggered() {
                #[cfg(feature = "println_debug")]
                rprintln!("{:?}", m);
            } else {
                #[cfg(feature = "println_debug")]
                rprintln!("Error!");
            }
            delay.delay_ms(1000u32);
        }

        let mut sdp_sampling = match sdp8xx.start_sampling_differential_pressure(true) {
            Ok(s) => s,
            Err(e) => {
                #[cfg(feature = "println_debug")]
                rprintln!("{:?}", e);
                loop {}
            }
        };
        for _ in 0..=50 {
            delay.delay_ms(100u16);
            let result = sdp_sampling.read_continuous_sample();
            match result {
                Ok(r) => {
                    #[cfg(feature = "println_debug")]
                    rprintln!("{:?}", r);
                }
                Err(e) => {
                    #[cfg(feature = "println_debug")]
                    rprintln!("Error while getting result: {:?}", e);
                }
            }
        }
        let mut idle_sensor = sdp_sampling.stop_sampling().unwrap();
        loop {
            if let Ok(m) = idle_sensor.read_sample_triggered() {
                #[cfg(feature = "println_debug")]
                rprintln!("{:?}", m);
            } else {
                #[cfg(feature = "println_debug")]
                rprintln!("Error!");
            }
            let _ = led.set_high();
            delay.delay_ms(1000u32);
            let _ = led.set_low();
            delay.delay_ms(1000u32);
        }
    }
    loop {}
}
