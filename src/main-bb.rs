#![no_std]
#![no_main]

use pac::TIM2;
use panic_halt as _;

use rtt_target::{rprintln, rtt_init_print};

use stm32f0xx_hal::timers::Timer;

use stm32f0xx_hal::gpio::gpiob::{PB9, PB8};

use stm32f0xx_hal::{
    pac,
    prelude::*,
};

use cortex_m::{interrupt::free as disable_interrupts, Peripherals};

use bitbang_hal::{self, i2c::I2cBB};

use sdp8xx::*;
use sdp8xx::states::*;

use panic_halt as _;

use cortex_m_rt::entry;

use stm32f0xx_hal::{delay::Delay, prelude::*};

#[entry]
fn main() -> ! {
    rtt_init_print!();

    if let (Some(dp), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let mut flash = dp.FLASH;
        let mut rcc = dp.RCC.configure().freeze(&mut flash);

        // Configure pins for I2C
        let timer_i2cbb1 = Timer::tim2(dp.TIM2, 50.khz(), &mut rcc);

        let mut delay = Delay::new(cp.SYST, &rcc);

        let gpiob = dp.GPIOB.split(&mut rcc);

        let (scl, sda) = disable_interrupts(|cs| {
            (
            gpiob.pb8.into_open_drain_output(cs),
            gpiob.pb9.into_open_drain_output(cs),
            )
        });

        // Configure I2C with 100kHz rate
        let i2c = bitbang_hal::i2c::I2cBB::new(scl, sda, timer_i2cbb1);

        let mut sdp8xx = Sdp8xx::new(i2c, 0x25, delay.clone());

        if let Ok(product_id) = sdp8xx.read_product_id() {
            rprintln!("Product ID: {:x?}", product_id);
        } else {
            rprintln!("Failed to read product ID!");
        }
        delay.delay_ms(1000u32);

        //rprintln!("{:?}", counter);
        /*
        rprintln!("Sending sensor to sleep.");
        let woken_up = if let Ok(sleeping) = sdp8xx.go_to_sleep() {
            delay.delay_ms(100u16);
            sleeping.wake_up()
        } else {
            rprintln!("Failed to send sensor to sleep!");
            loop {}
        };
        let mut sdp8xx = match woken_up {
            Ok(s) => s,
            Err(e) => {
                rprintln!("Failed to wake up sensor! {:?}", e);
                loop {}
            }
        };
        */

        rprintln!("Taking 10 triggered samples");
        for _ in 0..=10 {
            if let Ok(m) = sdp8xx.read_sample_triggered() {
                rprintln!("{:?}", m);
            } else {
                rprintln!("Error!");
            }
            delay.delay_ms(1000u32);
        }

        let mut sdp_sampling = match sdp8xx.start_sampling_differential_pressure(true) {
            Ok(s) => s,
            Err(e) => {
                rprintln!("{:?}", e);
                loop {}
            }
        };
        for _ in 0..=50 {
            delay.delay_ms(100u16);
            let result = sdp_sampling.read_continuous_sample();
            match result {
                Ok(r) => rprintln!("{:?}", r),
                Err(e) => rprintln!("Error while getting result: {:?}", e),
            }
        }
        let mut idle_sensor = sdp_sampling.stop_sampling().unwrap();
        loop {
            if let Ok(m) = idle_sensor.read_sample_triggered() {
                rprintln!("{:?}", m);
            } else {
                rprintln!("Error!");
            }
            delay.delay_ms(1000u32);
        }
    }
    loop {}
}
