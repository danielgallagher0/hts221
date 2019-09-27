//! Prints one reading each of humidity and temperature to the OpenOCD console using semihosting.
//!
//! This example shows how to:
//! * Access the first I2C bus on an STM32F30x chip
//! * Use that I2C handle to create an HTS221 handle
//! * Read the humidity and temperature from the HTS221

#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate cortex_m_semihosting;
extern crate embedded_hal;
extern crate hts221;
extern crate stm32f30x_hal as hal;

use core::fmt::Write;
use cortex_m_semihosting::hio;
use hal::flash::FlashExt;
use hal::gpio::GpioExt;
use hal::rcc::RccExt;
use hal::time::U32Ext;

#[inline(never)]
fn main() {
    cortex_m::interrupt::free(|_cs| {
        // Enable I2C1
        let peripherals = hal::stm32f30x::Peripherals::take().unwrap();

        let mut rcc = peripherals.RCC.constrain();
        let mut gpiob = peripherals.GPIOB.split(&mut rcc.ahb);
        let scl = gpiob.pb8.into_af4(&mut gpiob.moder, &mut gpiob.afrh);
        let sda = gpiob.pb9.into_af4(&mut gpiob.moder, &mut gpiob.afrh);
        let mut i2c = hal::i2c::I2c::i2c1(
            peripherals.I2C1,
            (scl, sda),
            50.khz(),
            rcc.cfgr.freeze(&mut peripherals.FLASH.constrain().acr),
            &mut rcc.apb1,
        );

        let mut hts221 = hts221::Builder::new()
            .with_avg_t(hts221::AvgT::Avg256)
            .with_avg_h(hts221::AvgH::Avg512)
            .build(&mut i2c)
            .unwrap();

        let mut stdout = hio::hstdout().unwrap();
        loop {
            match hts221.status(&mut i2c) {
                Ok(status) => {
                    if status.humidity_data_available() && status.temperature_data_available() {
                        break;
                    }
                }
                Err(_) => writeln!(stdout, "Could not get status").unwrap(),
            }
        }

        let humidity_x2 = hts221.humidity_x2(&mut i2c).unwrap();
        let temperature_x8 = hts221.temperature_x8(&mut i2c).unwrap();
        writeln!(
            stdout,
            "rH = {}.{}%",
            humidity_x2 >> 1,
            5 * humidity_x2 & 0b1
        )
        .unwrap();
        writeln!(
            stdout,
            "Temp = {}.{} deg C",
            temperature_x8 >> 3,
            125 * temperature_x8 & 0b111
        )
        .unwrap();
    });
}
