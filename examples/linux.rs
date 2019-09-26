//! Prints one reading each of humidity and temperature to the console.
//!
//! This example shows how to:
//! * Create an I2C handle
//! * Use that I2C handle to create an HTS221 handle
//! * Read the humidity and temperature from the HTS221

extern crate embedded_hal;
extern crate hts221;
extern crate linux_embedded_hal;

use linux_embedded_hal::I2cdev;

fn main() -> Result<(), linux_embedded_hal::i2cdev::linux::LinuxI2CError> {
    let mut i2c = I2cdev::new("/dev/i2c-1")?;
    let mut hts221 = hts221::Builder::new()
        .with_avg_t(hts221::AvgT::Avg256)
        .with_avg_h(hts221::AvgH::Avg512)
        .build(&mut i2c)?;

    loop {
        match hts221.status(&mut i2c) {
            Ok(status) => {
                if status.humidity_data_available() && status.temperature_data_available() {
                    break;
                }
            }
            Err(_) => println!("Could not get status"),
        }
    }

    let humidity_x2 = hts221.humidity_x2(&mut i2c)?;
    let temperature_x8 = hts221.temperature_x8(&mut i2c)?;
    println!("rH = {}.{}%", humidity_x2 >> 1, 5 * humidity_x2 & 0b1);
    println!(
        "Temp = {}.{} deg C",
        temperature_x8 >> 3,
        125 * temperature_x8 & 0b111
    );

    Ok(())
}
