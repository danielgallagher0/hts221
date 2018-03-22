# `hts221`

> I²C API for the HTS221 humidity and temperature sensor

# [Documentation](https://docs.rs/hts221)

# Example

To use the driver, you must have a concrete implementation of the
[embedded-hal](https://crates.io/crates/embedded-hal) traits.  This example uses
[linux-embedded-hal](http://crates.io/linux-embedded-hal):

``` rust
let i2c = I2cdev::new("/dev/i2c-1")?;
let mut hts221 = hts221::Builder::new(i2c).build()?;

let rh = hts221.humidity_x2()?;
let deg_c = hts221.temperature_x8()?;
println!("rH = {}%", rh as f32 / 2.0);
println!("Temp = {} deg C", deg_c as f32 / 8.0);
```

# License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
