//! I2C interface to the HTS221.
//!
//! This is a high-level API that provides functions to read humidity and temperature values from
//! the chip, using a blocking I2C communication protocol.
#![no_std]

extern crate embedded_hal as hal;

use core::fmt::Debug;

pub mod device;

#[doc(inline)]
pub use device::av_conf::AvgH;
#[doc(inline)]
pub use device::av_conf::AvgT;
#[doc(inline)]
pub use device::cr1::DataRate;

use hal::blocking::i2c::{Write, WriteRead};

fn clamp<T: PartialOrd>(n: T, min: T, max: T) -> T {
    if n < min {
        min
    } else if n > max {
        max
    } else {
        n
    }
}

/// Interface for the chip.
pub struct HTS221<Comm> {
    comm: Comm,
    calibration: device::Calibration,
}

impl<Comm> HTS221<Comm>
where
    Comm: Write + WriteRead,
{
    /// Returns the current humidity reading, in relative humidity half-percentage points.  To get
    /// the relative humidity as a percent, divide the result by 2.
    pub fn humidity_x2(&mut self) -> Result<u16, <Comm as WriteRead>::Error> {
        let raw = device::HumidityOut::new(&mut self.comm)?.value();
        Ok(self.convert_humidity_x2(raw))
    }

    /// Returns the current temperature reading, in 1/8 degrees Celsius.  To get the temperature in
    /// degrees Celsius, divide the result by 8.
    pub fn temperature_x8(&mut self) -> Result<i16, <Comm as WriteRead>::Error> {
        let raw = device::TemperatureOut::new(&mut self.comm)?.value();
        Ok(self.convert_temperature_x8(raw))
    }

    /// Converts a humidity ADC reading into relative humidity half-percentage points using the
    /// device's calibration.  To get the relative humidity as a percent, divide the result by 2.
    ///
    /// The result is clamped such that it is always within the device's operating range.
    pub fn convert_humidity_x2(&self, raw: i16) -> u16 {
        const MIN_HUMIDITY: u16 = 0;
        const MAX_HUMIDITY: u16 = 100;

        let h_range_x2 = (self.calibration.h1_rh_x2 - self.calibration.h0_rh_x2) as i16;
        let adc_range = self.calibration.h1_t0_out - self.calibration.h0_t0_out;
        let meas = raw - self.calibration.h0_t0_out;

        let humidity_x2 = self.calibration.h0_rh_x2 as u16 +
            (meas as i32 * h_range_x2 as i32 / adc_range as i32) as u16;
        clamp(humidity_x2, MIN_HUMIDITY * 2, MAX_HUMIDITY * 2) as u16
    }

    /// Converts a temperature ADC reading into 1/8 degrees Celsius using the device's calibration.
    /// To get the temperature in degrees Celsius, divide the result by 8.
    ///
    /// The result is clamped such that it is always within the device's operating range.
    pub fn convert_temperature_x8(&self, raw: i16) -> i16 {
        const MIN_TEMPERATURE: i16 = -40;
        const MAX_TEMPERATURE: i16 = 120;

        let t_range_x8 = (self.calibration.t1_deg_c_x8 - self.calibration.t0_deg_c_x8) as i16;
        let adc_range = self.calibration.t1_out - self.calibration.t0_out;
        let meas = raw - self.calibration.t0_out;

        let temperature_x8 = self.calibration.t0_deg_c_x8 as i16 +
            (meas as i32 * t_range_x8 as i32 / adc_range as i32) as i16;
        clamp(temperature_x8, MIN_TEMPERATURE * 8, MAX_TEMPERATURE * 8)
    }

    /// Returns the WHO_AM_I register.
    pub fn who_am_i(&mut self) -> Result<device::WhoAmI, <Comm as WriteRead>::Error> {
        device::WhoAmI::new(&mut self.comm)
    }

    /// Returns the AV_CONF register.
    pub fn av_conf(&mut self) -> Result<device::AvConf, <Comm as WriteRead>::Error> {
        device::AvConf::new(&mut self.comm)
    }

    /// Returns the CTRL_REG1 register.
    pub fn cr1(&mut self) -> Result<device::CtrlReg1, <Comm as WriteRead>::Error> {
        device::CtrlReg1::new(&mut self.comm)
    }

    /// Returns the CTRL_REG2 register.
    pub fn cr2(&mut self) -> Result<device::CtrlReg2, <Comm as WriteRead>::Error> {
        device::CtrlReg2::new(&mut self.comm)
    }

    /// Returns the CTRL_REG3 register.
    pub fn cr3(&mut self) -> Result<device::CtrlReg3, <Comm as WriteRead>::Error> {
        device::CtrlReg3::new(&mut self.comm)
    }

    /// Returns the STATUS register.
    pub fn status(&mut self) -> Result<device::StatusReg, <Comm as WriteRead>::Error> {
        device::StatusReg::new(&mut self.comm)
    }
}

/// Values for block-update mode.
///
/// In default (continuous) mode, the lower and upper parts of the output registers are updated
/// continuously. If it is not certain whether the read will be faster than output data rate, it is
/// recommended to use block-update mode.  In block-update mode, after the reading of the lower
/// (upper) register part, the content of that output register is not updated until the upper
/// (lower) part is read also.  This feature prevents the reading of LSB and MSB related to
/// different samples.
pub enum UpdateMode {
    Block,
    Continuous,
}

/// Polarity options for the data-ready signal.  Value is the polarity of the signal when data is
/// ready.
pub enum Polarity {
    High,
    Low,
}

/// Options for the data ready pin output mode.
pub enum PinMode {
    PushPull,
    OpenDrain,
}

/// Builder for an HTS221 structure.  This builder allows you to configure the chip without needing
/// to access the [device]:device module.
///
/// Defaults are:
/// * Averaged samples - untouched
/// * Powered on
/// * Block update mode
/// * One-shot mode
/// * No boot
/// * Data ready polarity and output mode are unchanged
/// * Data ready interrupt is disabled
pub struct Builder<Comm> {
    comm: Comm,

    avg_t: Option<AvgT>,
    avg_h: Option<AvgH>,

    powered_up: bool,
    update_mode: UpdateMode,
    data_rate: DataRate,

    boot: bool,

    data_ready_polarity: Option<Polarity>,
    data_ready_mode: Option<PinMode>,
    data_ready_enable: bool,
}

pub enum BuildError<W, WR>
where
    W: Write,
    W::Error: Debug,
    WR: WriteRead,
    WR::Error: Debug,
{
    WriteError(W::Error),
    WriteReadError(WR::Error),
}

impl<W, WR> Debug for BuildError<W, WR>
where
    W: Write,
    W::Error: Debug,
    WR: WriteRead,
    WR::Error: Debug,
{
    fn fmt(&self, fmt: &mut core::fmt::Formatter) -> core::fmt::Result {
        match self {
            &BuildError::WriteError(ref e) => write!(fmt, "BuildError::WriteError({:?})", e),
            &BuildError::WriteReadError(ref e) => {
                write!(fmt, "BuildError::WriteReadError({:?})", e)
            }
        }
    }
}

fn from_write_error<W: Write, WR: WriteRead>(e: W::Error) -> BuildError<W, WR>
where
    W::Error: Debug,
    WR::Error: Debug,
{
    BuildError::WriteError(e)
}

fn from_write_read_error<W: Write, WR: WriteRead>(e: WR::Error) -> BuildError<W, WR>
where
    W::Error: Debug,
    WR::Error: Debug,
{
    BuildError::WriteReadError(e)
}

impl<Comm> Builder<Comm>
where
    Comm: Write + WriteRead,
    <Comm as Write>::Error: Debug,
    <Comm as WriteRead>::Error: Debug,
{
    /// Initialize a new Builder for an HTS221 that will use `comm` for all communication.
    pub fn new(comm: Comm) -> Self {
        Self {
            comm: comm,
            avg_t: None,
            avg_h: None,
            powered_up: true,
            update_mode: UpdateMode::Block,
            data_rate: DataRate::OneShot,
            boot: false,
            data_ready_polarity: None,
            data_ready_mode: None,
            data_ready_enable: false,
        }
    }

    /// Configures the number of internal temperature samples that will be averaged into one output
    /// sample.
    pub fn with_avg_t(mut self, avg_t: AvgT) -> Self {
        self.avg_t = Some(avg_t);
        self
    }

    /// Configures the number of internal humidity samples that will be averaged into one output
    /// sample.
    pub fn with_avg_h(mut self, avg_h: AvgH) -> Self {
        self.avg_h = Some(avg_h);
        self
    }

    /// Powers up the device on initialization (default).
    pub fn powered_up(mut self) -> Self {
        self.powered_up = true;
        self
    }

    /// Keeps the device powered down on initialization.
    pub fn powered_down(mut self) -> Self {
        self.powered_up = false;
        self
    }

    /// Sets the update mode on initialization
    pub fn with_update_mode(mut self, mode: UpdateMode) -> Self {
        self.update_mode = mode;
        self
    }

    /// Sets the data rate on initialization
    pub fn with_data_rate(mut self, rate: DataRate) -> Self {
        self.data_rate = rate;
        self
    }

    /// Boots the device (resets stored values) on initialization
    pub fn with_boot(mut self) -> Self {
        self.boot = true;
        self
    }

    /// Does not boot the device on initialization (default)
    pub fn without_boot(mut self) -> Self {
        self.boot = false;
        self
    }

    /// Sets the polarity of the data-ready output pin.
    pub fn with_data_ready_polarity(mut self, polarity: Polarity) -> Self {
        self.data_ready_polarity = Some(polarity);
        self
    }

    /// Sets the output mode of the data-ready output pin.
    pub fn with_data_ready_mode(mut self, mode: PinMode) -> Self {
        self.data_ready_mode = Some(mode);
        self
    }

    /// Enables the data-ready external interrupt pin.
    pub fn with_data_ready_enabled(mut self) -> Self {
        self.data_ready_enable = true;
        self
    }

    /// Disables the data-ready external interrupt pin (default).
    pub fn with_data_ready_disabled(mut self) -> Self {
        self.data_ready_enable = false;
        self
    }

    /// Builds an HTS221 handle using the current builder configuration.  Consumes the builder.
    pub fn build(mut self) -> Result<HTS221<Comm>, BuildError<Comm, Comm>> {
        let self_avg_t = self.avg_t;
        let self_avg_h = self.avg_h;
        let self_powered_up = self.powered_up;
        let self_update_mode = self.update_mode;
        let self_data_rate = self.data_rate;
        let self_boot = self.boot;
        let self_data_ready_polarity = self.data_ready_polarity;
        let self_data_ready_mode = self.data_ready_mode;
        let self_data_ready_enable = self.data_ready_enable;

        let cal = device::Calibration::new(&mut self.comm).map_err(
            from_write_read_error,
        )?;
        let mut hts221 = HTS221::<Comm> {
            comm: self.comm,
            calibration: cal,
        };

        match (self_avg_t, self_avg_h) {
            (Some(avg_t), Some(avg_h)) => {
                let mut av_conf = device::AvConf::new(&mut hts221.comm).map_err(
                    from_write_read_error,
                )?;
                av_conf
                    .modify(&mut hts221.comm, |w| {
                        w.set_temperature_samples_averaged(avg_t);
                        w.set_humidity_samples_averaged(avg_h);
                    })
                    .map_err(from_write_error)?;
            }
            (Some(avg_t), None) => {
                let mut av_conf = device::AvConf::new(&mut hts221.comm).map_err(
                    from_write_read_error,
                )?;
                av_conf
                    .modify(&mut hts221.comm, |w| {
                        w.set_temperature_samples_averaged(avg_t);
                    })
                    .map_err(from_write_error)?;
            }
            (None, Some(avg_h)) => {
                let mut av_conf = device::AvConf::new(&mut hts221.comm).map_err(
                    from_write_read_error,
                )?;
                av_conf
                    .modify(&mut hts221.comm, |w| {
                        w.set_humidity_samples_averaged(avg_h);
                    })
                    .map_err(from_write_error)?;
            }
            (None, None) => (),
        }

        {
            let mut cr1 = device::CtrlReg1::new(&mut hts221.comm).map_err(
                from_write_read_error,
            )?;
            cr1.modify(&mut hts221.comm, |w| {
                if self_powered_up {
                    w.power_up();
                } else {
                    w.power_down();
                }

                match self_update_mode {
                    UpdateMode::Block => w.set_block_update(),
                    UpdateMode::Continuous => w.set_continuous_update(),
                }

                w.set_data_rate(self_data_rate);
            }).map_err(from_write_error)?;
        }

        if self_boot {
            device::CtrlReg2::new(&mut hts221.comm)
                .map_err(from_write_read_error)?
                .modify(&mut hts221.comm, |w| { w.boot(); })
                .map_err(from_write_error)?;
        }

        {
            let mut cr3 = device::CtrlReg3::new(&mut hts221.comm).map_err(
                from_write_read_error,
            )?;
            cr3.modify(&mut hts221.comm, |w| {
                match self_data_ready_polarity {
                    Some(Polarity::High) => w.data_ready_high(),
                    Some(Polarity::Low) => w.data_ready_low(),
                    None => (),
                }
                match self_data_ready_mode {
                    Some(PinMode::PushPull) => w.data_ready_push_pull(),
                    Some(PinMode::OpenDrain) => w.data_ready_open_drain(),
                    None => (),
                }
                if self_data_ready_enable {
                    w.data_ready_enable();
                } else {
                    w.data_ready_disable();
                }
            }).map_err(from_write_error)?;
        }

        Ok(hts221)
    }
}
