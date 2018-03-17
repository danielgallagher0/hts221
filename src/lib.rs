//! I2C interface to the HTS221.
//!
//! This is a high-level API that provides functions to read humidity and temperature values from
//! the chip, using a blocking I2C communication protocol.
#![no_std]

pub mod device;

#[doc(inline)]
pub use device::I2C;
#[doc(inline)]
pub use device::av_conf::AvgH;
#[doc(inline)]
pub use device::av_conf::AvgT;
#[doc(inline)]
pub use device::cr1::DataRate;

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

impl<Comm: I2C> HTS221<Comm> {
    /// Returns the current humidity reading, in relative humidity half-percentage points.  To get
    /// the relative humidity as a percent, divide the result by 2.
    pub fn humidity_x2(&mut self) -> Result<u16, Comm::Error> {
        let raw = device::HumidityOut::new(&mut self.comm)?.value();
        Ok(self.convert_humidity_x2(raw))
    }

    /// Returns the current temperature reading, in 1/8 degrees Celsius.  To get the temperature in
    /// degrees Celsius, divide the result by 8.
    pub fn temperature_x8(&mut self) -> Result<i16, Comm::Error> {
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
    pub fn who_am_i(&mut self) -> Result<device::WhoAmI, Comm::Error> {
        device::WhoAmI::new(&mut self.comm)
    }

    /// Returns the AV_CONF register.
    pub fn av_conf(&mut self) -> Result<device::AvConf, Comm::Error> {
        device::AvConf::new(&mut self.comm)
    }

    /// Returns the CTRL_REG1 register.
    pub fn cr1(&mut self) -> Result<device::CtrlReg1, Comm::Error> {
        device::CtrlReg1::new(&mut self.comm)
    }

    /// Returns the CTRL_REG2 register.
    pub fn cr2(&mut self) -> Result<device::CtrlReg2, Comm::Error> {
        device::CtrlReg2::new(&mut self.comm)
    }

    /// Returns the CTRL_REG3 register.
    pub fn cr3(&mut self) -> Result<device::CtrlReg3, Comm::Error> {
        device::CtrlReg3::new(&mut self.comm)
    }

    /// Returns the STATUS register.
    pub fn status(&mut self) -> Result<device::StatusReg, Comm::Error> {
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

impl<Comm: I2C> Builder<Comm> {
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
    pub fn build(mut self) -> Result<HTS221<Comm>, <Comm as I2C>::Error> {
        match (self.avg_t, self.avg_h) {
            (Some(avg_t), Some(avg_h)) => {
                let mut av_conf = device::AvConf::new(&mut self.comm)?;
                av_conf.modify(&mut self.comm, |w| {
                    w.set_temperature_samples_averaged(avg_t);
                    w.set_humidity_samples_averaged(avg_h);
                })?;
            }
            (Some(avg_t), None) => {
                let mut av_conf = device::AvConf::new(&mut self.comm)?;
                av_conf.modify(&mut self.comm, |w| {
                    w.set_temperature_samples_averaged(avg_t);
                })?;
            }
            (None, Some(avg_h)) => {
                let mut av_conf = device::AvConf::new(&mut self.comm)?;
                av_conf.modify(&mut self.comm, |w| {
                    w.set_humidity_samples_averaged(avg_h);
                })?;
            }
            (None, None) => (),
        }

        {
            let powered_up = self.powered_up;
            let update_mode = self.update_mode;
            let data_rate = self.data_rate;
            let mut cr1 = device::CtrlReg1::new(&mut self.comm)?;
            cr1.modify(&mut self.comm, |w| {
                if powered_up {
                    w.power_up();
                } else {
                    w.power_down();
                }

                match update_mode {
                    UpdateMode::Block => w.set_block_update(),
                    UpdateMode::Continuous => w.set_continuous_update(),
                }

                w.set_data_rate(data_rate);
            })?;
        }

        if self.boot {
            device::CtrlReg2::new(&mut self.comm)?.modify(
                &mut self.comm,
                |w| { w.boot(); },
            )?;
        }

        {
            let data_ready_polarity = self.data_ready_polarity;
            let data_ready_mode = self.data_ready_mode;
            let data_ready_enable = self.data_ready_enable;
            let mut cr3 = device::CtrlReg3::new(&mut self.comm)?;
            cr3.modify(&mut self.comm, |w| {
                match data_ready_polarity {
                    Some(Polarity::High) => w.data_ready_high(),
                    Some(Polarity::Low) => w.data_ready_low(),
                    None => (),
                }
                match data_ready_mode {
                    Some(PinMode::PushPull) => w.data_ready_push_pull(),
                    Some(PinMode::OpenDrain) => w.data_ready_open_drain(),
                    None => (),
                }
                if data_ready_enable {
                    w.data_ready_enable();
                } else {
                    w.data_ready_disable();
                }
            })?;
        }

        let cal = device::Calibration::new(&mut self.comm)?;
        Ok(HTS221::<Comm> {
            comm: self.comm,
            calibration: cal,
        })
    }
}
