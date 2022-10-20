// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Driver for the PCA9956B LED driver

use crate::Validate;
use drv_i2c_api::*;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

#[allow(dead_code)]
#[derive(Copy, Clone, Debug, FromPrimitive, Eq, PartialEq)]
pub enum Register {
    MODE1 = 0x00,
    MODE2 = 0x01,
    LEDOUT0 = 0x02,
    LEDOUT1 = 0x03,
    LEDOUT2 = 0x04,
    LEDOUT3 = 0x05,
    LEDOUT4 = 0x06,
    LEDOUT5 = 0x07,
    GRPPWM = 0x08,
    GRPFREQ = 0x09,
    PWM0 = 0x0A,
    PWM1 = 0x0B,
    PWM2 = 0x0C,
    PWM3 = 0x0D,
    PWM4 = 0x0E,
    PWM5 = 0x0F,
    PWM6 = 0x10,
    PWM7 = 0x11,
    PWM8 = 0x12,
    PWM9 = 0x13,
    PWM10 = 0x14,
    PWM11 = 0x15,
    PWM12 = 0x16,
    PWM13 = 0x17,
    PWM14 = 0x18,
    PWM15 = 0x19,
    PWM16 = 0x1A,
    PWM17 = 0x1B,
    PWM18 = 0x1C,
    PWM19 = 0x1D,
    PWM20 = 0x1E,
    PWM21 = 0x1F,
    PWM22 = 0x20,
    PWM23 = 0x21,
    IREF0 = 0x22,
    IREF1 = 0x23,
    IREF2 = 0x24,
    IREF3 = 0x25,
    IREF4 = 0x26,
    IREF5 = 0x27,
    IREF6 = 0x28,
    IREF7 = 0x29,
    IREF8 = 0x2A,
    IREF9 = 0x2B,
    IREF10 = 0x2C,
    IREF11 = 0x2D,
    IREF12 = 0x2E,
    IREF13 = 0x2F,
    IREF14 = 0x30,
    IREF15 = 0x31,
    IREF16 = 0x32,
    IREF17 = 0x33,
    IREF18 = 0x34,
    IREF19 = 0x35,
    IREF20 = 0x36,
    IREF21 = 0x37,
    IREF22 = 0x38,
    IREF23 = 0x39,
    OFFSET = 0x3A,
    SUBADR1 = 0x3B,
    SUBADR2 = 0x3C,
    SUBADR3 = 0x3D,
    ALLCALLADR = 0x3E,
    PWMALL = 0x3F,
    IREFALL = 0x40,
    EFLAG0 = 0x41,
    EFLAG1 = 0x42,
    EFLAG2 = 0x43,
    EFLAG3 = 0x44,
    EFLAG4 = 0x45,
    EFLAG5 = 0x46,
}

pub struct Pca9956B {
    device: I2cDevice,
}

const NUM_LEDS: u8 = 24;

#[derive(Debug)]
pub enum Error {
    /// The low-level I2C communication returned an error
    I2cError(ResponseCode),

    /// The LED index is too large
    InvalidLED(u8),
}

impl From<ResponseCode> for Error {
    fn from(err: ResponseCode) -> Self {
        Error::I2cError(err)
    }
}

impl From<Error> for ResponseCode {
    fn from(err: Error) -> Self {
        match err {
            Error::I2cError(code) => code,
            _ => panic!(),
        }
    }
}

impl Pca9956B {
    pub fn new(device: &I2cDevice) -> Self {
        Self { device: *device }
    }

    fn read_reg(&self, reg: Register) -> Result<u8, Error> {
        self.device
            .read_reg::<u8, u8>(reg as u8)
            .map_err(|code| Error::I2cError(code))
    }

    fn write_reg(&self, reg: Register, val: u8) -> Result<(), Error> {
        let buffer = [reg as u8, val];
        self.device
            .write(&buffer)
            .map_err(|code| Error::I2cError(code))
    }

    pub fn set_iref_all(&self, val: u8) -> Result<(), Error> {
        self.write_reg(Register::IREFALL, val)
    }

    pub fn set_pwm_all(&self, val: u8) -> Result<(), Error> {
        self.write_reg(Register::PWMALL, val)
    }

    pub fn set_led_pwm(&self, led: u8, val: u8) -> Result<(), Error> {
        if led >= NUM_LEDS {
            return Err(Error::InvalidLED(led));
        }
        let reg = FromPrimitive::from_u8((Register::PWM0 as u8) + led).unwrap();
        self.write_reg(reg, val)
    }
}

// The PCA9956B does not expose anything like a unique ID or manufacturer code,
// which is the type of information we typically like to validate against.
// MODE2[2:0] are set to read only an initialized to b101, so use that to
// validate.
impl Validate<Error> for Pca9956B {
    fn validate(device: &I2cDevice) -> Result<bool, Error> {
        let mode = Pca9956B::new(device).read_reg(Register::MODE2)?;

        Ok(mode & 0x7 == 0x05)
    }
}
