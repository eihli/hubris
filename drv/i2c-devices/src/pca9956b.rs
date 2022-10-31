// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Driver for the PCA9956B LED driver

use core::convert::TryInto;

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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum LedErr {
    NoError = 0b00,
    ShortCircuit = 0b01,
    OpenCircuit = 0b10,
    Invalid = 0b11,
}

impl Default for LedErr {
    fn default() -> Self {
        LedErr::NoError
    }
}

impl From<u8> for LedErr {
    fn from(i: u8) -> Self {
        match i {
            0 => LedErr::NoError,
            1 => LedErr::ShortCircuit,
            2 => LedErr::OpenCircuit,
            _ => LedErr::Invalid,
        }
    }
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct LedErrSummary {
    overtemp: bool,
    short_circuit: u8,
    open_circuit: u8,
    invalid: u8,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Pca9956BErrorState {
    led_errors: [LedErr; NUM_LEDS],
    overtemp: bool,
}

impl Pca9956BErrorState {
    pub fn summary(&self) -> LedErrSummary {
        let mut summary = LedErrSummary {
            overtemp: self.overtemp,
            ..Default::default()
        };

        for err in self.led_errors {
            if err == LedErr::OpenCircuit {
                summary.open_circuit += 1;
            } else if err == LedErr::ShortCircuit {
                summary.short_circuit += 1;
            } else if err == LedErr::Invalid {
                summary.invalid += 1;
            }
        }

        summary
    }
}

/// Auto-increment flag is Bit 7 of the control register. Bits 6..0 are address.
const CTRL_AUTO_INCR: u8 = 1 << 7;
/// The MODE2 OVERTEMP bit indicates if an overtempature condition has occurred
const MODE2_OVERTEMP: u8 = 1 << 7;
/// The MODE2 ERROR bit indicates if any error conditions are in EFLAGn
const MODE2_ERROR: u8 = 1 << 6;
/// The MODE2 CLRERR bit clears all error conditions in EFLAGn
const MODE2_CLRERR: u8 = 1 << 4;

pub struct Pca9956B {
    device: I2cDevice,
}

pub const NUM_LEDS: usize = 24;

#[derive(Copy, Clone, Debug, PartialEq)]
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

    fn read_buffer(&self, reg: Register, buf: &mut [u8]) -> Result<(), Error> {
        self.device
            .read_reg_into((reg as u8) | CTRL_AUTO_INCR, buf)
            .map_err(|code| Error::I2cError(code))?;

        Ok(())
    }

    fn write_reg(&self, reg: Register, val: u8) -> Result<(), Error> {
        let buffer = [reg as u8, val];
        self.device
            .write(&buffer)
            .map_err(|code| Error::I2cError(code))
    }

    fn write_buffer(&self, reg: Register, buf: &[u8]) -> Result<(), Error> {
        let mut data: [u8; 25] = [0; 25];
        data[0] = (reg as u8) | CTRL_AUTO_INCR;
        data[1..=buf.len()].copy_from_slice(buf);

        self.device
            .write(&data[..=buf.len()])
            .map_err(|code| Error::I2cError(code))
    }

    pub fn set_iref_all(&self, val: u8) -> Result<(), Error> {
        self.write_reg(Register::IREFALL, val)
    }

    pub fn set_pwm_all(&self, val: u8) -> Result<(), Error> {
        self.write_reg(Register::PWMALL, val)
    }

    pub fn set_a_led_pwm(&self, led: u8, val: u8) -> Result<(), Error> {
        if led >= NUM_LEDS as u8 {
            return Err(Error::InvalidLED(led));
        }
        let reg = FromPrimitive::from_u8((Register::PWM0 as u8) + led).unwrap();
        self.write_reg(reg, val)
    }

    pub fn set_all_led_pwm(&self, vals: &[u8]) -> Result<(), Error> {
        if vals.len() > NUM_LEDS {
            return Err(Error::InvalidLED(
                vals.len().try_into().unwrap_or(0xFF),
            ));
        }
        let reg = FromPrimitive::from_u8(Register::PWM0 as u8).unwrap();
        self.write_buffer(reg, &vals)
    }

    pub fn check_for_errors(
        &self,
    ) -> Result<Option<Pca9956BErrorState>, Error> {
        // Get MODE2 register, which holds OVERTEMP and ERROR information
        let mode2 = self.read_reg(Register::MODE2)?;
        let overtemp = (mode2 & MODE2_OVERTEMP) != 0;
        let error = (mode2 & MODE2_ERROR) != 0;

        // Check for error condition, go get EFLAGn registers,
        // clearing them afterwards
        if overtemp || error {
            let mut err_state = Pca9956BErrorState {
                ..Default::default()
            };
            err_state.overtemp = overtemp;

            let mut eflags: [u8; 6] = [0; 6];
            self.read_buffer(Register::EFLAG0, &mut eflags)?;

            for i in 0..eflags.len() {
                let eflag = eflags[i];
                for j in 0..=3 {
                    err_state.led_errors[(i * 4) + j] =
                        LedErr::from(eflag & (0b11 << j * 2));
                }
            }

            self.write_reg(Register::MODE2, mode2 & !MODE2_CLRERR)?;
            Ok(Some(err_state))
        } else {
            Ok(None)
        }
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
