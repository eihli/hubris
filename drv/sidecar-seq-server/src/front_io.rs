// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use crate::*;
use drv_i2c_devices::{at24csw080::At24Csw080, pca9956b::Error, pca9956b::Pca9956B, Validate};
use drv_sidecar_front_io::{controller::FrontIOController, phy_smi::PhySmi};

#[allow(dead_code)]
pub(crate) struct FrontIOBoard {
    pub fruid: I2cDevice,
    pub led_controllers: [Pca9956B; 2],
    pub controllers: [FrontIOController; 2],
    pub state_reset: bool,
    fpga_task: userlib::TaskId,
    auxflash_task: userlib::TaskId,
}

#[derive(Debug)]
pub enum FrontIOError {
    FpgaError,
    I2cError,
}

impl From<Error> for FrontIOError {
    fn from(_e: Error) -> Self {
        FrontIOError::I2cError
    }
}

impl From<FpgaError> for FrontIOError {
    fn from(_e: FpgaError) -> Self {
        FrontIOError::FpgaError
    }
}

/// Default LED Current
///
/// This will get written into the PCA9956B IREFALL register. The goal is to
/// make these LEDs look as close to Gimlet CEM attention LEDs as possible.
/// As of build C, Gimlet is pulling 50mA through those LEDs. From the PCA9956B
/// datasheet, the calculus is I = IREFALL/256 * (900mV/Rext) * 1/4. Rext (R47
///  & R48 on QSFP Front IO) is a 1K, so a bit of math results in a desired
/// IREF value of d222 (hDE).
const DEFAULT_LED_CURRENT: u8 = 222;

/// Default LED PWM
///
/// This can be used to adjust LED duty cycle. The math here is simple, just
/// PWM/256.
const DEFAULT_LED_PWM: u8 = 255;

const SYSTEM_LED_IDX: u8 = 23;

impl FrontIOBoard {
    pub fn new(
        fpga_task: userlib::TaskId,
        i2c_task: userlib::TaskId,
        auxflash_task: userlib::TaskId,
    ) -> Self {
        Self {
            fruid: i2c_config::devices::at24csw080_front_io(i2c_task)[0],
            led_controllers: [
                Pca9956B::new(
                    &i2c_config::devices::pca9956b_left_front_io(i2c_task)[0],
                ),
                Pca9956B::new(
                    &i2c_config::devices::pca9956b_right_front_io(i2c_task)[0],
                ),
            ],
            controllers: [
                FrontIOController::new(fpga_task, 0),
                FrontIOController::new(fpga_task, 1),
            ],
            state_reset: false,
            fpga_task,
            auxflash_task,
        }
    }

    pub fn phy_smi(&self) -> PhySmi {
        PhySmi::new(self.fpga_task)
    }

    pub fn present(&self) -> bool {
        At24Csw080::validate(&self.fruid).unwrap_or(false)
    }

    pub fn init(&mut self) -> Result<bool, FrontIOError> {
        let mut controllers_ready = true;

        for (i, controller) in self.controllers.iter_mut().enumerate() {
            let state = controller.await_fpga_ready(25)?;
            let mut ident;
            let mut ident_valid = false;
            let mut checksum;
            let mut checksum_valid = false;

            if state == DeviceState::RunningUserDesign {
                (ident, ident_valid) = controller.ident_valid()?;
                ringbuf_entry!(Trace::FrontIOControllerIdent {
                    fpga_id: i,
                    ident
                });

                (checksum, checksum_valid) = controller.checksum_valid()?;
                ringbuf_entry!(Trace::FrontIOControllerChecksum {
                    fpga_id: i,
                    checksum,
                    expected: FrontIOController::short_checksum(),
                });

                if !ident_valid || !checksum_valid {
                    // Attempt to correct the invalid IDENT by reloading the
                    // bitstream.
                    controller.fpga_reset()?;
                }
            }

            if ident_valid && checksum_valid {
                ringbuf_entry!(Trace::SkipLoadingFrontIOControllerBitstream {
                    fpga_id: i
                });
            } else {
                ringbuf_entry!(Trace::LoadingFrontIOControllerBitstream {
                    fpga_id: i
                });

                if let Err(e) = controller.load_bitstream(self.auxflash_task) {
                    ringbuf_entry!(Trace::FpgaBitstreamError(
                        u32::try_from(e).unwrap()
                    ));
                    return Err(FrontIOError::from(e));
                }

                (ident, ident_valid) = controller.ident_valid()?;
                ringbuf_entry!(Trace::FrontIOControllerIdent {
                    fpga_id: i,
                    ident
                });

                controller.write_checksum()?;
                (checksum, checksum_valid) = controller.checksum_valid()?;
                ringbuf_entry!(Trace::FrontIOControllerChecksum {
                    fpga_id: i,
                    checksum,
                    expected: FrontIOController::short_checksum(),
                });
            }

            controllers_ready &= ident_valid & checksum_valid;
        }

        Ok(controllers_ready)
    }

    pub fn enable_led_controllers(&mut self) -> Result<(), FrontIOError> {
        for (_i, controller) in self.controllers.iter_mut().enumerate() {
            controller.enable_led_controller()?;
        }

        for (_i, led_controller) in self.led_controllers.iter_mut().enumerate()
        {
            led_controller.set_iref_all(DEFAULT_LED_CURRENT)?;
        }

        Ok(())
    }

    pub fn turn_on_system_led(&mut self) -> Result<(), FrontIOError> {
        self.led_controllers[0].set_led_pwm(SYSTEM_LED_IDX, DEFAULT_LED_PWM)?;
        Ok(())
    }
}
