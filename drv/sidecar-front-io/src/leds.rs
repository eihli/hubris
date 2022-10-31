// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.
use drv_i2c_api::I2cDevice;
use drv_i2c_devices::pca9956b::{
    Error, LedErrSummary, Pca9956B, Pca9956BErrorState,
};

pub struct Leds {
    controllers: [Pca9956B; 2],
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

// There are two LED controllers, each controlling the LEDs on either the left
// or right of the board.
#[derive(PartialEq)]
pub enum LedController {
    Left = 0,
    Right = 1,
}

// The necessary information to control a given LED.
struct LedLocation {
    controller: LedController,
    output: u8,
}

#[derive(Copy, Clone, Default, PartialEq, Eq)]
pub struct FullErrorSummary {
    pub left: LedErrSummary,
    pub right: LedErrSummary,
}

/// System LED IDX
///
/// Index of the System LED in the LED_MAP
const SYSTEM_LED_IDX: usize = 32;

/// LED Map
///
/// Index 0 represents port 0, 1 to port 1, and so on. Following the ports, the
/// system LED is placed at index 32 (exposed as SYSTEM_LED_IDX above).
/// The 32 QSFP ports are mapped between two PCA9956Bs, split between left and
/// right since each can only drive 24 LEDs. The System LED is wired to the left
/// LED driver.
const LED_MAP: [LedLocation; 33] = [
    // Port 0
    LedLocation {
        controller: LedController::Left,
        output: 0,
    },
    // Port 1
    LedLocation {
        controller: LedController::Left,
        output: 2,
    },
    // Port 2
    LedLocation {
        controller: LedController::Left,
        output: 4,
    },
    // Port 3
    LedLocation {
        controller: LedController::Left,
        output: 6,
    },
    // Port 4
    LedLocation {
        controller: LedController::Left,
        output: 8,
    },
    // Port 5
    LedLocation {
        controller: LedController::Left,
        output: 10,
    },
    // Port 6
    LedLocation {
        controller: LedController::Left,
        output: 15,
    },
    // Port 7
    LedLocation {
        controller: LedController::Left,
        output: 13,
    },
    // Port 8
    LedLocation {
        controller: LedController::Right,
        output: 0,
    },
    // Port 9
    LedLocation {
        controller: LedController::Right,
        output: 2,
    },
    // Port 10
    LedLocation {
        controller: LedController::Right,
        output: 4,
    },
    // Port 11
    LedLocation {
        controller: LedController::Right,
        output: 6,
    },
    // Port 12
    LedLocation {
        controller: LedController::Right,
        output: 8,
    },
    // Port 13
    LedLocation {
        controller: LedController::Right,
        output: 10,
    },
    // Port 14
    LedLocation {
        controller: LedController::Right,
        output: 15,
    },
    // Port 15
    LedLocation {
        controller: LedController::Right,
        output: 13,
    },
    // Port 16
    LedLocation {
        controller: LedController::Left,
        output: 3,
    },
    // Port 17
    LedLocation {
        controller: LedController::Left,
        output: 1,
    },
    // Port 18
    LedLocation {
        controller: LedController::Left,
        output: 7,
    },
    // Port 19
    LedLocation {
        controller: LedController::Left,
        output: 5,
    },
    // Port 20
    LedLocation {
        controller: LedController::Left,
        output: 9,
    },
    // Port 21
    LedLocation {
        controller: LedController::Left,
        output: 11,
    },
    // Port 22
    LedLocation {
        controller: LedController::Left,
        output: 14,
    },
    // Port 23
    LedLocation {
        controller: LedController::Left,
        output: 12,
    },
    // Port 24
    LedLocation {
        controller: LedController::Right,
        output: 3,
    },
    // Port 25
    LedLocation {
        controller: LedController::Right,
        output: 1,
    },
    // Port 26
    LedLocation {
        controller: LedController::Right,
        output: 7,
    },
    // Port 27
    LedLocation {
        controller: LedController::Right,
        output: 5,
    },
    // Port 28
    LedLocation {
        controller: LedController::Right,
        output: 9,
    },
    // Port 29
    LedLocation {
        controller: LedController::Right,
        output: 11,
    },
    // Port 30
    LedLocation {
        controller: LedController::Right,
        output: 14,
    },
    // Port 31
    LedLocation {
        controller: LedController::Right,
        output: 12,
    },
    // System
    LedLocation {
        controller: LedController::Left,
        output: 23,
    },
];

impl Leds {
    pub fn new(
        left_controller: &I2cDevice,
        right_controller: &I2cDevice,
    ) -> Self {
        Self {
            controllers: [
                Pca9956B::new(left_controller),
                Pca9956B::new(right_controller),
            ],
        }
    }

    pub fn initialize_current(&self) -> Result<(), Error> {
        self.set_current(DEFAULT_LED_CURRENT)?;

        Ok(())
    }

    pub fn set_current(&self, value: u8) -> Result<(), Error> {
        for (_i, controller) in self.controllers.iter().enumerate() {
            controller.set_iref_all(value)?;
        }

        Ok(())
    }

    pub fn turn_on_system_led(&self) -> Result<(), Error> {
        self.controllers[LED_MAP[SYSTEM_LED_IDX].controller as usize]
            .set_a_led_pwm(LED_MAP[SYSTEM_LED_IDX].output, DEFAULT_LED_PWM)?;

        Ok(())
    }

    pub fn update_led_state(&self, mask: u32) -> Result<(), Error> {
        let mut data_l: [u8; 16] = [0; 16];
        let mut data_r: [u8; 16] = [0; 16];

        for i in 0..32 {
            let bit_mask: u32 = 1 << i;
            let pwm_value = if (mask & bit_mask) != 0 {
                DEFAULT_LED_PWM
            } else {
                0
            };

            if LED_MAP[i].controller == LedController::Left {
                data_l[LED_MAP[i].output as usize] = pwm_value;
            } else {
                data_r[LED_MAP[i].output as usize] = pwm_value;
            }
        }

        self.controllers[LedController::Left as usize]
            .set_all_led_pwm(&data_l)?;
        self.controllers[LedController::Right as usize]
            .set_all_led_pwm(&data_r)?;

        Ok(())
    }

    pub fn check_errors(
        &self,
        controller: LedController,
    ) -> Result<Option<Pca9956BErrorState>, Error> {
        Ok(self.controllers[controller as usize].check_for_errors()?)
    }

    pub fn error_summary(&self) -> Result<Option<FullErrorSummary>, Error> {
        let errs = [
            self.check_errors(LedController::Left).unwrap_or(None),
            None
            // self.check_errors(LedController::Right).unwrap_or(None),
        ];

        let no_errors: bool = errs
            .iter()
            .fold(true, |no_error, next| no_error | next.is_none());
        if no_errors {
            return Ok(None);
        }

        let mut summary = FullErrorSummary {
            ..Default::default()
        };

        if errs[LedController::Left as usize].is_some() {
            summary.left =
                errs[LedController::Left as usize].unwrap().summary();
        }

        if errs[LedController::Right as usize].is_some() {
            summary.right =
                errs[LedController::Right as usize].unwrap().summary();
        }

        Ok(Some(summary))
    }
}
