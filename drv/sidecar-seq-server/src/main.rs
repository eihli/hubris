// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

//! Server for managing the Sidecar sequencing process.

#![no_std]
#![no_main]

use crate::clock_generator::ClockGenerator;
use crate::front_io::FrontIOBoard;
use crate::tofino::Tofino;
use drv_fpga_api::{DeviceState, FpgaError, WriteOp};
use drv_i2c_api::{I2cDevice, ResponseCode};
use drv_sidecar_mainboard_controller::tofino2::{
    DebugPortState, DirectBarSegment, Tofino2Vid, TofinoPcieReset,
    TofinoSeqError, TofinoSeqState,
};
use drv_sidecar_mainboard_controller::MainboardController;
use drv_sidecar_seq_api::{SeqError, TofinoSequencerPolicy};
use idol_runtime::{
    ClientError, Leased, NotificationHandler, RequestError, R, W,
};
use ringbuf::*;
use userlib::*;

task_slot!(I2C, i2c_driver);
task_slot!(MAINBOARD, mainboard);
task_slot!(FRONT_IO, front_io);
task_slot!(AUXFLASH, auxflash);

include!(concat!(env!("OUT_DIR"), "/i2c_config.rs"));

mod clock_generator;
mod front_io;
mod tofino;

#[allow(dead_code)]
#[derive(Copy, Clone, PartialEq)]
enum Trace {
    None,
    FpgaInit,
    FpgaInitComplete,
    FpgaBitstreamError(u32),
    LoadingFpgaBitstream,
    SkipLoadingBitstream,
    MainboardControllerId(u32),
    MainboardControllerChecksum(u32),
    MainboardControllerVersion(u32),
    MainboardControllerSha(u32),
    InvalidMainboardControllerId(u32),
    ExpectedMainboardControllerChecksum(u32),
    LoadingClockConfiguration,
    SkipLoadingClockConfiguration,
    ClockConfigurationError(usize, ResponseCode),
    ClockConfigurationComplete,
    TofinoSequencerPolicyUpdate(TofinoSequencerPolicy),
    TofinoSequencerTick(TofinoSequencerPolicy, TofinoSeqState, TofinoSeqError),
    TofinoSequencerError(SeqError),
    TofinoSequencerFault(TofinoSeqError),
    TofinoVidAck,
    TofinoEepromIdCode(u32),
    InitiateTofinoPowerUp,
    InitiateTofinoPowerDown,
    SetVddCoreVout(userlib::units::Volts),
    SetPCIePresent,
    ClearPCIePresent,
    ClearingTofinoSequencerFault(TofinoSeqError),
    FrontIOBoardPresent,
    NoFrontIOBoardPresent,
    LoadingFrontIOControllerBitstream {
        fpga_id: usize,
    },
    SkipLoadingFrontIOControllerBitstream {
        fpga_id: usize,
    },
    FrontIOControllerIdent {
        fpga_id: usize,
        ident: u32,
    },
    FrontIOControllerChecksum {
        fpga_id: usize,
        checksum: [u8; 4],
        expected: [u8; 4],
    },
    FrontIOVsc8562Ready,
    FrontIOPca9956BEnabled,
}
ringbuf!(Trace, 32, Trace::None);

const TIMER_NOTIFICATION_MASK: u32 = 1 << 0;
const TIMER_INTERVAL: u64 = 1000;

struct ServerImpl {
    mainboard_controller: MainboardController,
    clock_generator: ClockGenerator,
    tofino: Tofino,
    front_io_board: FrontIOBoard,
}

impl idl::InOrderSequencerImpl for ServerImpl {
    fn tofino_seq_policy(
        &mut self,
        _msg: &userlib::RecvMessage,
    ) -> Result<TofinoSequencerPolicy, RequestError<SeqError>> {
        Ok(self.tofino.policy)
    }

    fn set_tofino_seq_policy(
        &mut self,
        _msg: &userlib::RecvMessage,
        policy: TofinoSequencerPolicy,
    ) -> Result<(), RequestError<SeqError>> {
        ringbuf_entry!(Trace::TofinoSequencerPolicyUpdate(policy));
        self.tofino.policy = policy;
        Ok(())
    }

    fn tofino_seq_state(
        &mut self,
        _: &RecvMessage,
    ) -> Result<TofinoSeqState, RequestError<SeqError>> {
        Ok(self.tofino.sequencer.state().map_err(SeqError::from)?)
    }

    fn tofino_seq_error(
        &mut self,
        _: &RecvMessage,
    ) -> Result<TofinoSeqError, RequestError<SeqError>> {
        Ok(self.tofino.sequencer.error().map_err(SeqError::from)?)
    }

    fn clear_tofino_seq_error(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<SeqError>> {
        if let Ok(e) = self.tofino.sequencer.error().map_err(SeqError::from) {
            ringbuf_entry!(Trace::ClearingTofinoSequencerFault(e));
        }
        Ok(self
            .tofino
            .sequencer
            .clear_error()
            .map_err(SeqError::from)?)
    }

    fn tofino_power_status(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<SeqError>> {
        Ok(self
            .tofino
            .sequencer
            .power_status()
            .map_err(SeqError::from)?)
    }

    fn tofino_pcie_hotplug_ctrl(
        &mut self,
        _: &userlib::RecvMessage,
    ) -> Result<u8, RequestError<SeqError>> {
        Ok(self
            .tofino
            .sequencer
            .pcie_hotplug_ctrl()
            .map_err(SeqError::from)?)
    }

    fn set_tofino_pcie_hotplug_ctrl(
        &mut self,
        _: &userlib::RecvMessage,
        mask: u8,
    ) -> Result<(), RequestError<SeqError>> {
        Ok(self
            .tofino
            .sequencer
            .write_pcie_hotplug_ctrl(WriteOp::BitSet, mask)
            .map_err(SeqError::from)?)
    }

    fn clear_tofino_pcie_hotplug_ctrl(
        &mut self,
        _: &userlib::RecvMessage,
        mask: u8,
    ) -> Result<(), RequestError<SeqError>> {
        Ok(self
            .tofino
            .sequencer
            .write_pcie_hotplug_ctrl(WriteOp::BitClear, mask)
            .map_err(SeqError::from)?)
    }

    fn tofino_pcie_reset(
        &mut self,
        _: &userlib::RecvMessage,
    ) -> Result<TofinoPcieReset, RequestError<SeqError>> {
        Ok(self.tofino.sequencer.pcie_reset().map_err(SeqError::from)?)
    }

    fn set_tofino_pcie_reset(
        &mut self,
        _: &userlib::RecvMessage,
        reset: TofinoPcieReset,
    ) -> Result<(), RequestError<SeqError>> {
        Ok(self
            .tofino
            .sequencer
            .set_pcie_reset(reset)
            .map_err(SeqError::from)?)
    }

    fn tofino_pcie_hotplug_status(
        &mut self,
        _: &userlib::RecvMessage,
    ) -> Result<u8, RequestError<SeqError>> {
        Ok(self
            .tofino
            .sequencer
            .pcie_hotplug_status()
            .map_err(SeqError::from)?)
    }

    fn load_clock_config(
        &mut self,
        _: &RecvMessage,
    ) -> Result<(), RequestError<SeqError>> {
        Ok(self.clock_generator.load_config()?)
    }

    fn is_clock_config_loaded(
        &mut self,
        _: &RecvMessage,
    ) -> Result<bool, RequestError<SeqError>> {
        Ok(self.clock_generator.config_loaded)
    }

    fn front_io_board_present(
        &mut self,
        _: &RecvMessage,
    ) -> Result<bool, RequestError<SeqError>> {
        Ok(self.front_io_board.present())
    }

    fn front_io_phy_ready(
        &mut self,
        _: &RecvMessage,
    ) -> Result<bool, RequestError<SeqError>> {
        if !self.front_io_board.present() {
            Err(SeqError::NoFrontIOBoard.into())
        } else {
            let phy_smi = self.front_io_board.phy_smi();
            Ok(phy_smi.phy_powered_up_and_ready().map_err(SeqError::from)?)
        }
    }

    fn tofino_debug_port_state(
        &mut self,
        _: &RecvMessage,
    ) -> Result<DebugPortState, RequestError<SeqError>> {
        Ok(self.tofino.debug_port.state().map_err(SeqError::from)?)
    }

    fn set_tofino_debug_port_state(
        &mut self,
        _: &RecvMessage,
        state: DebugPortState,
    ) -> Result<(), RequestError<SeqError>> {
        Ok(self
            .tofino
            .debug_port
            .set_state(state)
            .map_err(SeqError::from)?)
    }

    fn tofino_read_direct(
        &mut self,
        _: &RecvMessage,
        segment: DirectBarSegment,
        offset: u32,
    ) -> Result<u32, RequestError<SeqError>> {
        Ok(self
            .tofino
            .debug_port
            .read_direct(segment, offset)
            .map_err(SeqError::from)?)
    }

    fn tofino_write_direct(
        &mut self,
        _: &RecvMessage,
        segment: DirectBarSegment,
        offset: u32,
        value: u32,
    ) -> Result<(), RequestError<SeqError>> {
        Ok(self
            .tofino
            .debug_port
            .write_direct(segment, offset, value)
            .map_err(SeqError::from)?)
    }

    fn spi_eeprom_idcode(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u32, RequestError<SeqError>> {
        Ok(self
            .tofino
            .debug_port
            .spi_eeprom_idcode()
            .map_err(SeqError::from)?)
    }

    fn spi_eeprom_status(
        &mut self,
        _: &RecvMessage,
    ) -> Result<u8, RequestError<SeqError>> {
        Ok(self
            .tofino
            .debug_port
            .spi_eeprom_status()
            .map_err(SeqError::from)?
            .into())
    }

    fn read_spi_eeprom_bytes(
        &mut self,
        _: &RecvMessage,
        offset: u32,
        data: Leased<W, [u8]>,
    ) -> Result<(), RequestError<SeqError>> {
        let mut buf = [0u8; 128];
        let mut eeprom_offset = offset as usize;
        let mut data_offset = 0;
        let eeprom_end = offset as usize + data.len();

        while eeprom_offset < eeprom_end {
            let amount = (eeprom_end - eeprom_offset).min(buf.len());
            self.tofino
                .debug_port
                .read_spi_eeprom_bytes(eeprom_offset, &mut buf[..amount])
                .map_err(SeqError::from)?;
            data.write_range(
                data_offset..(data_offset + amount),
                &buf[..amount],
            )
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;
            data_offset += amount;
            eeprom_offset += amount;
        }

        Ok(())
    }

    fn write_spi_eeprom_bytes(
        &mut self,
        _: &RecvMessage,
        offset: u32,
        data: Leased<R, [u8]>,
    ) -> Result<(), RequestError<SeqError>> {
        let mut buf = [0u8; 128];
        let mut eeprom_offset = offset as usize;
        let mut data_offset = 0;
        let eeprom_end = offset as usize + data.len();

        while eeprom_offset < eeprom_end {
            let amount = (eeprom_end - eeprom_offset).min(buf.len());
            data.read_range(
                data_offset..(data_offset + amount),
                &mut buf[..amount],
            )
            .map_err(|_| RequestError::Fail(ClientError::WentAway))?;
            self.tofino
                .debug_port
                .write_spi_eeprom_bytes(eeprom_offset, &buf[..amount])
                .map_err(SeqError::from)?;
            data_offset += amount;
            eeprom_offset += amount;
        }

        Ok(())
    }
}

impl NotificationHandler for ServerImpl {
    fn current_notification_mask(&self) -> u32 {
        TIMER_NOTIFICATION_MASK
    }

    fn handle_notification(&mut self, _bits: u32) {
        let start = sys_get_timer().now;

        if let Err(e) = self.tofino.handle_tick() {
            ringbuf_entry!(Trace::TofinoSequencerError(e));
        }

        let finish = sys_get_timer().now;

        // We now know when we were notified and when any work was completed.
        // Note that the assumption here is that `start` < `finish` and that
        // this won't hold if the system time rolls over. But, the system timer
        // is a u64, with each bit representing a ms, so in practice this should
        // be fine. Anyway, armed with this information, find the next deadline
        // some multiple of `TIMER_INTERVAL` in the future.

        let delta = finish - start;
        let next_deadline = finish + TIMER_INTERVAL - (delta % TIMER_INTERVAL);

        sys_set_timer(Some(next_deadline), TIMER_NOTIFICATION_MASK);
    }
}

#[export_name = "main"]
fn main() -> ! {
    let mut buffer = [0; idl::INCOMING_SIZE];

    let mainboard_controller =
        MainboardController::new(MAINBOARD.get_task_id());
    let clock_generator = ClockGenerator::new(I2C.get_task_id());
    let tofino = Tofino::new(I2C.get_task_id());
    let front_io_board = FrontIOBoard::new(
        FRONT_IO.get_task_id(),
        I2C.get_task_id(),
        AUXFLASH.get_task_id(),
    );

    let mut server = ServerImpl {
        mainboard_controller,
        clock_generator,
        tofino,
        front_io_board,
    };

    ringbuf_entry!(Trace::FpgaInit);

    match server
        .mainboard_controller
        .await_fpga_ready(25)
        .unwrap_or(DeviceState::Unknown)
    {
        DeviceState::AwaitingBitstream => {
            ringbuf_entry!(Trace::LoadingFpgaBitstream);

            match server
                .mainboard_controller
                .load_bitstream(AUXFLASH.get_task_id())
            {
                Err(e) => {
                    let code = u32::try_from(e).unwrap();
                    ringbuf_entry!(Trace::FpgaBitstreamError(code));

                    // If this is an auxflash error indicating that we can't
                    // find the target blob, then it's possible that data isn't
                    // present (i.e. this is an initial boot at the factory). To
                    // prevent this task from spinning too hard, we add a brief
                    // delay before resetting.
                    //
                    // Note that other auxflash errors (e.g. a failed read) will
                    // reset immediately, matching existing behavior on a failed
                    // FPGA reset.
                    if matches!(e, FpgaError::AuxMissingBlob) {
                        userlib::hl::sleep_for(100);
                    }
                    panic!();
                }
                // Write the checksum fuses to lock the design until the FPGA is
                // reset.
                Ok(()) => {
                    server.mainboard_controller.write_checksum().unwrap_lite();
                }
            }
        }
        DeviceState::RunningUserDesign => {
            ringbuf_entry!(Trace::SkipLoadingBitstream);
        }
        _ => panic!(),
    }

    // Read the design Ident and determine if a bitstream reload is needed.
    let ident = server.mainboard_controller.read_ident().unwrap_lite();

    match ident.id.into() {
        MainboardController::EXPECTED_ID => {
            ringbuf_entry!(Trace::MainboardControllerId(ident.id.into()))
        }
        _ => {
            // The FPGA is running something unexpected. Reset the device and
            // fire the escape thrusters. This will force a bitstream load when
            // the task is restarted.
            ringbuf_entry!(Trace::InvalidMainboardControllerId(
                ident.id.into()
            ));
            server.mainboard_controller.reset().unwrap_lite();
            panic!()
        }
    }

    ringbuf_entry!(Trace::MainboardControllerChecksum(ident.checksum.into()));

    if !server.mainboard_controller.checksum_valid(&ident) {
        ringbuf_entry!(Trace::ExpectedMainboardControllerChecksum(
            MainboardController::short_checksum()
        ));

        // The mainboard controller does not match the checksum of the
        // bitstream which is expected to run. This means the register map
        // may not match the APIs in this binary so a bitstream reload is
        // required.
        //
        // Attempt to shutdown Tofino somewhat gracefully instead of letting
        // the PDN collapse. This may need some tweaking because the host
        // CPU will suddenly lose its PCIe device. Perhaps an interrupt is
        // in order here to prep the driver for what's about to happen.
        if let Ok(()) = server.tofino.power_down() {
            // Give the sequencer some time to shut down the PDN.
            userlib::hl::sleep_for(25);
        }

        // Reset the FPGA and deploy the parashutes. This will cause the
        // bitstream to be reloaded when the task is restarted.
        server.mainboard_controller.reset().unwrap_lite();
        panic!()
    }

    // The expected version of the mainboard controller is running. Log some
    // more details.
    ringbuf_entry!(Trace::MainboardControllerVersion(ident.version.into()));
    ringbuf_entry!(Trace::MainboardControllerSha(ident.sha.into()));
    ringbuf_entry!(Trace::FpgaInitComplete);

    // The sequencer for the clock generator currently does not have a feedback
    // mechanism/register we can read. Sleeping a short while seems to be
    // sufficient for now.
    //
    // TODO (arjen): Implement reset control through the mainboard controller.
    userlib::hl::sleep_for(100);

    if let TofinoSeqState::A0 = server
        .tofino
        .sequencer
        .state()
        .unwrap_or(TofinoSeqState::Initial)
    {
        ringbuf_entry!(Trace::SkipLoadingClockConfiguration);
        server.clock_generator.config_loaded = true;
        server.tofino.policy = TofinoSequencerPolicy::LatchOffOnFault;
    } else if server.clock_generator.load_config().is_err() {
        panic!()
    }
    ringbuf_entry!(Trace::ClockConfigurationComplete);

    // Initialize a connected Front IO board.
    if server.front_io_board.present() {
        ringbuf_entry!(Trace::FrontIOBoardPresent);

        if !server.front_io_board.init().unwrap() {
            panic!();
        }

        let phy_smi = server.front_io_board.phy_smi();
        phy_smi.set_phy_power_enabled(true).unwrap();

        while !phy_smi.phy_powered_up_and_ready().unwrap() {
            userlib::hl::sleep_for(10);
        }

        ringbuf_entry!(Trace::FrontIOVsc8562Ready);

        server.front_io_board.enable_led_controllers().unwrap();

        ringbuf_entry!(Trace::FrontIOPca9956BEnabled);
    } else {
        ringbuf_entry!(Trace::NoFrontIOBoardPresent);
    }

    //
    // This will put our timer in the past, and should immediately kick us.
    //
    let deadline = sys_get_timer().now;
    sys_set_timer(Some(deadline), TIMER_NOTIFICATION_MASK);

    loop {
        idol_runtime::dispatch_n(&mut buffer, &mut server);
    }
}

mod idl {
    use super::{
        DebugPortState, DirectBarSegment, SeqError, TofinoPcieReset,
        TofinoSeqError, TofinoSeqState, TofinoSequencerPolicy,
    };

    include!(concat!(env!("OUT_DIR"), "/server_stub.rs"));
}
