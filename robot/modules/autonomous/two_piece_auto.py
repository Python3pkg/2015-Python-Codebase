import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class TwoPieceAuto(yeti.Module):
    """
    This is a two-piece autonomous mode. It lifts the tote, strafe-turns to capture the RC,
    and drives to the auto zone. It averages about 4.4 seconds for the sequence and should work in all staging zones.
    It probably won't work for unloading the stack at the end.
    """

    DO_PAUSES = False

    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")
        self.elevator_setpoint_datastream = datastreams.get_datastream("elevator_setpoint")
        self.elevator_input_datastream = datastreams.get_datastream("elevator_input")
        wpilib.SmartDashboard.putBoolean("do_pauses", False)

    def check_mode(self):
        if not gamemode.is_autonomous():
            raise EndOfAutoException

    @asyncio.coroutine
    def do_pause(self):
        if self.DO_PAUSES:
            drivetrain_input_data = self.drivetrain_sensor_input.get()
            print("Position Update: ({},{},{})".format(drivetrain_input_data.get("x_pos", 0), drivetrain_input_data.get("y_pos", 0), drivetrain_input_data.get("r_pos", 0)))
            yield from asyncio.sleep(1)

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            self.DO_PAUSES = wpilib.SmartDashboard.getBoolean("do_pauses")
            start_time = wpilib.Timer.getFPGATimestamp()
            call_public_method("drivetrain.auto_drive_enable")

            # Zero drive setpoints
            call_public_method("drivetrain.reset_sensor_input")
            call_public_method("drivetrain.reset_auto_config")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0, "r_pos": 0})


            # Grab tote
            yield from call_public_coroutine("elevator.goto_bottom")
            self.check_mode()
            yield from call_public_coroutine("elevator.goto_pos", 5)
            self.check_mode()
            yield from self.do_pause()

            # Strafe-turn to capture container
            self.logger.info("Drive phase 1")
            self.drivetrain_setpoint_datastream.push({"x_pos": -1.5, "y_pos": 2.5, "r_pos": -45})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()
            yield from self.do_pause()

            # Drive to auto zone at x=8.5
            self.logger.info("Drive phase 2")
            self.drivetrain_setpoint_datastream.push({"x_pos": 10.5, "r_pos": -90})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            self.logger.info("Autonomous routine took {} seconds total".format(wpilib.Timer.getFPGATimestamp() - start_time))
            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")



