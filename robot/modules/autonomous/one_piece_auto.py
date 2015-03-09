import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class OnePieceAuto(yeti.Module):
    """
    This is a one-piece autonomous mode. It lifts a game piece and moves into the auto zone, rotating on the way.
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
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0, "r_pos": 0})

            # Grab item
            yield from call_public_coroutine("elevator.goto_bottom")
            self.check_mode()
            yield from call_public_coroutine("elevator.goto_pos", 5)
            self.check_mode()
            yield from self.do_pause()

            # Drive to x=4, spinning 90 degrees clockwise
            self.drivetrain_setpoint_datastream.push({"x_pos": 4, "r_pos": -90})
            yield from call_public_coroutine("drivetrain.wait_for_r")
            self.check_mode()

            # Drive to auto zone at x=10.5
            self.drivetrain_setpoint_datastream.push({"x_pos": 10.5})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            self.logger.info("Autonomous routine took {} seconds total".format(wpilib.Timer.getFPGATimestamp() - start_time))
            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")




