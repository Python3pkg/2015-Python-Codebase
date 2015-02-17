import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.remote_methods import call_public_coroutine, call_public_method

class EndOfAutoException(Exception):
    pass

class DriveCirclesAuto(yeti.Module):
    """A basic autonomous routine that simply strafes in circles"""

    CIRCLE_RADIUS = 3


    def module_init(self):
        self.drivetrain_control_datastream = datastreams.get_datastream("drivetrain_control")
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("auto_drive_setpoint")

    def check_mode(self):
        if not gamemode.is_autonomous():
            raise EndOfAutoException

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            call_public_method("auto_drive_reset_tracking")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0})
            call_public_method("auto_drive_enable")

            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": self.CIRCLE_RADIUS})
            yield from call_public_coroutine("auto_drive_wait_for_xy")
            self.check_mode()

            self.drivetrain_setpoint_datastream.push({"x_pos": -self.CIRCLE_RADIUS, "y_pos": 0})
            yield from call_public_coroutine("auto_drive_wait_for_xy")
            self.check_mode()

            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": -self.CIRCLE_RADIUS})
            yield from call_public_coroutine("auto_drive_wait_for_xy")
            self.check_mode()

            self.drivetrain_setpoint_datastream.push({"x_pos": self.CIRCLE_RADIUS, "y_pos": 0})
            yield from call_public_coroutine("auto_drive_wait_for_xy")
            self.check_mode()

            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": self.CIRCLE_RADIUS})
            yield from call_public_coroutine("auto_drive_wait_for_xy")
            self.check_mode()

            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0})
            yield from call_public_coroutine("auto_drive_wait_for_xy")
            self.check_mode()

            call_public_method("auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")