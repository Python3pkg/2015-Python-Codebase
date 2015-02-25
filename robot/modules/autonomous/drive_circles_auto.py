import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_coroutine, call_public_method

class EndOfAutoException(Exception):
    pass

class DriveCirclesAuto(yeti.Module):
    """A basic autonomous routine that simply strafes in circles"""

    CIRCLE_RADIUS = 3


    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_external_input_datastream = datastreams.get_datastream("drivetrain_external_sensor_input")

    def check_mode(self):
        if not gamemode.is_autonomous():
            raise EndOfAutoException

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            self.drivetrain_external_input_datastream.push({"ultrasonic": {"enabled": True, "x_pos": 0}})

            call_public_method("drivetrain.reset_sensor_input")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0, "r_pos": 0})
            call_public_method("drivetrain.auto_drive_enable")

            self.logger.info("Point 1")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": self.CIRCLE_RADIUS, "r_pos": 0})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            self.logger.info("Point 2")
            self.drivetrain_setpoint_datastream.push({"x_pos": -self.CIRCLE_RADIUS, "y_pos": 0, "r_pos": 0})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            self.logger.info("Point 3")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": -self.CIRCLE_RADIUS, "r_pos": 0})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            self.logger.info("Point 4")
            self.drivetrain_setpoint_datastream.push({"x_pos": self.CIRCLE_RADIUS, "y_pos": 0, "r_pos": 0})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            self.logger.info("Point 5")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": self.CIRCLE_RADIUS, "r_pos": 0})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            self.logger.info("Point 6")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0, "r_pos": 360})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            call_public_method("auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")
        finally:
            call_public_method("drivetrain.auto_drive_disable")
            self.drivetrain_external_input_datastream.push({"ultrasonic": {"enabled": False, "x_pos": None}})
