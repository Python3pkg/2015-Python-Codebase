import asyncio
import yeti
import wpilib
import json

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine

class AdvancecdDriveAuto(yeti.Module):
    """A basic autonomous routine that simply moves forward for a given speed and time"""

    def module_init(self):
        self.drivetrain_control = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_config = datastreams.get_datastream("drivetrain_auto_config")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")
        config_options = [("x_setpoint", 0), ("y_setpoint", 5), ("r_setpoint", 0), ("start_delay", 0)]
        for key, value in config_options:
            if wpilib.SmartDashboard.getNumber("autonomous/" + key, value) == value:
                wpilib.SmartDashboard.putNumber("autonomous/" + key, value)
        wpilib.SmartDashboard.putString("autonomous/config_keys", json.dumps([k for k, v in config_options]))


    @asyncio.coroutine
    @gamemode.autonomous_task
    def do_auto(self):
        self.logger.info("Starting autonomous mode given by module " + self.name)

        x_setpoint = wpilib.SmartDashboard.getNumber("autonomous/x_setpoint")
        y_setpoint = wpilib.SmartDashboard.getNumber("autonomous/y_setpoint")
        r_setpoint = wpilib.SmartDashboard.getNumber("autonomous/r_setpoint")
        start_delay = wpilib.SmartDashboard.getNumber("autonomous/start_delay")

        self.logger.info("Config options: x_setpoint: {}, y_setpoint: {}, r_setpoint: {}, start_delay: {}".format(x_setpoint, y_setpoint, r_setpoint, start_delay))

        start_time = wpilib.Timer.getFPGATimestamp()

        self.drivetrain_control.push({"x_pos": 0, "y_pos": 0, "r_pos": 0})
        call_public_method("drivetrain.reset_sensor_input")
        call_public_method("drivetrain.auto_drive_enable")

        yield from asyncio.sleep(start_delay)

        self.drivetrain_control.push({"x_pos": x_setpoint, "y_pos": y_setpoint, "r_pos": r_setpoint})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        call_public_method("drivetrain.auto_drive_disable")
        self.logger.info("Ending autonomous mode -- waiting for mode to change")
        self.logger.info("Autonomous routine took {} seconds total".format(wpilib.Timer.getFPGATimestamp() - start_time))
        while gamemode.is_autonomous():
            yield from asyncio.sleep(.5)
        self.logger.info("Ended autonomous mode")
