import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.remote_methods import call_public_method


class EndOfAutoException(Exception):
    pass

class ThreeToteAuto(yeti.Module):

    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("auto_drive_setpoint")

    def check_mode(self):
        if not gamemode.is_autonomous():
            raise EndOfAutoException

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            #Grab tote
            yield from call_public_method("elevator_goto_home")
            self.check_mode()
            yield from call_public_method("elevator_goto_bottom")
            self.check_mode()
            yield from call_public_method("elevator_goto_home")
            self.check_mode()
            #Drive around tote
            yield from call_public_method("auto_drive_enable")
            self.logger.info("Drive phase 1")
            self.drivetrain_setpoint_datastream.push({"x_pos": 3})
            yield from call_public_method("auto_drive_wait_for_x")
            self.check_mode()
            self.logger.info("Drive phase 2")
            self.drivetrain_setpoint_datastream.push({"y_pos": 3})
            yield from call_public_method("auto_drive_wait_for_y")
            self.check_mode()
            #Drive into auto zone
            self.logger.info("Drive phase 3")
            self.drivetrain_setpoint_datastream.push({"x_pos": -8})
            yield from call_public_method("auto_drive_wait_for_x")
            yield from call_public_method("auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")
