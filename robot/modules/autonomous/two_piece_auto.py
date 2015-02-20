import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class TwoPieceAuto(yeti.Module):

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

            #Grab tote
            yield from call_public_coroutine("elevator_goto_home")
            self.check_mode()
            yield from call_public_coroutine("elevator_goto_bottom")
            self.check_mode()
            yield from call_public_coroutine("elevator_goto_home")
            self.check_mode()
            #Drive around tote

            call_public_method("auto_drive_enable")

            #Strafe left 3 ft
            self.logger.info("Drive phase 1")
            self.drivetrain_setpoint_datastream.push({"x_pos": -3, "y_pos": 0})
            yield from call_public_coroutine("auto_drive_wait_for_x")
            self.check_mode()

            #Drive forward 3ft
            self.logger.info("Drive phase 2")
            self.drivetrain_setpoint_datastream.push({"y_pos": 3})
            yield from call_public_coroutine("auto_drive_wait_for_y")
            self.check_mode()

            #Strafe right 5 ft
            self.logger.info("Drive phase 3")
            self.drivetrain_setpoint_datastream.push({"x_pos": 5})
            yield from call_public_coroutine("auto_drive_wait_for_x")
            self.check_mode()

            call_public_method("auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")



