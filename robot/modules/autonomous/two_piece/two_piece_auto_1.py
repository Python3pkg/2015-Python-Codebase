import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class TwoPieceAutoOne(yeti.Module):
    """
    This is one possible version of a two piece autonomous mode.


    """

    DO_PAUSES = True

    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")
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
            call_public_method("drivetrain.reset_sensor_input")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0})

            # Grab tote
            yield from call_public_coroutine("elevator.goto_bottom")
            self.check_mode()
            yield from call_public_coroutine("elevator.goto_home")
            self.check_mode()
            # Drive around tote

            call_public_method("drivetrain.auto_drive_enable")

            # Strafe around container
            self.logger.info("Drive phase 1")
            self.drivetrain_setpoint_datastream.push({"y_pos": 1})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()
            yield from self.do_pause()

            # Back up to origin
            self.logger.info("Drive phase 2")
            self.drivetrain_setpoint_datastream.push({"x_pos": -1.25})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()
            yield from self.do_pause()

            # Drive along wall to -2.5, 2.5
            self.logger.info("Drive phase 3")
            self.drivetrain_setpoint_datastream.push({"x_pos": -2.5, "y_pos": 2.5})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()
            yield from self.do_pause()

            # Drive forward to y=4.5
            self.logger.info("Drive phase 4")
            self.drivetrain_setpoint_datastream.push({"y_pos": 4.5})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()
            yield from self.do_pause()

            # Strafe to auto zone at x=8
            self.logger.info("Drive phase 5")
            self.drivetrain_setpoint_datastream.push({"x_pos": 8})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")
            self.check_mode()

            self.logger.info("Autonomous routine took {} seconds total".format(wpilib.Timer.getFPGATimestamp() - start_time))
            self.logger.info("Auto mode done.")
            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")



