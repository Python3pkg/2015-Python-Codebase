import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class ThreeToteAuto(yeti.Module):

    DO_PAUSES = True
    auto_start_timestamp = 0

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
            yield from asyncio.sleep(2)

    def reset_auto_time(self):
        self.auto_start_timestamp = wpilib.Timer.getFPGATimestamp()

    def get_auto_time(self):
        return wpilib.Timer.getFPGATimestamp() - self.auto_start_timestamp

    @asyncio.coroutine
    def two_piece_run(self):
        """
        This is one possible version of a two piece autonomous mode. It strafes left, drives forward, and strafes right.
        It averages about 3.1 seconds for the sequence but won't work in staging zone 1.
        """
        self.logger.info("Begin two_piece_run at {}".format(self.get_auto_time()))
        call_public_method("drivetrain.reset_sensor_input")
        self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0})

        # Grab tote
        yield from call_public_coroutine("elevator.goto_pos", .5)
        self.check_mode()
        call_public_method("elevator.set_setpoint", 1)
        self.check_mode()

        # Strafe to side
        self.logger.info("Drive phase 1")
        self.drivetrain_setpoint_datastream.push({"x_pos": 2.5})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")
        yield from self.do_pause()

        # Drive forward
        self.logger.info("Drive phase 2")
        self.drivetrain_setpoint_datastream.push({"y_pos": 2.7})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")
        yield from self.do_pause()

        # Strafe back to center
        self.logger.info("Drive phase 3")
        self.drivetrain_setpoint_datastream.push({"x_pos": 0})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")
        self.check_mode()
        self.logger.info("End two_piece_auto at {}".format(self.get_auto_time()))

    @asyncio.coroutine
    def final_two_piece_run(self):

        self.logger.info("Begin final_two_piece_run at {}".format(self.get_auto_time()))
        call_public_method("drivetrain.reset_sensor_input")
        self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0})

        # Grab tote
        yield from call_public_coroutine("elevator.goto_pos", .5)
        self.check_mode()
        call_public_method("elevator.set_setpoint", 1)
        self.check_mode()

        # Strafe to auto zone at x=8
        self.logger.info("Drive phase 3")
        self.drivetrain_setpoint_datastream.push({"x_pos": 8})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        yield from call_public_coroutine("elevator.goto_bottom")
        self.drivetrain_setpoint_datastream.push({"y_pos": -4})
        self.check_mode()
        self.logger.info("End two_piece_auto at {}".format(self.get_auto_time()))

    @asyncio.coroutine
    def get_next_tote(self):
        self.logger.info("Begin get_next_tote at {}".format(self.get_auto_time()))
        self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 6, "r_pos": 0})
        yield from call_public_coroutine("elevator.goto_home")
        yield from call_public_coroutine("drivetrain.wait_for_x")
        self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 6.5, "r_pos": 0})
        call_public_method("elevator.set_setpoint", 1)
        yield from call_public_coroutine("drivetrain.wait_for_xyr")
        self.logger.info("End get_next_tote at {}".format(self.get_auto_time()))

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            self.DO_PAUSES = wpilib.SmartDashboard.getBoolean("do_pauses")
            call_public_method("drivetrain.auto_drive_enable")
            self.reset_auto_time()

            yield from self.two_piece_run()
            yield from self.get_next_tote()
            yield from self.two_piece_run()
            yield from self.get_next_tote()
            yield from self.final_two_piece_run()

            self.logger.info("Autonomous routine took {} seconds total".format(self.get_auto_time()))

            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")



