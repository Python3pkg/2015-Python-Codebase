import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class TwoPieceAutoFour(yeti.Module):
    """
    This is one possible version of a two piece autonomous mode. It strafes left, drives forward, and strafes right.
    It averages about 3.1 seconds for the sequence but won't work in staging zone 1.
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
    def two_piece_run(self):
        """
        This is one possible version of a two piece autonomous mode. It strafes left, drives forward, and strafes right.
        It averages about 3.1 seconds for the sequence but won't work in staging zone 1.
        """
        call_public_method("drivetrain.reset_sensor_input")
        self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0})

        # Grab tote
        yield from call_public_coroutine("elevator.goto_bottom")
        self.check_mode()
        call_public_method("elevator.set_setpoint", 1)
        self.check_mode()

        # Strafe to side
        self.logger.info("Drive phase 1")
        self.drivetrain_setpoint_datastream.push({"x_pos": -2.5})
        while self.drivetrain_sensor_input.get().get("x_pos") > -2:
            yield from asyncio.sleep(.1)
            self.check_mode()
        yield from self.do_pause()

        # Drive forward
        self.logger.info("Drive phase 2")
        self.drivetrain_setpoint_datastream.push({"y_pos": 2.7})
        while self.drivetrain_sensor_input.get().get("y_pos") < 2.4:
            yield from asyncio.sleep(.1)
            self.check_mode()
        yield from self.do_pause()

        # Strafe to auto zone at x=8
        self.logger.info("Drive phase 3")
        self.drivetrain_setpoint_datastream.push({"x_pos": 8})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")
        self.check_mode()


    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            self.DO_PAUSES = wpilib.SmartDashboard.getBoolean("do_pauses")
            call_public_method("drivetrain.auto_drive_enable")
            start_time = wpilib.Timer.getFPGATimestamp()

            yield from self.two_piece_run()

            self.logger.info("Autonomous routine took {} seconds total".format(wpilib.Timer.getFPGATimestamp() - start_time))

            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")



