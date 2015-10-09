import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class TwoToteAuto(yeti.Module):

    PAUSE = 0
    auto_start_timestamp = 0

    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")
        self.drivetrain_config_datastream = datastreams.get_datastream("drivetrain_auto_config")
        self.elevator_setpoint_datastream = datastreams.get_datastream("elevator_setpoint")
        self.elevator_input_datastream = datastreams.get_datastream("elevator_input")
        wpilib.SmartDashboard.putNumber("pause_duration", 0)

    def check_mode(self):
        if not gamemode.is_autonomous():
            raise EndOfAutoException

    @asyncio.coroutine
    def do_pause(self):
        self.check_mode()
        if self.PAUSE > 0:
            drivetrain_input_data = self.drivetrain_sensor_input.get()
            print("Position Update: ({},{},{})".format(drivetrain_input_data.get("x_pos", 0), drivetrain_input_data.get("y_pos", 0), drivetrain_input_data.get("r_pos", 0)))
            yield from asyncio.sleep(self.PAUSE)

    def reset_auto_time(self):
        self.auto_start_timestamp = wpilib.Timer.getFPGATimestamp()

    def get_auto_time(self):
        return wpilib.Timer.getFPGATimestamp() - self.auto_start_timestamp

    def report(self, msg):
        self.logger.info("{} at {} seconds".format(msg, self.get_auto_time()))

    @asyncio.coroutine
    def get_tote(self, y_pos):
        """
        If we are farther than 6 inches away from the tote, raise forks to home position and drive to 6 inches away.
        Drive to the tote. Once there, lower forks to the bottom. Then set them to raise to 2, waiting for
        them to be at least .5 before exiting.
        """
        self.report("Getting tote at y={}".format(y_pos))

        # If we have room, raise forks and close in on tote.
        if self.drivetrain_sensor_input.get()["y_pos"] < y_pos - 1.7:
            self.logger.info("waiting!")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": y_pos - 1.7, "r_pos": 0})
            yield from call_public_coroutine("elevator.goto_pos", 2.5)
            yield from call_public_coroutine("drivetrain.wait_for_x")

        # Drive to tote (slightly ahead to nestle the tote in place).
        self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": y_pos+.25, "r_pos": 0})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Increase y tolerance to stop any movement.
        self.drivetrain_config_datastream.push({"y_tolerance": 1, "x_tolerance": 1})

        # Grab tote.
        yield from call_public_coroutine("elevator.goto_pos", 0)

        # Lift tote slightly
        yield from call_public_coroutine("elevator.goto_pos", 2)

        # Set y setpoint
        self.drivetrain_setpoint_datastream.push({"y_pos": y_pos-.25})

        # Decrease translation tolerance back to normal.
        call_public_method("drivetrain.reset_auto_config")


    @asyncio.coroutine
    def move_container(self, y_pos):
        """
        Drive to clear the container in the x direction and just before the container in the y direction.
        Once we clear on the x axis, drive to the y coordinate of the container. Then drive back to x=0
        and return.
        """
        self.report("Moving container at y={}".format(y_pos))

        # Set the x and y setpoint to off the corner of the container (If it had a corner!)
        self.drivetrain_setpoint_datastream.push({"x_pos": 2.7, "y_pos": y_pos - 3, "r_pos": 0})
        self.check_mode()

        # Wait until we clear the container
        while self.drivetrain_sensor_input.get().get("x_pos") < 2:
            yield from asyncio.sleep(.1)
            self.check_mode()

        # Set y_pos a little ahead of the container
        self.drivetrain_setpoint_datastream.push({"y_pos": y_pos + 1})

        # Wait for y to be close enough
        while self.drivetrain_sensor_input.get().get("y_pos") < y_pos:
            yield from asyncio.sleep(.1)
            self.check_mode()


    @asyncio.coroutine
    def platform_align(self):
        """
        Spin 90 degrees and approach the scoring platform. This is so that we don't drive sideways over the scoring platforms.
        """

        self.report("Aligning to scoring platform")

        # Drive to x=4, spinning 90 degrees clockwise
        self.drivetrain_setpoint_datastream.push({"x_pos": 3, "r_pos": -90})
        yield from call_public_coroutine("drivetrain.wait_for_r")
        self.check_mode()

    @asyncio.coroutine
    def score_bot(self):
        """ Scores the robot, moving to the auto zone at x=10.5 """
        self.report("Scoring robot")

        self.drivetrain_setpoint_datastream.push({"x_pos": 10.5})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")
        self.check_mode()

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            call_public_method("drivetrain.auto_drive_enable")
            call_public_method("drivetrain.reset_sensor_input")
            call_public_method("drivetrain.reset_auto_config")
            self.reset_auto_time()

            yield from self.get_tote(0)
            yield from self.do_pause()
            yield from self.move_container(2.7)
            yield from self.do_pause()
            yield from self.get_tote(6.5)
            yield from self.do_pause()
            yield from self.platform_align()
            yield from self.do_pause()
            yield from self.score_bot()

            self.logger.info("Autonomous routine took {} seconds total".format(self.get_auto_time()))

            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")



