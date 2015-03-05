import asyncio
import yeti
import wpilib

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class ThreeToteAuto(yeti.Module):

    PAUSE = 1
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
        self.report("Getting tote at y={}".format(y_pos))

        # If we have room, raise forks and close in on tote.
        if self.drivetrain_sensor_input.get()["y_pos"] < y_pos - .5:
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": y_pos - .5, "r_pos": 0})
            yield from call_public_coroutine("elevator.goto_home")
            yield from call_public_coroutine("drivetrain.wait_for_x")

        # Drive to tote.
        self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": y_pos, "r_pos": 0})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Increase translation tolerance to stop any movement.
        self.drivetrain_config_datastream.push({"trans-tolerance": 1})

        # Grab tote.
        yield from call_public_coroutine("elevator.goto_bottom")

        # Lift tote slightly
        yield from call_public_coroutine("elevator.goto_pos", .5)

        # Decrease translation tolerance back to normal.
        self.drivetrain_config_datastream.push({"trans-tolerance": .2})

        # Set elevator to lift before exiting
        call_public_method("elevator.set_setpoint", 2)

    @asyncio.coroutine
    def move_container(self, y_pos):
        self.report("Moving container at y={}".format(y_pos))

        # Set the x and y setpoint to off the corner of the container (If it had a corner!)
        self.drivetrain_setpoint_datastream.push({"x_pos": 2.5, "y_pos": y_pos - 1, "r_pos": 0})
        self.check_mode()

        # Wait until we clear the container (cutting the corner slightly)
        while self.drivetrain_sensor_input.get().get("x_pos") < 2:
            yield from asyncio.sleep(.1)
            self.check_mode()

        # Set the correct y_pos
        self.drivetrain_setpoint_datastream.push({"y_pos": y_pos})

        # Wait for x to be in-place
        yield from call_public_coroutine("drivetrain.wait_for_x")

        # Wait for y to be close enough
        while self.drivetrain_sensor_input.get().get("y_pos") < y_pos - .5:
            yield from asyncio.sleep(.1)
            self.check_mode()

        # Move container, waiting for x
        self.drivetrain_setpoint_datastream.push({"x_pos": 0})
        yield from call_public_coroutine("drivetrain.wait_for_x")

    @asyncio.coroutine
    def score_stack(self, stack_x_pos):

        self.report("Scoring stack at x={}".format(stack_x_pos))

        # Strafe to stack x pos
        self.drivetrain_setpoint_datastream.push({"x_pos": stack_x_pos})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Drop stack
        yield from call_public_coroutine("elevator.goto_bottom")

        # Get stack y and back off
        stack_y = self.drivetrain_sensor_input.get()["y_pos"]
        self.drivetrain_setpoint_datastream.push({"y_pos": stack_y - 2})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")


    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            self.PAUSE = wpilib.SmartDashboard.getNumber("pause_duration")
            call_public_method("drivetrain.auto_drive_enable")
            call_public_method("drivetrain.reset_sensor_input")
            self.reset_auto_time()

            yield from self.get_tote(0)
            yield from self.do_pause()
            yield from self.move_container(2.7)
            yield from self.do_pause()
            yield from self.get_tote(6.5)
            yield from self.do_pause()
            yield from self.move_container(9.2)
            yield from self.do_pause()
            yield from self.get_tote(13)
            yield from self.do_pause()
            yield from self.score_stack(8)

            self.logger.info("Autonomous routine took {} seconds total".format(self.get_auto_time()))

            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")



