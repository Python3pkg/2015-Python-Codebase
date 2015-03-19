import asyncio
import yeti
import wpilib
import json

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass

class UniversalAuto(yeti.Module):

    PAUSE = 0
    auto_start_timestamp = 0

    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")
        self.drivetrain_config_datastream = datastreams.get_datastream("drivetrain_auto_config")
        self.elevator_setpoint_datastream = datastreams.get_datastream("elevator_setpoint")
        self.elevator_input_datastream = datastreams.get_datastream("elevator_input")
        config_options = [("start_delay", 0),
                          ("tote_one", 0), ("container_one", 0),
                          ("tote_two", 0), ("container_two", 0),
                          ("tote_three", 0), ("container_three", 0),
                          ("platform_align", 0), ("end_routine", 0)]
        for key, value in config_options:
            if wpilib.SmartDashboard.getNumber("autonomous/" + key, value) == value:
                wpilib.SmartDashboard.putNumber("autonomous/" + key, value)
        wpilib.SmartDashboard.putString("autonomous/config_keys", json.dumps([k for k, v in config_options]))

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            # Get auto config
            tote_y_coordinates = [0, 6.5, 13]
            container_y_coordinates = [2.7, 9.2, 15.7]

            tote_commands = [
                wpilib.SmartDashboard.getNumber("autonomous/tote_one"),
                wpilib.SmartDashboard.getNumber("autonomous/tote_two"),
                wpilib.SmartDashboard.getNumber("autonomous/tote_three")]

            container_commands = [
                wpilib.SmartDashboard.getNumber("autonomous/container_one"),
                wpilib.SmartDashboard.getNumber("autonomous/container_two"),
                wpilib.SmartDashboard.getNumber("autonomous/container_three")]

            start_delay = wpilib.SmartDashboard.getNumber("autonomous/start_delay")
            platform_align = wpilib.SmartDashboard.getNumber("autonomous/end_routine")
            end_routine = wpilib.SmartDashboard.getNumber("autonomous/end_routine")

            # Reset and enable autonomous drivetrain
            call_public_method("drivetrain.reset_sensor_input")
            call_public_method("drivetrain.reset_auto_config")
            call_public_method("drivetrain.auto_drive_enable")
            self.reset_auto_time()

            # If start_delay, wait for it before proceeding
            if start_delay > 0:
                self.logger.info("Start delay enabled: sleeping for {} seconds.".format(start_delay))
                yield from asyncio.sleep(start_delay)

            # Iterate through gamepiece commands

            for tote_command, tote_y, container_command, container_y in\
                zip(tote_commands, tote_y_coordinates, container_commands, container_y_coordinates):

                # Do the tote command
                if tote_command == 0:
                    pass
                elif tote_command == 1:
                    yield from self.get_tote(tote_y)

                # Do the container command
                if container_command == 0:
                    pass
                elif container_command == 1:
                    yield from self.move_container(container_y)

            # Align to platform if instructed
            if platform_align == 0:
                pass
            elif platform_align == 1:
                yield from self.platform_align()

            # Perform end routine
            if end_routine == 0:
                pass
            elif end_routine == 1:
                yield from self.score_bot()
            elif end_routine == 2:
                yield from self.score_stack()

            self.logger.info("Autonomous routine took {} seconds total".format(self.get_auto_time()))

            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")

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

    # Gamepiece routines

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
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": y_pos - 1.7, "r_pos": 0})
            yield from call_public_coroutine("elevator.goto_pos", 2.5)
            yield from call_public_coroutine("drivetrain.wait_for_x")

        # Drive to tote.
        self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": y_pos, "r_pos": 0})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Increase y tolerance to stop any movement.
        self.drivetrain_config_datastream.push({"y_tolerance": 1, "x_tolerance": 1})

        # Grab tote.
        yield from call_public_coroutine("elevator.goto_pos", .3)

        # Lift tote slightly
        yield from call_public_coroutine("elevator.goto_pos", .8)

        # Decrease translation tolerance back to normal.
        call_public_method("drivetrain.reset_auto_config")

        # Set elevator to lift before exiting
        call_public_method("elevator.set_setpoint", 2.5)

    @asyncio.coroutine
    def move_container(self, y_pos):
        """
        Drive to clear the container in the x direction and just before the container in the y direction.
        Once we clear on the x axis, drive to the y coordinate of the container. Then drive back to x=0
        and return.
        """
        self.report("Moving container at y={}".format(y_pos))

        # Strafe to the side of the container
        self.drivetrain_setpoint_datastream.push({"x_pos": 2.5, "y_pos": y_pos - 2.5, "r_pos": 0})
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




    ##################################
    # Finalizing procedures

    @asyncio.coroutine
    def platform_align(self):
        """
        Spin 90 degrees and approach the scoring platform. This is so that we don't drive sideways over the scoring platforms.
        """

        self.report("Aligning to scoring platform")

        # Drive to x=4, spinning 90 degrees clockwise
        self.drivetrain_setpoint_datastream.push({"x_pos": 4, "r_pos": -90})
        yield from call_public_coroutine("drivetrain.wait_for_r")
        self.check_mode()

    @asyncio.coroutine
    def score_stack(self):
        """
        Drive to the x position of the scoring spot. Then lower the forks. Then back up 2 feet.
        """
        self.report("Scoring stack")

        # Start lowering the forks
        call_public_method("elevator.set_setpoint", .5)

        # Strafe to stack x pos
        self.drivetrain_setpoint_datastream.push({"x_pos": 10.5})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Drop stack
        yield from call_public_coroutine("elevator.goto_bottom")

        # Get stack y and back off
        stack_y = self.drivetrain_sensor_input.get()["y_pos"]
        self.drivetrain_setpoint_datastream.push({"y_pos": stack_y - 2})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

    @asyncio.coroutine
    def score_bot(self):
        """ Scores the robot, moving to the auto zone at x=10.5 """
        self.report("Scoring robot")

        self.drivetrain_setpoint_datastream.push({"x_pos": 10.5})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")
        self.check_mode()






