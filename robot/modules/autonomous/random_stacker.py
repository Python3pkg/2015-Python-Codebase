import asyncio
import yeti
import wpilib
import json

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass


class RandomStacker(yeti.Module):

    PAUSE = 0
    auto_start_timestamp = 0
    stack_heights = None

    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")
        self.drivetrain_config_datastream = datastreams.get_datastream("drivetrain_auto_config")
        self.elevator_setpoint_datastream = datastreams.get_datastream("elevator_setpoint")
        self.elevator_input_datastream = datastreams.get_datastream("elevator_input")
        config_options = [("stack_spacing", 3), ("stack_count", 3),
                          ("starting_totes", 0), ("iterations", 0)]
        for key, value in config_options:
            if wpilib.SmartDashboard.getNumber("autonomous/" + key, value) == value:
                wpilib.SmartDashboard.putNumber("autonomous/" + key, value)
        wpilib.SmartDashboard.putString("autonomous/config_keys", json.dumps([k for k, v in config_options]))

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            # Get auto config

            stack_spacing = wpilib.SmartDashboard.getNumber("autonomous/stack_spacing")
            stack_count = wpilib.SmartDashboard.getNumber("autonomous/stack_count")
            starting_totes = wpilib.SmartDashboard.getNumber("autonomous/starting_totes")
            iterations = wpilib.SmartDashboard.getNumber("autonomous/iterations")

            # Reset tote memory
            self.stack_heights = [0] * stack_count
            self.stack_heights[0] = starting_totes

            # Reset and enable autonomous drivetrain
            call_public_method("drivetrain.reset_sensor_input")
            call_public_method("drivetrain.reset_auto_config")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0, "r_pos": 0})
            call_public_method("drivetrain.auto_drive_enable")
            self.reset_auto_time()

            # TODO Add main control loop here.

            self.logger.info("Autonomous routine took {} seconds total".format(self.get_auto_time()))

            call_public_method("drivetrain.auto_drive_disable")
            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")

    def check_mode(self):
        if not gamemode.is_autonomous():
            raise EndOfAutoException

    def reset_auto_time(self):
        self.auto_start_timestamp = wpilib.Timer.getFPGATimestamp()

    def get_auto_time(self):
        return wpilib.Timer.getFPGATimestamp() - self.auto_start_timestamp

    def report(self, msg):
        self.logger.info("{} at {} seconds".format(msg, self.get_auto_time()))


    @asyncio.coroutine
    def pick_tote(self, stack, level):
        """
        Grab the tote from the indicated stack at the indicated level, (eg, stack 1 (second from left) level 1 (second from bottom))
        """
        self.report("Getting tote at stack={}, level={}".format(stack, level))

    @asyncio.coroutine
    def goto_stack(self, stack_num, stack_spacing):
        """
        Navigate to the indicated stack and cover it
        """
        # Back up so that forks clear any existing stack.
        self.drivetrain_setpoint_datastream.push({"y_pos": -3})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Figure out what stack we are currently on
        x_pos = self.drivetrain_sensor_input.get()["x_pos"]
        current_stack = int(x_pos/stack_spacing)
        target_stack = current_stack

        while target_stack != stack_num:
            pass

