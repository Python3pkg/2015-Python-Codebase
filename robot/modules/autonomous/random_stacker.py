import asyncio
import yeti
import wpilib
import json
import random

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import call_public_method, call_public_coroutine


class EndOfAutoException(Exception):
    pass


class RandomStacker(yeti.Module):

    PAUSE = 0
    auto_start_timestamp = 0
    stack_heights = None
    TOTE_HEIGHT = 2
    stack_spacing = 3

    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")
        self.drivetrain_config_datastream = datastreams.get_datastream("drivetrain_auto_config")
        self.elevator_input_datastream = datastreams.get_datastream("elevator_input")
        config_options = [("stack_spacing", 3), ("stack_count", 3),
                          ("starting_totes", 4), ("iterations", 1)]
        for key, value in config_options:
            if wpilib.SmartDashboard.getNumber("autonomous/" + key, value) == value:
                wpilib.SmartDashboard.putNumber("autonomous/" + key, value)
        wpilib.SmartDashboard.putString("autonomous/config_keys", json.dumps([k for k, v in config_options]))

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            # Get auto config

            self.stack_spacing = wpilib.SmartDashboard.getNumber("autonomous/stack_spacing")
            stack_count = int(wpilib.SmartDashboard.getNumber("autonomous/stack_count"))
            starting_totes = int(wpilib.SmartDashboard.getNumber("autonomous/starting_totes"))
            iterations = int(wpilib.SmartDashboard.getNumber("autonomous/iterations"))

            # Reset tote memory
            self.stack_heights = [0 for _ in range(int(stack_count))]
            self.stack_heights[0] = starting_totes

            # Reset and enable autonomous drivetrain
            call_public_method("drivetrain.reset_sensor_input")
            call_public_method("drivetrain.reset_auto_config")
            self.drivetrain_setpoint_datastream.push({"x_pos": 0, "y_pos": 0, "r_pos": 0})
            call_public_method("drivetrain.auto_drive_enable")
            self.reset_auto_time()

            for n in range(int(iterations)):
                possible_totes = []
                for stack in range(len(self.stack_heights)):
                    possible_totes = [(stack, tote) for tote in range(int(self.stack_heights[stack]))]
                possible_stacks = range(stack_count)
                stack, tote = random.choice(possible_totes)
                end_stack = random.choice(possible_stacks)
                yield from self.pick_tote(stack, tote)
                yield from self.goto_stack(end_stack)

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

        # Drive to the target stack
        yield from self.goto_stack(stack)

        # Drive in close to stack.
        self.drivetrain_setpoint_datastream.push({"y_pos": 0})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        tote_height = stack * self.TOTE_HEIGHT

        # Lower forks to below target tote.
        yield from call_public_coroutine("elevator.goto_pos", tote_height)

        # Lift tote slightly
        yield from call_public_coroutine("elevator.goto_pos", tote_height + 1)


    @asyncio.coroutine
    def goto_stack(self, stack_num):
        """
        Navigate to the indicated stack and cover it
        """
        # Back up so that forks clear any existing stack.
        self.drivetrain_setpoint_datastream.push({"y_pos": -3})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Figure out what stack we are currently on
        x_pos = self.drivetrain_sensor_input.get()["x_pos"]
        current_stack = int(x_pos/self.stack_spacing)
        target_stack = current_stack

        while target_stack != stack_num:
            if target_stack < stack_num:
                target_stack += 1
            else:
                target_stack -= 1
            next_stack_tote_count = self.stack_heights[target_stack]
            min_fork_height = (next_stack_tote_count + 1) * self.TOTE_HEIGHT
            if self.elevator_input_datastream.get()["pos"] < min_fork_height:
                yield from call_public_coroutine("elevator.goto_pos", min_fork_height)
            self.drivetrain_setpoint_datastream.push({"x_pos": target_stack * self.stack_spacing})
        yield from call_public_coroutine("drivetrain.wait_for_x")

    @asyncio.coroutine
    def drop_stack(self, stack):
        """
        Grab the tote from the indicated stack at the indicated level, (eg, stack 1 (second from left) level 1 (second from bottom))
        """
        self.report("Placing tote on stack={}".format(stack))

        # Drive to the target stack
        yield from self.goto_stack(stack)

        # Drive in close to stack.
        self.drivetrain_setpoint_datastream.push({"y_pos": 0})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Drop stack.
        yield from call_public_coroutine("elevator.goto_pos", self.stack_heights[stack] * self.TOTE_HEIGHT)

        # Back away from stack.
        self.drivetrain_setpoint_datastream.push({"y_pos": -3})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")