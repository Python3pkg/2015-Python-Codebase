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
    TOTE_HEIGHT = 1
    stack_spacing = 3
    held_totes = 0

    def module_init(self):
        self.drivetrain_setpoint_datastream = datastreams.get_datastream("drivetrain_auto_setpoint")
        self.drivetrain_sensor_input = datastreams.get_datastream("drivetrain_sensor_input")
        self.drivetrain_config_datastream = datastreams.get_datastream("drivetrain_auto_config")
        self.elevator_input_datastream = datastreams.get_datastream("elevator_input")
        config_options = [("stack_spacing", 4), ("stack_count", 3),
                          ("starting_totes", 5), ("iterations", 10)]
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

            # Drop forks and back away to start
            yield from call_public_coroutine("elevator.goto_pos", 0)
            self.drivetrain_setpoint_datastream.push({"y_pos": -3})
            yield from call_public_coroutine("drivetrain.wait_for_xyr")

            for n in range(int(iterations)):
                possible_totes = []
                for stack in range(len(self.stack_heights)):
                    possible_totes.extend([(stack, tote) for tote in range(int(self.stack_heights[stack]))])
                stack, tote = random.choice(possible_totes)
                possible_stacks = [s for s in range(stack_count) if s != stack]
                end_stack = random.choice(possible_stacks)
                yield from self.pick_tote(stack, tote)
                yield from self.drop_stack(end_stack)

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
        self.report("Getting tote from stack={}, level={}".format(stack, level))

        # Drive to the target stack
        yield from self.goto_stack(stack)

        # Ensure forks clear stack
        min_fork_height = (self.stack_heights[stack] + 1) * self.TOTE_HEIGHT
        if self.elevator_input_datastream.get()["pos"] < min_fork_height:
            yield from call_public_coroutine("elevator.goto_pos", min_fork_height)

        # Drive in close to stack.
        self.drivetrain_setpoint_datastream.push({"y_pos": 0})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        tote_height = level * self.TOTE_HEIGHT

        # Lower forks to below target tote.
        yield from call_public_coroutine("elevator.goto_pos", tote_height)

        # Lift tote slightly
        yield from call_public_coroutine("elevator.goto_pos", tote_height + 1)

        # Set stack to be shortened
        self.held_totes = self.stack_heights[stack] - level
        self.stack_heights[stack] = level


    @asyncio.coroutine
    def goto_stack(self, stack_num):
        """
        Navigate to the indicated stack and cover it
        """
        # Back up so that forks clear any existing stack.
        self.drivetrain_setpoint_datastream.push({"y_pos": -3})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

        # Set drivetrain setpoint
        self.drivetrain_setpoint_datastream.push({"x_pos": stack_num * self.stack_spacing})

        # Move forks
        yield from call_public_coroutine("elevator.goto_pos", (self.stack_heights[stack_num] + 1) * self.TOTE_HEIGHT)

        # Wait for drivetrain setpoint
        yield from call_public_coroutine("drivetrain.wait_for_xyr")

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

        # Set stack to be extended
        self.stack_heights[stack] += self.held_totes
        self.held_totes = 0

        # Back away from stack.
        self.drivetrain_setpoint_datastream.push({"y_pos": -3})
        yield from call_public_coroutine("drivetrain.wait_for_xyr")