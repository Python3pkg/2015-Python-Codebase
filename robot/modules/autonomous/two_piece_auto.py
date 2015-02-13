import asyncio
import yeti

from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.remote_coroutines import call_public_coroutine


class EndOfAutoException(Exception):
    pass

class TwoPieceAuto(yeti.Module):

    def module_init(self):
        self.drivetrain_control_datastream = datastreams.get_datastream("drivetrain_control")
        self.drivetrain_state_datastream = datastreams.get_datastream("drivetrain_state")

    def check_mode(self):
        if not gamemode.is_autonomous():
            raise EndOfAutoException

    @asyncio.coroutine
    @gamemode.autonomous_task
    def run_auto(self):
        try:
            #Grab tote
            yield from call_public_coroutine("elevator_goto_home")
            self.check_mode()
            yield from call_public_coroutine("elevator_goto_bottom")
            self.check_mode()
            yield from call_public_coroutine("elevator_goto_home")
            self.check_mode()

            # Drive around container
            yield from call_public_coroutine("drivetrain_start_tracking")

            # Drive sideways
            SIDEWAYS_DISTANCE = 3
            while gamemode.is_autonomous():
                current_state_data = self.drivetrain_state_datastream.get()
                # This is formatted as a tuple of (position x, position y, angle)
                position = current_state_data.get("drivetrain_position", 0)
                self.drivetrain_control_datastream.push({"forward_fps": 0, "right_fps": -5, "clockwise_rps": 0})


            #



            while gamemode.is_autonomous():
                yield from asyncio.sleep(.5)
        except EndOfAutoException:
            self.logger.info("Aborted Autonomous mode")



