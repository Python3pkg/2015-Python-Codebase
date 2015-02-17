import asyncio
import yeti
import wpilib
import math
from yeti.wpilib_extensions import Referee
from yeti.interfaces import gamemode, remote_methods

class AdvancedElevator(yeti.Module):
    """An advanced CAN controller for an elevator"""

    ####################################
    # CONFIGURATION

    # The list of CAN ids for the CAN Jaguars
    # First value is assumed to be master, all others will be slaves
    MASTER_CAN_ID = 14
    SLAVE_CAN_IDS = []

    # PID Vals
    JAG_SPEED_P = 0.500
    JAG_SPEED_I = 0.000
    JAG_SPEED_D = 0.000

    JAG_DIST_P = 1000.000
    JAG_DIST_I = 2.000
    JAG_DIST_D = 0.000

    # Manual Control
    MANUAL_MAX_SPEED = 5
    MANUAL_RUN_BUTTON = 1
    MANUAL_RUN_AXIS = 0

    # Method Buttons
    GOTO_HOME_BUTTON = 2
    PICKUP_TOTE_BUTTON = 3
    TOP_STACK_BUTTON = 4

    # Encoder Config
    HOME_POSITION = 2
    POSITION_TOLERANCE = .05
    ENCODER_TICS_PER_ROTATION = 2048
    RPM_PER_FPS = 60/(.25 * math.pi)
    ROT_PER_FOOT = 1/(.25 * math.pi)

    #################################
    # RUN VARS
    manual_run = False
    calibration_ref = 0
    run_events = list()

    def module_init(self):
        self.referee = Referee(self)

        # Setup joystick
        self.joystick = wpilib.Joystick(1)

        # Setup CAN Jaguars

        # Setup Master
        self.master_jaguar = wpilib.CANJaguar(self.MASTER_CAN_ID)
        self.referee.watch(self.master_jaguar)

        # Setup Slaves
        self.slave_jaguars = list()
        for id in self.SLAVE_CAN_IDS:
            canjag = wpilib.CANJaguar(id)
            canjag.setVoltageMode()
            self.referee.watch(canjag)
            self.slave_jaguars.append(canjag)


    @yeti.autorun_coroutine
    @asyncio.coroutine
    def jaguar_slaver(self):

        # Return if we don't have any slaves to control
        if len(self.slave_jaguars) == 0:
            return

        while True:

            while gamemode.is_enabled():
                # Run slaves at the exact voltage the master jaguar is outputting
                wanted_voltage = self.master_jaguar.getOutputVoltage()
                for slave in self.slave_jaguars:
                    slave.set(wanted_voltage)
                yield from asyncio.sleep(.05)

            yield from asyncio.sleep(.1)

    def release_control(self):
        for event in self.run_events:
            event.set()

    def get_control(self):
        self.release_control()
        event = asyncio.Event()
        self.run_events.append(event)
        return event

    @asyncio.coroutine
    @yeti.autorun_coroutine
    def calibrator_loop(self):
        while True:
            if not self.master_jaguar.getReverseLimitOK():
                self.calibration_ref = self.master_jaguar.getPosition()
            yield from asyncio.sleep(.01)

    @asyncio.coroutine
    @gamemode.teleop_task
    def joystick_control(self):
        last_buttons = {"goto_home": True, "pickup_tote": True, "top_stack": True}
        while gamemode.is_teleop():
            # Check all method buttons
            buttons = {}
            buttons["goto_home"] = self.joystick.getRawButton(self.GOTO_HOME_BUTTON)
            buttons["pickup_tote"] = self.joystick.getRawButton(self.PICKUP_TOTE_BUTTON)
            buttons["top_stack"] = self.joystick.getRawButton(self.TOP_STACK_BUTTON)

            if not self.manual_run:

                if buttons["goto_home"] and not last_buttons["goto_home"]:
                    self.start_coroutine(self.goto_home())
                elif buttons["pickup_tote"] and not last_buttons["pickup_tote"]:
                    self.start_coroutine(self.pickup_tote())

            last_buttons = buttons.copy()

            yield from asyncio.sleep(.1)

    # Mode setting helpers
    def set_position_mode(self):
        self.logger.info("Set position mode")
        if self.master_jaguar.getControlMode() != wpilib.CANJaguar.ControlMode.Position:
            self.master_jaguar.setPositionModeQuadEncoder(self.ENCODER_TICS_PER_ROTATION, self.JAG_DIST_P, self.JAG_DIST_I, self.JAG_DIST_D)

    def set_percent_mode(self):
        self.logger.info("Set percent mode")
        if self.master_jaguar.getControlMode() != wpilib.CANJaguar.ControlMode.PercentVbus:
            self.master_jaguar.setPercentModeQuadEncoder(self.ENCODER_TICS_PER_ROTATION)


    @gamemode.teleop_task
    @asyncio.coroutine
    def manual_run_loop(self):
        while gamemode.is_teleop():
            if self.joystick.getRawButton(self.MANUAL_RUN_BUTTON):
                self.release_control()
                self.manual_run = True
                self.set_percent_mode()
                while self.joystick.getRawButton(self.MANUAL_RUN_BUTTON):
                    axis_val = self.joystick.getRawAxis(self.MANUAL_RUN_AXIS)
                    self.master_jaguar.set(axis_val)
                    yield from asyncio.sleep(.05)
                self.manual_run = False
                self.master_jaguar.set(0)
            yield from asyncio.sleep(.1)

    # Distance mode helpers
    def set_distance_setpoint(self, value):
        self.set_position_mode()
        self.master_jaguar.set(value * self.ROT_PER_FOOT)
        self.logger.info("Setting distance setpoint to {}".format(value))

    def is_at_setpoint(self, value):
        return abs(self.master_jaguar.getPosition() - (value * self.ROT_PER_FOOT)) < self.POSITION_TOLERANCE

    @asyncio.coroutine
    def goto_pos(self, value):
        stop_event = self.get_control()
        self.set_distance_setpoint(value)
        while not stop_event.is_set() and not self.is_at_setpoint(value):
            yield from asyncio.sleep(.1)
        return

    @asyncio.coroutine
    def goto_home(self):
        self.logger.info("Goto home!")
        yield from self.goto_pos(self.HOME_POSITION)
        self.logger.info("Ending goto home!")

    @asyncio.coroutine
    def goto_bottom(self):
        self.logger.info("Goto bottom!")
        yield from self.goto_pos(0)
        self.logger.info("Ending goto bottom!")

    @asyncio.coroutine
    def pickup_tote(self):
        self.logger.info("Getting tote!")
        stop_event = self.get_control()
        self.set_distance_setpoint(0)
        while not stop_event.is_set() and self.joystick.getRawButton(self.PICKUP_TOTE_BUTTON):
            yield from asyncio.sleep(.1)
        if not stop_event.is_set():
            self.set_distance_setpoint(self.HOME_POSITION)
        self.logger.info("Ending getting tote!")


    #Public coroutines with which to control this remotely
    @asyncio.coroutine
    @remote_methods.public_coroutine
    def elevator_goto_home(self):
        yield from self.goto_home()

    @asyncio.coroutine
    @remote_methods.public_coroutine
    def elevator_goto_bottom(self):
        yield from self.goto_bottom()

    @asyncio.coroutine
    @remote_methods.public_coroutine
    def elevator_goto_pos(self, position):
        yield from self.goto_pos(position)