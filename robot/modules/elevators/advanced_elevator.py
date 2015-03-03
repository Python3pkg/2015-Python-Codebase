import asyncio
import yeti
import wpilib
import math
from yeti.wpilib_extensions import Referee
from yeti.interfaces import gamemode, datastreams
from yeti.interfaces.object_proxy import public_object

class SimulatedCANJaguar():

    MAX_RPM_OUTPUT = 120
    FORWARD_LIMIT = 8
    REVERSE_LIMIT = 0
    STARTING_POS = 5

    def __init__(self, CAN_ID):
        self.talon = wpilib.Talon(CAN_ID - 10)
        self.position = self.STARTING_POS
        self.last_update = wpilib.Timer.getFPGATimestamp()
        self.speed = 0
        self.mode = wpilib.CANJaguar.ControlMode.PercentVbus

    def _reset_physics(self):
        self.position = self.STARTING_POS
        self.speed = 0

    def setSpeedModeQuadEncoder(self, codesPerRev, p, i, d):
        self.mode = wpilib.CANJaguar.ControlMode.Speed
        self._reset_physics()

    def setPercentModeQuadEncoder(self, codesPerRev):
        self.mode = wpilib.CANJaguar.ControlMode.PercentVbus
        self._reset_physics()

    def getReverseLimitOK(self):
        return self.REVERSE_LIMIT is None or self.position > self.REVERSE_LIMIT

    def getForwardLimitOK(self):
        return self.FORWARD_LIMIT is None or self.position < self.FORWARD_LIMIT

    def _update_physics(self):
        current_time = wpilib.Timer.getFPGATimestamp()
        delta_time = current_time - self.last_update
        self.position += self.speed * (delta_time/60)
        if self.position <= 0:
            self.position = 0
            self.speed = 0
        self.last_update = current_time

    def set(self, value):
        self._update_physics()
        if self.mode == wpilib.CANJaguar.ControlMode.Speed:
            self.setSpeed(value)
        elif self.mode == wpilib.CANJaguar.ControlMode.PercentVbus:
            self.setSpeed(value * self.MAX_RPM_OUTPUT)

    def setSpeed(self, speed):
        if (speed < 0 and not self.getReverseLimitOK()) or (speed > 0 and not self.getForwardLimitOK()):
            speed = 0
        self.speed = speed
        self.talon.set(speed / self.MAX_RPM_OUTPUT)

    def get(self):
        return self.speed

    def getControlMode(self):
        return self.mode

    def getSpeed(self):
        return self.speed

    def getPosition(self):
        self._update_physics()
        return self.position

    def enableControl(self):
        pass

    def disableControl(self):
        pass

    def getOutputCurrent(self):
        return 0

    def getOutputVoltage(self):
        return 0

    def free(self):
        self.talon.free()

class AdvancedElevator(yeti.Module):
    """An advanced CAN controller for an elevator"""

    ####################################
    # CONFIGURATION

    # The list of CAN ids for the CAN Jaguars
    # First value is assumed to be master, all others will be slaves
    MASTER_CAN_ID = 10

    USE_SIMULATED_JAGUAR = True
    NT_DEBUG_OUT = True

    # Encoder Config
    HOME_POSITION = 2
    TOTE_HEIGHT = 1.1

    POSITION_TOLERANCE = .2
    ENCODER_TICS_PER_ROTATION = 400
    ROT_PER_FOOT = 1/(.25 * math.pi)

    #################################
    # RUN VARS
    calibration_ref = 0
    calibrated = False

    def module_init(self):
        self.referee = Referee(self)

        # Setup joystick
        self.joystick = wpilib.Joystick(1)

        # Setup CAN Jaguars

        # Setup Master
        if self.USE_SIMULATED_JAGUAR:
            self.master_jaguar = SimulatedCANJaguar(self.MASTER_CAN_ID)
        else:
            self.master_jaguar = wpilib.CANJaguar(self.MASTER_CAN_ID)
        self.master_jaguar.setPercentModeQuadEncoder(self.ENCODER_TICS_PER_ROTATION)
        self.referee.watch(self.master_jaguar)


    @yeti.autorun_coroutine
    @asyncio.coroutine
    def run_loop(self):
        self.setpoint = self.get_position()
        self.master_jaguar.enableControl()
        while True:

            if self.NT_DEBUG_OUT:
                wpilib.SmartDashboard.putNumber("elvevator_pos", self.get_position())
                wpilib.SmartDashboard.putNumber("elvevator_setpoint", self.setpoint)
                wpilib.SmartDashboard.putBoolean("elvevator_calibrated", self.calibrated)

            if not self.master_jaguar.getForwardLimitOK():
                self.calibration_ref = -self.master_jaguar.getPosition()
                self.calibrated = True

            output = 0
            if gamemode.is_teleop():
                output = self.joystick.getY()
                self.setpoint = self.get_position()
            elif gamemode.is_autonomous():

                # If setpoint is zero, always go down.
                if self.setpoint <= 0 or not self.calibrated:
                    if self.master_jaguar.getForwardLimitOK():
                        output = -1
                    else:
                        output = 0
                else:
                    pos_delta = self.setpoint - self.get_position()

                    if abs(pos_delta) > self.POSITION_TOLERANCE:
                        output = 1
                        if pos_delta < 0:
                            output = -output

            self.master_jaguar.set(-output)
            wpilib.SmartDashboard.putNumber("elevator output", output)

            yield from asyncio.sleep(.05)
        self.master_jaguar.disableControl()

    def get_position(self):
        return (-self.master_jaguar.getPosition() - self.calibration_ref) / self.ROT_PER_FOOT

    @public_object(prefix="elevator")
    def set_setpoint(self, value):
        self.setpoint = value

    @public_object(prefix="elevator")
    @asyncio.coroutine
    def goto_pos(self, value):
        self.logger.info("Goto {}".format(value))
        self.set_setpoint(value)
        while True:
            pos = self.get_position()
            if abs(value - pos) <= self.POSITION_TOLERANCE and self.calibrated:
                break
            if not gamemode.is_autonomous():
                break
            yield from asyncio.sleep(.1)
        self.logger.info("End goto {}".format(value))

    @public_object(prefix="elevator")
    @asyncio.coroutine
    def goto_home(self):
        yield from self.goto_pos(self.HOME_POSITION)

    @public_object(prefix="elevator")
    @asyncio.coroutine
    def goto_bottom(self):
        yield from self.goto_pos(0)
