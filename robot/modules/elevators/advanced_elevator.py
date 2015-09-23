import asyncio
import yeti
import wpilib
import math

class AdvancedElevator(yeti.Module):
    """An advanced CAN controller for an elevator"""

    ####################################
    # CONFIGURATION

    # The CAN id for the CAN Jaguar
    JAG_CAN_ID = 10

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
        self.gameclock = self.engine.get_module("gameclock")
        self.pipeline = self.engine.get_module("pipeline")

        # Setup joystick
        self.joystick = wpilib.Joystick(1)

        # Setup CAN jaguar
        self.lift_jaguar = wpilib.CANJaguar(self.JAG_CAN_ID)
        self.lift_jaguar.setPercentModeQuadEncoder(self.ENCODER_TICS_PER_ROTATION)
        self.pipeline.add_sensor_poll(self.lift_jaguar.get, "lift_encoder", deriv_order=1)
        self.pipeline.add_sensor_poll(self.lift_jaguar.get, "lift_limit_upper")
        self.pipeline.add_sensor_poll(self.lift_jaguar.get, "lift_limit_lower")
        self.pipeline.add_output_poll(self.lift_jaguar.set, "lift_motor")

    def pipeline_sensor_prediction(self, dt, output, last_sensor):
        sensor = {}
        if last_sensor["lift_encoder"] >= 8*self.ROT_PER_FOOT*self.ENCODER_TICS_PER_ROTATION:
            sensor["lift_limit_upper"] = True
            sensor["lift_limit_lower"] = False
            sensor["lift_encoder"] = last_sensor["lift_encoder"] + min(output["lift_motor"], 0)*8*dt
        elif last_sensor["lift_encoder"] <= 0:
            sensor["lift_limit_upper"] = True
            sensor["lift_limit_lower"] = False
            sensor["lift_encoder"] = last_sensor["lift_encoder"] + max(output["lift_encoder"], 0)*8*dt
        else:
            sensor["lift_limit_upper"] = False
            sensor["lift_limit_lower"] = False
            sensor["lift_encoder"] = last_sensor["lift_encoder"] + output["lift_encoder"]*8*dt
        return sensor

    def pipeline_state_update(self, dt, sensors, last_state):
        state = {}
        if sensors["lift_limit_lower"]:
            state["lift_pos"] = 0
        else:
            state["lift_pos"] = last_state["lift_pos"] + sensors["lift_encoder"][1]*dt
        return state

    def pipeline_control_update(self, dt, state, control):
        return {"lift_motor": control["lift_pwr"]}

    def teleop_periodic(self):
        self.pipeline.control["lift_pwr"] = self.joystick.getY()

    @asyncio.coroutine
    def run_loop(self):
        self.setpoint = self.get_position()
        self.lift_jaguar.enableControl()
        while True:

            if self.NT_DEBUG_OUT:
                wpilib.SmartDashboard.putNumber("elvevator_pos", self.get_position())
                wpilib.SmartDashboard.putNumber("elvevator_setpoint", self.setpoint)
                wpilib.SmartDashboard.putBoolean("elvevator_calibrated", self.calibrated)

            if not self.lift_jaguar.getForwardLimitOK():
                self.calibration_ref = -self.lift_jaguar.getPosition()
                self.calibrated = True

            output = 0
            if self.gameclock.is_teleop():
                output = self.joystick.getY()
                self.setpoint = self.get_position()
            elif self.gameclock.is_autonomous():

                # If setpoint is zero, always go down.
                if self.setpoint <= 0 or not self.calibrated:
                    if self.lift_jaguar.getForwardLimitOK():
                        output = -1
                    else:
                        output = 0
                else:
                    pos_delta = self.setpoint - self.get_position()

                    if abs(pos_delta) > self.POSITION_TOLERANCE:
                        output = 1
                        if pos_delta < 0:
                            output = -output

            self.lift_jaguar.set(-output)
            wpilib.SmartDashboard.putNumber("elevator output", output)

            yield from asyncio.sleep(.05)
        self.lift_jaguar.disableControl()

    def get_position(self):
        return (-self.lift_jaguar.getPosition() - self.calibration_ref) / self.ROT_PER_FOOT


    def set_setpoint(self, value):
        self.setpoint = value

    @asyncio.coroutine
    def goto_pos(self, value):
        self.logger.info("Goto {}".format(value))
        self.set_setpoint(value)
        while True:
            pos = self.get_position()
            if abs(value - pos) <= self.POSITION_TOLERANCE and self.calibrated:
                break
            if not self.gameclock.is_autonomous():
                break
            self.set_setpoint(value)
            yield from asyncio.sleep(.1)
        self.logger.info("End goto {}".format(value))

    @asyncio.coroutine
    def goto_home(self):
        yield from self.goto_pos(self.HOME_POSITION)

    @asyncio.coroutine
    def goto_bottom(self):
        yield from self.goto_pos(0)

    def module_deinit(self):
        self.lift_jaguar.free()
