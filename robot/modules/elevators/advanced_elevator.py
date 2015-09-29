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

    POSITION_TOLERANCE = .2
    ENCODER_TICS_PER_ROTATION = 400
    ROT_PER_FOOT = 1/(.25 * math.pi)

    #################################
    # RUN VARS
    lift_setpoint = 0

    def module_init(self):
        self.gameclock = self.engine.get_module("gameclock")
        self.pipeline = self.engine.get_module("pipeline")

        # Setup joystick
        self.joystick = wpilib.Joystick(1)

        # Setup CAN jaguar
        self.lift_jaguar = wpilib.CANJaguar(self.JAG_CAN_ID)
        self.lift_jaguar.setPercentModeQuadEncoder(self.ENCODER_TICS_PER_ROTATION)

        # Configure IO
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
            state["lift_calibrated"] = True
        else:
            state["lift_pos"] = last_state["lift_pos"] + sensors["lift_encoder"][1]*dt/self.ROT_PER_FOOT
        return state

    def pipeline_control_update(self, dt, state):
        if self.gameclock.is_autonomous():
            error = self.lift_setpoint - state["lift_pos"]
            if not state["lift_calibrated"] or error < -self.POSITION_TOLERANCE:
                return {"lift_motor": -1}
            elif error > self.POSITION_TOLERANCE:
                return {"lift_motor": 1}
            else:
                return {"lift_motor": 0}
        else:
            if state["lift_calibrated"]:
                self.lift_setpoint = state["lift_pos"]
            return {"lift_motor": self.joystick.getY()}

    def enabled_init(self):
        self.lift_jaguar.enableControl()

    def disabled_init(self):
        self.lift_jaguar.disableControl()

    def get_position(self):
        return self.pipeline.state["lift_pos"]

    def set_setpoint(self, value):
        self.pipeline.control["lift_setpoint"] = value

    @asyncio.coroutine
    def goto_pos(self, value):

        # Configure control setpoint
        self.set_setpoint(value)
        self.pipeline.control["lift_auto"] = True

        # Wait for lift to be calibrated and at the desired value
        yield from self.pipeline.until_state("lift_calibrated", True)
        yield from self.pipeline.until_state("lift_pos", value, self.POSITION_TOLERANCE)

    def module_deinit(self):
        self.lift_jaguar.free()
