import asyncio
import yeti
import wpilib
import pickle


class DrivetrainDebugger(yeti.Module):

    # Data sample rate in Hz
    SAMPLE_RATE = 20

    last_run_data = None
    data = b""

    def module_init(self):
        self.drivetrain_mod = self.engine.get_module("drivetrain.advanced_can_mecanum")
        self.gameclock = self.engine.get_module("gameclock")

        self.start_coroutine(self.start_server())
        wpilib.SmartDashboard.putNumber("drive_jag_p", 1.000)
        wpilib.SmartDashboard.putNumber("drive_jag_i", 0.005)
        wpilib.SmartDashboard.putNumber("drive_jag_d", 0.000)

    @asyncio.coroutine
    def start_server(self):
        modobj = self

        class DataServer(asyncio.Protocol):
            data = b""

            def connection_made(self, transport):
                transport.write(modobj.data + b"FIN")
                transport.close()
        self.server = yield from self.event_loop.create_server(DataServer, port=2222)

    @yeti.autorun
    @asyncio.coroutine
    def pid_updater(self):
        last_p = 0
        last_i = 0
        last_d = 0
        while True:
            yield from asyncio.sleep(.25)
            p = wpilib.SmartDashboard.getNumber("drive_jag_p", 1.000)
            i = wpilib.SmartDashboard.getNumber("drive_jag_i", 0.005)
            d = wpilib.SmartDashboard.getNumber("drive_jag_d", 0.000)
            if p != last_p or i != last_i or d != last_d:
                last_p = p
                last_i = i
                last_d = d
                self.drivetrain_mod.set_pid(p, i, d)

    @gamemode.enabled_task
    @asyncio.coroutine
    def monitor_drivetrain(self):
        current_run_data = list()
        while self.gameclock.is_enabled():
            sensor_data = self.sensor_input_datastream.get()
            control_data = self.control_datastream.get()

            control_y_speed = control_data.get("forward_fps", 0)
            sensor_y_speed = sensor_data.get("y_speed", 0)
            current_run_data.append((control_y_speed, sensor_y_speed))

            yield from asyncio.sleep(1/self.SAMPLE_RATE)
        # Pause for a moment to ensure we stop motion prior to blocking when we pickle the data.
        yield from asyncio.sleep(.5)
        data_packet = {"rate": self.SAMPLE_RATE, "data": current_run_data}
        self.data = pickle.dumps(data_packet)
