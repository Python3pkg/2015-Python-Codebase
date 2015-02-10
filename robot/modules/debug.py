import asyncio
import wpilib
import yeti
from yeti.wpilib_extensions.buttons import Button

class Debug(yeti.Module):
    """
    A simple module for reloading all modules with the push of a joystick button.
    """

    def module_init(self):
        self.joystick = wpilib.Joystick(0)
        self.start_coroutine(self.reload_mods())
        self.motor = wpilib.Talon(5)
        wpilib.LiveWindow.setEnabled(True)
        for i in range(20):
            wpilib.SmartDashboard.putString("message" + str(i), "Hi!")

    @asyncio.coroutine
    def reload_mods(self):
        """Reload all mods when button 10 is pressed."""
        context = yeti.get_context()

        #Wait until the button is pressed.
        yield from Button(self.joystick, 10).until_rising()

        #Reload all modules with a loader object
        loaded_mods = context.get_modules()
        for module in loaded_mods:
            if hasattr(module, "loader"):
                module.loader.reload()