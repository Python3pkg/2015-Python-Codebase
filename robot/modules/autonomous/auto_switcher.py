import yeti
import asyncio
import wpilib
import json

class AutoSwitcher(yeti.Module):

    auto_module_paths = []
    sd_prefix = "autonomous/"

    def module_init(self):
        self.auto_mode_loader = yeti.ModuleLoader()
        self.context = yeti.get_context()
        self.auto_mode_loader.set_context(self.context)
        self.auto_module_paths = self.context.config_manager.config_structure["autonomous"]
        self.auto_module_paths.remove(self.loader.module_path)
        self.auto_mode_loader.set_fallback(self.auto_module_paths)
        data = json.dumps(self.auto_module_paths)
        wpilib.SmartDashboard.putString(self.sd_prefix + "auto_modules", data)
        wpilib.SmartDashboard.putNumber(self.sd_prefix + "selected_auto", 0)

    @asyncio.coroutine
    def load_auto_module(self, path):
        wpilib.SmartDashboard.putString(self.sd_prefix + "config_keys", "[]")
        if path == "":
            yield from self.auto_mode_loader.unload_coroutine()
        else:
            yield from self.auto_mode_loader.load_coroutine(path)

    @asyncio.coroutine
    @yeti.autorun_coroutine
    def run_loop(self):
        last_selected_auto = ""
        while True:
            selected_auto_id = int(wpilib.SmartDashboard.getNumber(self.sd_prefix + "selected_auto"))
            if selected_auto_id < len(self.auto_module_paths):
                selected_auto = self.auto_module_paths[selected_auto_id]
            else:
                selected_auto = ""
            if last_selected_auto != selected_auto:
                if selected_auto != "":
                    self.logger.info("Loading " + selected_auto + "!")
                yield from self.load_auto_module(selected_auto)
                last_selected_auto = selected_auto
            yield from asyncio.sleep(.5)

    def module_deinit(self):
        self.auto_mode_loader.unload()

