from tkinter import *
from networktables import *
import time
import threading
import json
from functools import partial
from subprocess import Popen

root = Tk()
root.title("Auto Mode Selector")
root.resizable(width=0, height=0)
mainframe = Frame(root)
mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
mainframe.columnconfigure(0, weight=1)
mainframe.rowconfigure(0, weight=1)

auto_module_listbox = Listbox(mainframe, width=40)
auto_module_listbox.grid(column=1, row=0, sticky=N)

options_frame = Frame(mainframe, width=200)
options_frame.grid(column=2, row=0, sticky=(N, W, E, S))

# Start networktables
NetworkTable.setIPAddress("roborio-4819.local")
#NetworkTable.setIPAddress("127.0.0.1")
NetworkTable.setClientMode()
NetworkTable.initialize()

table = NetworkTable.getTable("SmartDashboard/autonomous")

def launch_video():
    Popen("FlatMountainView.bat")

video_launch_button = Button(mainframe, text="Launch Video Stream", command=launch_video)
video_launch_button.grid(column=0, row=0, sticky=N)

def update_listbox(listbox, newdata):

    # Figure out what entry to select when we are done
    sel_ids = listbox.curselection()
    new_selected_index = 0
    if len(sel_ids) > 0:
        new_selected_index = sel_ids[0]

    # Empty and refill listbox
    listbox.delete(0, END)
    for entry in newdata:
        listbox.insert(END, entry)

    # Set selection
    listbox.selection_set(new_selected_index)
    listbox.selection_anchor(new_selected_index)

die = False

auto_modules = []

last_auto = 0
current_auto = 0

current_exposed_options = []
stored_options = {}
option_widgets = {}

def option_write(key, var, *args):
    val = var.get()
    if val == "":
        val = 0
    try:
        table.putNumber(key, float(val))
    except ValueError:
        val = table.getNumber(key)
        var.set(val)

def display_options():
    # Save current values
    if last_auto != "":
        if last_auto not in stored_options:
            stored_options[last_auto] = {}
        for option_key in option_widgets:
            stored_options[last_auto][option_key] = option_widgets[option_key]["var"].get()

    # Delete widgets
    while len(option_widgets) > 0:
        _, widget = option_widgets.popitem()
        widget["frame"].pack_forget()
        widget["frame"].destroy()

    # Create new ones
    auton_option_keys = json.loads(table.getString("config_keys", "[]"))
    def_vals = stored_options.get(current_auto, {})
    for option_key in auton_option_keys:
        option_frame = Frame(options_frame)
        option_label = Label(option_frame, text=option_key)
        option_label.pack(side=LEFT)
        option_var = StringVar()
        option_var.trace("w", partial(option_write, option_key, option_var))
        option_entry = Entry(option_frame, textvariable=option_var)
        option_entry.pack(side=RIGHT)
        option_entry.get()
        option_frame.pack()
        if option_key in def_vals:
            option_var.set(def_vals[option_key])
        else:
            option_var.set(table.getNumber(option_key, 0))
        option_widgets[option_key] = {"frame": option_frame, "entry": option_entry, "label": option_label, "var": option_var}


def run():
    global auto_modules, current_auto, last_auto
    last_auto_modules_str = ""
    while not die:
        try:
            auto_modules_str = table.getString("auto_modules", "[]")
            if last_auto_modules_str != auto_modules_str:
                auto_modules = json.loads(auto_modules_str)
                update_listbox(auto_module_listbox, auto_modules)
                last_auto_modules_str = auto_modules_str

            current_auto_index = int(table.getNumber("loaded_auto", 0))
            current_auto = ""
            if current_auto_index < len(auto_modules):
                current_auto = auto_modules[current_auto_index]

            if current_auto != last_auto:
                display_options()
                last_auto = current_auto

            ui_selected_auto = auto_module_listbox.get(ANCHOR)
            if ui_selected_auto != current_auto:
                table.putNumber("selected_auto", auto_modules.index(ui_selected_auto))

        except KeyError:
            time.sleep(1)
        time.sleep(.1)

runthread = threading.Thread(target=run)
runthread.start()
mainloop()
#NetworkTable.Shutdown()
die = True
runthread.join()
