import os
import sys


sys.path.append(f"{os.path.dirname(__file__)}/../..")

import logging
import tkinter as tk
import tkinter.ttk as ttk

import customtkinter as ctk

from src.icon_db import IconDB
from src.event_definition import EVENT_DEFINITIONS

if __name__ != "__main__":
    from .event_table import ScheduledEventTable
    from .virtual_clock_widget import VirtualClockWidget


class ScheduledEventWidget(ctk.CTkFrame):
    def __init__(self, config={}, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._config = config

        self.label = ctk.CTkLabel(
            master=self,
            anchor="center",
            font=("Roboto Medium", 42),
            justify="center",
            text=f"scheduled events".title(),
            corner_radius=8,
        )
        self.label.pack(padx=30, pady=10, expand=True)

        self.vclock = VirtualClockWidget(self._config["clock"], master=self)
        self.vclock.pack(padx=10, pady=10, expand=True)

        table_config = dict(self._config["table"])
        table_config.update(
            {
                "clock": self.vclock,
            }
        )
        self.table = ScheduledEventTable(table_config, master=self)
        self.table.pack(padx=10, pady=30, expand=True)


if __name__ == "__main__":
    from event_table import ScheduledEventTable
    from virtual_clock_widget import VirtualClockWidget

    from pydispatch import dispatcher

    import rospy
    from event_simulator_gui.srv import ClockCommand, ClockCommandResponse

    def handle_clock_command(request):
        command = request.command.lower()
        parameter = request.parameter.lower()

        if command == "start":
            dispatcher.send("clock_start")

        elif command == "stop":
            dispatcher.send("clock_stop")

        elif command == "reset":
            dispatcher.send("clock_reset")

        elif command == "set_time":
            dispatcher.send("clock_set_time", datetime_str=parameter)

        elif command == "set_initial_time":
            dispatcher.send("clock_set_initial_time", datetime_str=parameter)

        elif command == "normal":
            dispatcher.send("clock_set_speed", multiplier=1)

        elif command == "fast":
            dispatcher.send("clock_set_speed", multiplier=4)

        elif command == "faster":
            dispatcher.send("clock_set_speed", multiplier=8)

        elif command == "fastest":
            dispatcher.send("clock_set_speed", multiplier=16)

        return ClockCommandResponse()

    rospy.init_node("scheduled_event_widgets")
    rospy.Service("/vclock/cmd", ClockCommand, handle_clock_command)

    app = ctk.CTk()
    app.geometry("1120x780")
    app.title("Scheduled Events")

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../../res/icons")

    config = {
        "table": {
            "events": [x for x in EVENT_DEFINITIONS if x["eventType"] in ["periodic"]],
            "icons": icon_db.icons,
        },
        "clock": {
            "vclock": {
                "initial_time": "2023-01-01 05:00:00",
                "virtual_second": 60 / (4 * 60 * 60),
                "loop_rate": 200,
            },
            "icons": {
                "play": icon_db.icons["forward-fast-solid-32"],
                "stop": icon_db.icons["stop-solid-32"],
            },
            "font": ("Roboto Medium", 80),
        },
    }
    widget = ScheduledEventWidget(config, master=app)
    widget.pack(expand=True)

    app.mainloop()
