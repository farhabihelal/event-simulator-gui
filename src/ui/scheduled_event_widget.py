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
            text_font=("Roboto Medium", 42),
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
    from virtual_clock import VirtualClockWidget

    app = ctk.CTk()
    app.geometry("600x400")

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../../res/icons")

    config = {
        "table": {
            "events": [x for x in EVENT_DEFINITIONS if x["type"] in ["scheduled"]],
            "icons": icon_db.icons,
        },
        "clock": {
            "vclock": {
                "virtual_second": 1 / 7200,
                "loop_rate": 1000,
            },
            "icons": {
                "play": icon_db.icons["forward-fast-solid-32"],
                "stop": icon_db.icons["stop-solid-32"],
            },
            "font": ("Roboto Medium", 40),
        },
    }
    widget = ScheduledEventWidget(config, master=app)
    widget.pack(expand=True)

    app.mainloop()
