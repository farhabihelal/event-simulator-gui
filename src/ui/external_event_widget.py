import os
import sys


sys.path.append(f"{os.path.dirname(__file__)}/../..")

if __name__ != "__main__":
    from .external_event import ExternalEventGUI

from src.icon_db import IconDB

import logging
import tkinter as tk
import tkinter.ttk as ttk

import customtkinter as ctk

from src.event_definition import EVENT_DEFINITIONS, EVENT_UI_DEFINITIONS


class ExternalEventWidget(ctk.CTkFrame):
    def __init__(self, config={}, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._config = config
        self._events = self._config["events"]

        self.label = ctk.CTkLabel(
            master=self,
            anchor="center",
            font=("Roboto Medium", 42),
            justify="center",
            text=f"external events".title(),
            corner_radius=8,
        )
        self.label.pack(ipadx=30, ipady=30)

        self.frm_events = ctk.CTkFrame(master=self)
        self.frm_events.pack()

        self.guis = []

        self.create_guis()

    def create_guis(self):
        for i, e in enumerate(self._events):
            config = {
                "event": e,
                "ui": {
                    "image": self._config["icons"][
                        self._config["ui_confs"][e["id"]]["image"]
                    ],
                },
                "font": ("Roboto Medium", 20),
            }
            gui = ExternalEventGUI(config=config, master=self.frm_events)
            gui.grid(row=int(i / 3), column=i % 3)

            self.guis.append(gui)


if __name__ == "__main__":
    from external_event import ExternalEventGUI

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("dark-blue")

    import rospy

    rospy.init_node("external_event_widget")

    app = ctk.CTk()
    app.title("Event Table")
    app.geometry("820x460")

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../../res/icons")
    config = {
        "events": [x for x in EVENT_DEFINITIONS if x["eventType"] in ["external"]],
        "ui_confs": EVENT_UI_DEFINITIONS,
        "icons": icon_db.icons,
    }
    widget = ExternalEventWidget(config, master=app)
    widget.pack(expand=True, fill=tk.BOTH)
    app.mainloop()
