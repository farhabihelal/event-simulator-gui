import json
import os
import sys
import tkinter as tk
import tkinter.ttk as ttk
import customtkinter as ctk


import rospy

from strawberry_dialogue_knowrob.msg import Event


sys.path.append(f"{os.path.dirname(__file__)}/../..")

from src.ui.event_log_item import EventLogItem
from src.ui.scrollable_frame import ScrollableFrame

from src.event_definition import EVENT_DEFINITIONS, EVENT_UI_DEFINITIONS


class EventLog(ScrollableFrame):
    def __init__(self, config, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.init_ros()

        self._config = config

        self._event_log_items = []

        self.canvas.configure(width=570)
        self.canvas.configure(height=800)

    def init_ros(self):
        rospy.init_node("event_log_node")
        rospy.Subscriber("/strawberry/kb/event", Event, self.on_event)

    def on_event(self, msg):
        event = json.loads(msg.event.payload)
        event_definition = next(
            iter(x for x in EVENT_DEFINITIONS if x["id"] == event["definitionID"])
        )

        config = {
            "event": event,
            "event_definition": event_definition,
            "font_size": self._config["font_size"],
            "icon": self._config["icons"][
                EVENT_UI_DEFINITIONS[event_definition["id"]]["image"]
            ],
        }

        event_log_item = EventLogItem(config, master=self.scrollable_frame, width=500)
        event_log_item.pack(fill="both", expand=True, side="top", padx=10, pady=5)
        self.canvas.yview_moveto(1)

        self._event_log_items.append(event_log_item)


if __name__ == "__main__":

    from src.icon_db import IconDB

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("dark-blue")

    app = ctk.CTk()
    app.title("Event Log")
    app.geometry("600x810")

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../../res/icons")

    config = {
        "font_size": 24,
        "icons": icon_db.icons,
    }

    event_log = EventLog(config, master=app)
    event_log.pack()

    app.mainloop()
