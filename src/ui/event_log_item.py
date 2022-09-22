import rospy
from datetime import datetime
import os
import sys
import tkinter as tk
import tkinter.ttk as ttk
import customtkinter as ctk


sys.path.append(f"{os.path.dirname(__file__)}/../..")

from src.event_definition import EVENT_DEFINITIONS, EVENT_UI_DEFINITIONS
from src.icon_db import IconDB


class EventLogItem(ctk.CTkFrame):
    def __init__(self, config, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._config = config
        self._event = self._config["event"]
        self._event_definition = self._config["event_definition"]

        self.lbl_event = ctk.CTkLabel(
            master=self,
            anchor="center",
            text_font=("Roboto Medium", int(self._config["font_size"] / 1.5)),
            justify="center",
            text=self._event_definition["name"].replace("-", " ").title(),
            corner_radius=8,
            image=self._config["icon"],
            compound="top",
        )

        self.lbl_event.grid(row=0, column=0, padx=10, pady=10)

        self.lbl_time = ctk.CTkLabel(
            master=self,
            anchor="center",
            text_font=("Roboto Medium", self._config["font_size"]),
            justify="center",
            text=f"{datetime.fromtimestamp(rospy.get_time()).strftime('%Y-%m-%d %H:%M:%S')}",
            corner_radius=8,
        )
        self.lbl_time.grid(row=0, column=1, padx=10)

    def init_ros(self):
        pass


if __name__ == "__main__":

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("dark-blue")

    rospy.init_node("event_item_node")

    app = ctk.CTk()
    app.title("Event Log Item")
    app.geometry("300x200")

    event = {
        "id": "f1f0e932e46e46fc8526e3b65319b701",
        "definitionID": "2",
        "hasTimeInterval": {
            "hasIntervalDate": "2022-09-22",
            "hasIntervalTime": "15:16:24",
        },
        "hasEventParameter": [
            {
                "id": "15c805fb54904ea2a9d634b8413bdd34",
                "hasKey": "user_id",
                "hasValue": "0",
            }
        ],
    }

    event_definition = next(
        iter(x for x in EVENT_DEFINITIONS if x["id"] == event["definitionID"])
    )

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../../res/icons")

    config = {
        "event": event,
        "event_definition": event_definition,
        "font_size": 24,
        "icon": icon_db.icons[EVENT_UI_DEFINITIONS[event_definition["id"]]["image"]],
    }

    item = EventLogItem(config, master=app)
    item.pack()

    app.mainloop()
