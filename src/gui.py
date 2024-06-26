import os

from pydispatch import dispatcher
import tkinter as tk
import tkinter.ttk as ttk

import customtkinter as ctk

import rospy

import logging

from icon_db import IconDB

from ui.external_event_widget import (
    ExternalEventWidget,
)
from ui.scheduled_event_widget import (
    ScheduledEventWidget,
)

# from strawberry_dialogue_knowrob.msg import Event, QueryType, OntologyType
# from strawberry_dialogue_knowrob.srv import Query, QueryRequest, QueryResponse

from event_simulator_gui.srv import (
    ClockCommand,
    ClockCommandRequest,
    ClockCommandResponse,
)

from event_definition import EVENT_DEFINITIONS, EVENT_TYPES, EVENT_UI_DEFINITIONS


logging.basicConfig(
    format="%(asctime)s | %(levelname)8s | %(message)s", level=logging.DEBUG
)


class EventSimulatorGUI(ctk.CTkFrame):
    def __init__(self, config=None, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._config = config
        self._events = self._config["events"]

        self.lbl_title = None

        # self.kb_client = rospy.ServiceProxy("/strawberry/kb/query", Query)
        self.init_ros()

        self.lbl_title = ctk.CTkLabel(
            master=self,
            name="lbl_title",
            text="event simulator".upper(),
            text_font=("Roboto Medium", 42),
            anchor="center",
            corner_radius=8,
        )

        self.lbl_title.pack()

        self.notebook = ttk.Notebook(master=self)
        self.notebook.pack(expand=True, fill="both")

        external_event_config = {
            "events": [x for x in self._events if x["eventType"] == "external"],
            "ui_confs": EVENT_UI_DEFINITIONS,
            "icons": self._config["icons"],
        }

        widget_external_event = ExternalEventWidget(
            external_event_config, master=self.notebook
        )
        widget_external_event.pack()

        scheduled_event_config = {
            "table": {
                "events": [
                    x for x in EVENT_DEFINITIONS if x["eventType"] == "periodic"
                ],
                "icons": self._config["icons"],
            },
            "clock": {
                "vclock": {
                    "virtual_second": 60 / (4 * 60 * 60),
                    "loop_rate": 200,
                    "refresh_rate": 1000,
                    "initial_time": "2122-09-09 05:00:00",
                },
                "icons": {
                    "play": icon_db.icons["forward-fast-solid-32"],
                    "stop": icon_db.icons["stop-solid-32"],
                },
                "font": ("Roboto Medium", 40),
            },
        }
        widget_scheduled_event = ScheduledEventWidget(
            scheduled_event_config, master=self.notebook
        )
        widget_scheduled_event.pack()

        self.notebook.add(
            widget_external_event,
            text="external events".title(),
        )
        self.notebook.add(
            widget_scheduled_event,
            text="scheduled events".title(),
        )

    def handle_clock_command(self, request):

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

    def init_ros(self):

        rospy.init_node("event_simulator_gui")
        rospy.Service("/vclock/cmd", ClockCommand, self.handle_clock_command)


if __name__ == "__main__":

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("dark-blue")

    app = ctk.CTk()
    app.title("event simulator".title())
    app.geometry("820x800")
    app.resizable(False, False)

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../res/icons")
    config = {
        "events": EVENT_DEFINITIONS,
        "icons": icon_db.icons,
    }

    gui = EventSimulatorGUI(config, master=app)
    gui.pack()

    app.mainloop()
