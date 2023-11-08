import os
import sys

sys.path.append(f"{os.path.dirname(__file__)}/../..")

from threading import Thread
from time import sleep, time
from datetime import datetime
import logging

from src.icon_db import IconDB
from src.event_definition import EVENT_DEFINITIONS, EVENT_UI_DEFINITIONS
from src.schedule import Schedule

from src.ui.virtual_clock_widget import VirtualClockWidget
from src.ui.base_event import BaseEventGUI


import tkinter as tk
import tkinter.ttk as ttk

import customtkinter as ctk


class EventTableHeader(ctk.CTkFrame):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.lbl_state = ctk.CTkLabel(
            master=self,
            anchor="center",
            corner_radius=10,
            text="enabled".title(),
            text_font=("Roboto Medium", 10),
            width=100,
        )

        self.lbl_state.grid(
            row=0,
            column=0,
            # columnspan=1,
            padx=50,
            pady=5,
        )

        self.lbl_event_name = ctk.CTkLabel(
            master=self,
            anchor="center",
            corner_radius=10,
            text="event name".title(),
            text_font=("Roboto Medium", 10),
            width=200,
        )

        self.lbl_event_name.grid(
            row=0,
            column=1,
            # columnspan=3,
            padx=5,
            pady=5,
        )

        self.lbl_schedule = ctk.CTkLabel(
            master=self,
            anchor="center",
            corner_radius=10,
            text="scheduled at".title(),
            text_font=("Roboto Medium", 10),
            width=230,
        )

        self.lbl_schedule.grid(
            row=0,
            column=4,
            # columnspan=3,
            sticky="e",
            padx=5,
            pady=5,
        )


class ScheduledEventTableItem(BaseEventGUI):
    def __init__(self, config, *args, **kwargs):
        self.row_idx = kwargs.pop("row_idx", 0)
        self.col_idx = kwargs.pop("col_idx", 0)

        super().__init__(config, *args, **kwargs)

        self._current_time = time()

        self._clock: VirtualClockWidget = self._config["clock"]

        self._clock.callback["on_start"].append(self.on_clock_start)
        self._clock.callback["on_update"].append(self.on_clock_update)
        self._clock.callback["on_stop"].append(self.on_clock_stop)

        self._schedule = Schedule(
            [
                x["hasValue"]
                for x in self._config["event"]["hasEventParameter"]
                if x["hasKey"] == "schedule"
            ][0]
        )

        self.cb_state = ctk.CTkCheckBox(
            master=self,
            corner_radius=10,
            command=self.on_cb_clicked,
            offvalue=0,
            onvalue=1,
            text="",
        )

        self.cb_state.grid(
            row=0,
            column=0,
            padx=50,
            pady=5,
        )
        self.cb_state.select()

        self.lbl_event_name = ctk.CTkLabel(
            master=self,
            anchor="w",
            corner_radius=10,
            text=f"{self.event_name}",
            font=("Roboto Medium", 10),
            # width=200,
            image=self._config["icon"],
            compound="top",
        )

        self.lbl_event_name.grid(
            row=0,
            column=1,
            columnspan=3,
            padx=5,
            pady=10,
        )

        self.lbl_schedule = ctk.CTkLabel(
            master=self,
            anchor="w",
            corner_radius=10,
            text=f"{self._schedule}".title(),
            font=("Roboto Medium", 12),
            width=200,
        )

        self.lbl_schedule.grid(
            row=0,
            column=4,
            columnspan=3,
            padx=5,
            pady=0,
        )

        self.cb_state.after(10, self.handle_clock_update)

    def on_cb_clicked(self):
        self.enabled = True if self.cb_state.get() == 1 else False

    def on_clock_update(self, time):
        self._current_time = time

    def on_clock_start(self):
        return

    def on_clock_stop(self):
        return

    def handle_clock_update(self):
        if self.enabled:
            _datetime = datetime.fromtimestamp(self._current_time)
            _time = _datetime.time()

            if self._schedule.daily:
                if (
                    _time.hour == self._schedule._datetime.hour
                    and _time.minute == self._schedule._datetime.minute
                ):
                    self.publish()
                    self.cb_state.after(500, self.reset)
            else:
                if (
                    _datetime.date() == self._schedule._datetime.date()
                    and _time.hour == self._schedule._datetime.hour
                    and _time.minute == self._schedule._datetime.minute
                ):
                    self.publish()

        self.cb_state.after(10, self.handle_clock_update)

    @property
    def event_name(self) -> str:
        return self._event["name"].replace("-", " ").title()

    def publish(self):
        super().publish()

        self.enabled = False

        print(f"{self.event_name} published.")

        self.cb_state.deselect()
        self.cb_state.configure(state=tk.DISABLED)

        self.configure(bg_color="green")
        self.lbl_event_name.configure(bg_color="green")
        self.lbl_schedule.configure(text_color="green")

    def reset(self):
        self.enabled = True
        self.cb_state.select()
        self.cb_state.configure(state=tk.NORMAL)

        self.configure(bg_color="transparent")
        self.lbl_event_name.configure(bg_color="transparent")
        self.lbl_schedule.configure(text_color="white")

    def __del__(self):
        self.enabled = False


class ScheduledEventTable(ctk.CTkFrame):
    def __init__(self, config, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._config = config

        self._events = self._config["events"]
        self.dimension = (len(self._events), 1)

        self._clock = self._config["clock"]

        self._cells = []
        # self._header = EventTableHeader(master=self)
        # self._header.grid(row=0)

        for i in range(self.dimension[0]):
            for j in range(self.dimension[1]):
                config = {
                    "event": self._events[i],
                    "icon": self._config["icons"][
                        EVENT_UI_DEFINITIONS[self._events[i]["id"]]["image"]
                    ],
                    "clock": self._clock,
                }
                cell = ScheduledEventTableItem(
                    config, row_idx=i, col_idx=j, master=self
                )
                cell.grid(
                    row=i + 1,
                    column=j,
                    padx=5,
                    pady=10,
                )
                self._cells.append(cell)


if __name__ == "__main__":
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.title("Event Table")
    app.geometry("600x200")

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../../res/icons")

    vclock_config = {
        "vclock": {
            "virtual_second": 1 / 7200,
            "loop_rate": 1000,
        },
        "icons": {
            "play": icon_db.icons["forward-fast-solid-32"],
            "stop": icon_db.icons["stop-solid-32"],
        },
        "font": ("Roboto Medium", 40),
    }
    vclock = VirtualClockWidget(vclock_config, master=app)
    vclock.pack()

    config = {
        "events": [x for x in EVENT_DEFINITIONS if x["type"] in ["scheduled"]],
        "icons": icon_db.icons,
        "clock": vclock,
    }
    table = ScheduledEventTable(config, master=app)
    table.pack(expand=True, fill=tk.BOTH)
    app.mainloop()
