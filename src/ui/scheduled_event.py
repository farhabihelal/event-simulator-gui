import logging
from random import random
import tkinter as tk
import tkinter.ttk as ttk

import customtkinter as ctk

from ui.base_event import BaseEventGUI


class ScheduledEvent(BaseEventGUI):
    def __init__(self, config={}) -> None:
        super().__init__(config)

        self.schedule = self.event["parameters"]["schedule"]

        self.running = False

        self.countdown_time = random.randint(100, 500) * 60

        self.frame = ctk.CTkFrame(master=self.config["ui"]["master"])
        row, col = self.config["ui"]["grid"]
        self.frame.grid(
            row=row,
            column=col,
            padx=5,
            pady=5,
        )

        self.label = ctk.CTkLabel(
            master=self.frame,
            anchor="center",
            text_font=("Roboto Medium", 12),
            justify="center",
            text=f"{self.ms_to_sec(self.countdown_time)}",
            corner_radius=8
            # width=10,
            # padding=5,
        )
        self.label.pack()

        self.button = ctk.CTkButton(
            master=self.frame,
            text=self.event["name"].replace("-", " ").title(),
            command=self.btn_handler,
            name=self.event["name"],
            # padding=(10, 0),
            image=self.config["ui"]["image"],
            compound=tk.TOP,
        )
        self.button.pack()

    def btn_handler(self):
        if self.running:
            logging.debug(f"Event cancelled : {self.button.config('text')[-1]}")
            self.button.config(text=self.event["name"].replace("-", " ").title())
            self.running = not self.running

        else:
            logging.debug(f"Event countdown started : {self.button.config('text')[-1]}")
            self.running = not self.running
            self.countdown(self.countdown_time)

    def countdown(self, remaining: int = 0, interval: int = 1000):
        if self.running:
            self.label.config(text=f"{self.ms_to_sec(remaining)}")

            if remaining <= 0:
                self.publish()
                self.running = False
                self.label.config(text=f"{self.ms_to_sec(self.countdown_time)}")
                return

            else:
                return self.label.after(
                    interval, lambda: self.countdown(remaining - interval, interval)
                )
        else:
            self.label.config(text=f"{self.ms_to_sec(self.countdown_time)}")

    def ms_to_sec(self, ms: int):
        return int(ms / 1000)


# class TimeEventGUI(ScheduledEvent):
#     def __init__(self, config={}, **kwargs) -> None:
#         super().__init__(config, **kwargs)


# class PeriodicEventGUI(ScheduledEvent):
#     def __init__(self, config={}, **kwargs) -> None:
#         super().__init__(config, **kwargs)

#     def countdown(self, remaining: int = 0, interval: int = 1000):
#         if self.running:
#             self.label.config(text=f"{self.ms_to_sec(remaining)}")

#             if remaining <= 0:
#                 self.publish()
#                 remaining = self.countdown_time

#                 return self.label.after(
#                     interval, lambda: self.countdown(remaining, interval)
#                 )
#             else:
#                 return self.label.after(
#                     interval, lambda: self.countdown(remaining - interval, interval)
#                 )
#         else:
#             self.label.config(text=f"{self.ms_to_sec(self.countdown_time)}")
