from datetime import datetime
import os
import sys


sys.path.append(f"{os.path.dirname(__file__)}/../..")

import tkinter as tk
import tkinter.ttk as ttk

import customtkinter as ctk

from src.virtual_clock import VirtualClock
from src.icon_db import IconDB


class VirtualClockWidget(ctk.CTkFrame):
    def __init__(self, config, **kwargs) -> None:
        super().__init__(**kwargs)

        self._config = config
        self._icons = self._config["icons"]

        self.clock = VirtualClock(self._config["vclock"])

        self.callback = {
            "on_start": [],
            "on_update": [],
            "on_stop": [],
        }

        self.clock.callback["on_start"] = self.callback["on_start"]
        self.clock.callback["on_update"] = self.callback["on_update"]
        self.clock.callback["on_stop"] = self.callback["on_stop"]

        self.lbl_clock = ctk.CTkLabel(
            master=self,
            text=f"{self.clock}",
            text_font=self._config["font"],
            anchor="center",
            corner_radius=10,
        )

        self.lbl_clock.grid(row=0, column=0, columnspan=3, padx=5, pady=5)

        self.btn_start_stop = ctk.CTkButton(
            master=self,
            image=self._icons["play"],
            text="",
            command=self.on_button_click,
            width=50,
        )
        self.btn_start_stop.grid(row=0, column=4, padx=5, pady=5)

        self.update_time()

    def update_time(self):
        self.lbl_clock.configure(text=f"{self.clock}")

        self.lbl_clock.after(10, self.update_time)

    def on_button_click(self):
        if self.clock.running:
            self.btn_start_stop.configure(image=self._icons["play"])
            self.clock.stop()
        else:
            self.btn_start_stop.configure(image=self._icons["stop"])
            self.clock.start()


if __name__ == "__main__":

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.title("Virtual Clock")
    app.geometry("600x200")

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../../res/icons")

    config = {
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
    vc = VirtualClockWidget(config, master=app)
    vc.pack()

    vc.callback["on_start"].append(lambda: print("Clock started!"))
    vc.callback["on_update"].append(
        lambda x: print(f"{datetime.fromtimestamp(x).strftime('%Y-%m-%d %H:%M:%S')}")
    )
    vc.callback["on_stop"].append(lambda: print("Clock stopped!"))

    app.mainloop()
