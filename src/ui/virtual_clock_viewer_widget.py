from datetime import datetime
import os
import sys

sys.path.append(f"{os.path.dirname(__file__)}/../..")

import tkinter as tk
import tkinter.ttk as ttk
import customtkinter as ctk

from src.icon_db import IconDB

import rospy
from rosgraph_msgs.msg import Clock


class VirtualClockViewerWidget(ctk.CTkFrame):
    def __init__(self, config, **kwargs) -> None:
        super().__init__(**kwargs)

        self._config = config
        self.clock = None

        self.init_ros()

        self.lbl_clock = ctk.CTkLabel(
            master=self,
            text=f"{self.clock}",
            text_font=self._config["font"],
            anchor="center",
            corner_radius=10,
        )

        self.lbl_clock.grid(row=0, column=0, columnspan=3, padx=5, pady=5)

        self.update_time()

    def init_ros(self):
        rospy.init_node("virtual_clock_viewer")

        rospy.Subscriber("/clock", Clock, self.on_clock)

    def on_clock(self, msg):
        self.clock = datetime.fromtimestamp(msg.clock.secs)

    def update_time(self):
        if self.clock:
            self.lbl_clock.configure(text=f"DAY {self.clock.strftime('%d %H:%M:%S')}")

        self.lbl_clock.after(10, self.update_time)


if __name__ == "__main__":

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.title("Virtual Clock Viewer")
    app.geometry("1540x250")

    icon_db = IconDB(f"{os.path.dirname(__file__)}/../../res/icons")

    config = {
        "font": ("Roboto Medium", 150),
    }
    vc = VirtualClockViewerWidget(config, master=app)
    vc.pack()

    app.mainloop()
