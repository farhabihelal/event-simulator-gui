import tkinter as tk
import tkinter.ttk as ttk

import customtkinter as ctk


from src.ui.base_event import BaseEventGUI


class ExternalEventGUI(BaseEventGUI):
    def __init__(self, config={}, *args, **kwargs) -> None:
        super().__init__(config, *args, **kwargs)

        self.button = ctk.CTkButton(
            master=self,
            text=self._event["name"].replace("-", " ").title(),
            command=self.btn_handler,
            name=self._event["name"],
            image=self._config["ui"]["image"],
            compound=tk.TOP,
            text_font=self._config["font"],
            width=250,
        )
        self.button.pack(
            padx=10,
            pady=10,
        )

    def btn_handler(self):
        self.publish()
