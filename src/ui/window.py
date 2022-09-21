import tkinter as tk
import tkinter.ttk as ttk
import customtkinter as ctk


class Window(ctk.CTk):
    def __init__(self, *args, fg_color="default_theme", **kwargs):
        super().__init__(*args, fg_color=fg_color, **kwargs)

        self._fullscreen_state = False

        self.bind("<F11>", self.toggle_fullscreen)
        self.bind("<Escape>", self.end_fullscreen)

    def toggle_fullscreen(self, event=None):
        self._fullscreen_state = not self._fullscreen_state  # Just toggling the boolean
        self.attributes("-fullscreen", self._fullscreen_state)
        # return "break"

    def end_fullscreen(self, event=None):
        self._fullscreen_state = False
        self.attributes("-fullscreen", self._fullscreen_state)
        # return "break"


if __name__ == "__main__":
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    app = Window()

    app.mainloop()
