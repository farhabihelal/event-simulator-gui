import tkinter as tk
import tkinter.ttk as ttk
import customtkinter as ctk

from window import Window
from virtual_clock_viewer_widget import VirtualClockViewerWidget


class ImageViewerWindow(Window):
    def __init__(self, *args, fg_color="default_theme", **kwargs):
        super().__init__(*args, fg_color=fg_color, **kwargs)

        self._images = {}

        self.lbl_image = ctk.CTkLabel(
            master=self,
            anchor="center",
            text_font=("Roboto Medium", 42),
            justify="center",
            text=f"external events".title(),
            corner_radius=8,
            image="",
        )

        self.lbl_image.pack()

        config = {
            "font": ("Roboto Medium", 100),
        }
        self.clock_viewer = VirtualClockViewerWidget(config)
        self.clock_viewer.pack()

    def load_images(self):
        pass


if __name__ == "__main__":

    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    window = ImageViewerWindow()
    window.mainloop()
