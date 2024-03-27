import tkinter as tk
import tkinter.ttk as ttk
import customtkinter as ctk


class BorderlessWindow(tk.Toplevel):
    def __init__(self, *args, **kwargs):
        tk.Toplevel.__init__(self, *args, **kwargs)
        self.overrideredirect(True)
        self.wm_geometry("400x400")

        self.label = tk.Label(self, text="Grab the lower-right corner to resize")
        self.label.pack(side="top", fill="both", expand=True)

        self.grip = ttk.Sizegrip(self)
        self.grip.place(relx=1.0, rely=1.0, anchor="se")
        self.grip.lift(self.label)
        self.grip.bind("<B1-Motion>", self.OnMotion)

    def OnMotion(self, event):
        x1 = self.winfo_pointerx()
        y1 = self.winfo_pointery()
        x0 = self.winfo_rootx()
        y0 = self.winfo_rooty()
        self.geometry("%sx%s" % ((x1 - x0), (y1 - y0)))
        return


if __name__ == "__main__":
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    app = BorderlessWindow()

    app.mainloop()
