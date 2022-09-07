import os

import tkinter as tk


class IconDB:
    def __init__(self, icons_path) -> None:

        self.icons_path = icons_path

        self.icons = {}

        self.load_icons(icons_path)

    def load_icons(self, icons_path=None):
        files = os.listdir(self.icons_path)

        for file in files:
            name, ext = os.path.splitext(file)

            if ext.lower() == ".png":
                img = tk.PhotoImage(
                    file=f"{os.path.join(self.icons_path, file)}",
                    name=name,
                )
                self.icons[name.lower()] = img


if __name__ == "__main__":

    app = tk.Tk()

    db = IconDB(f"{os.path.dirname(__file__)}/../res/icons")
    print(f"Icons found: {len(db.icons.keys())}")
