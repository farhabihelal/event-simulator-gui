from doctest import master
import os
import sys

sys.path.append(f"{os.path.dirname(__file__)}/../..")

import tkinter as tk
import tkinter.ttk as ttk

import customtkinter as ctk


class TableItem(ctk.CTkFrame):
    def __init__(self, config, row_idx, col_idx, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.row_idx = row_idx
        self.col_idx = col_idx

        self.entry = ctk.CTkEntry(
            master=self, placeholder_text=f"{self.row_idx, self.col_idx}"
        )
        self.entry.pack()


class Table(ctk.CTkFrame):
    def __init__(self, config, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._config = config
        self.dimension = self._config["dimension"]

        self._cells = []
        self._header_cells = []

        for j, header in enumerate(self._config["headers"]):
            cell = ctk.CTkLabel(
                master=self,
                text=header,
                text_font=("Roboto Medium", 12),
            )
            cell.grid(row=0, column=j, columnspan=self._config["colspans"][j])
            self._header_cells.append(cell)

        for i in range(1, self.dimension[0]):
            for j in range(self.dimension[1]):
                cell = self._config["cell_type"]({}, row_idx=i, col_idx=j, master=self)
                cell.grid(row=i, column=j)
                self._cells.append(cell)


if __name__ == "__main__":
    ctk.set_appearance_mode("dark")
    ctk.set_default_color_theme("blue")

    app = ctk.CTk()
    app.title("Virtual Clock")
    app.geometry("600x200")

    config = {
        "headers": ["H1", "H2", "H3"],
        "colspans": [1, 1, 1],
        "dimension": (15, 3),
        "cell_type": TableItem,
    }
    table = Table(config, master=app)
    table.pack(expand=True)
    app.mainloop()
