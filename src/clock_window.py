import os
from icon_db import IconDB
from ui.virtual_clock_widget import VirtualClockWidget
from ui.window import Window

app = Window()
app.geometry("1975x400")


icon_db = IconDB(f"{os.path.dirname(__file__)}/../res/icons")

config = {
    "vclock": {
        "virtual_second": 60 / (4 * 60 * 60),
        "loop_rate": 200,
        "refresh_rate": 1000,
        "initial_time": "3022-01-01 05:00:00",
    },
    "icons": {
        "play": icon_db.icons["forward-fast-solid-32"],
        "stop": icon_db.icons["stop-solid-32"],
    },
    "font": ("Roboto Medium", 40),
}
vc = VirtualClockWidget(config, master=app)
vc.lbl_clock.configure(font=("Roboto Medium", 150))
vc.pack()

app.mainloop()
