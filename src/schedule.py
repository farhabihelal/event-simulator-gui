import datetime


class Schedule:
    def __init__(self, schedule) -> None:

        self.seconds = schedule[-2:]
        self.minutes = schedule[-5:-3]
        self.hours = schedule[-8:-6]
        self.days = schedule[-11:-9] if schedule[-10] != "*" else schedule[-10]
        self.months = schedule[-14:-12] if schedule[-12] != "*" else schedule[-12]

        self.years = schedule[2:6] if schedule[2] != "*" else schedule[2]
        self.dow = schedule[:1]

        self._datetime = (
            datetime.time(int(self.hours), int(self.minutes), int(self.seconds))
            if self.daily
            else datetime.datetime(
                int(self.years),
                int(self.months),
                int(self.days),
                int(self.hours),
                int(self.minutes),
                int(self.seconds),
            )
        )

    @property
    def daily(self):
        return self.days == "*" and self.months == "*" and self.years == "*"

    @property
    def one_time(self):
        return not self.daily

    def __repr__(self) -> str:
        return (
            f"daily {self._datetime.strftime('%H:%M')}"
            if self.daily
            else f"{self._datetime.strftime('%Y-%m-%d %H:%M')}"
        )


if __name__ == "__main__":
    schedule = [
        "* 0000-00-00 00:00:00",
        "* *-*-* 00:00:00",
        # "Mon *-*-* 00:00:00",
        # "* *-01-01 00:00:00",
    ]

    for s in schedule:
        sched = Schedule(s)
        print(sched)
