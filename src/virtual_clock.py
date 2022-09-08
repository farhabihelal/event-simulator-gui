import rospy

from datetime import datetime
import threading
from time import sleep, time

# from rosgraph_msgs import Clock


class VirtualClock:
    def __init__(self, config) -> None:
        super().__init__()

        self.config = config
        self._sleep_time = 1 / self.config.get("loop_rate", 100.0)
        self._time = time()
        self.running = False

        self._th_update = None
        self._th_update_checker = None

        self.callback = {
            "on_start": [],
            "on_update": [],
            "on_stop": [],
        }

        self._updated_flag_lock = threading.Lock()
        self._updated_flag = False

        # self._clock_publisher = rospy.Publisher(
        #     "/clock", Clock, latch=False, queue_size=1
        # )

    def start(self):
        self._time = time()
        self.running = True

        self.run_callbacks(self.callback["on_start"])

        self._th_update_checker = threading.Thread(
            name="Update Callback Thread", target=self.check_update
        )
        self._th_update_checker.start()

        self._th_update = threading.Thread(name="Update Thread", target=self.run)
        self._th_update.start()

    def stop(self):
        self.running = False

        if self._th_update:
            self._th_update.join()
            self._th_update = None

        if self._th_update_checker:
            self._th_update_checker.join()
            self._th_update_checker = None

        self.run_callbacks(self.callback["on_stop"])

    def run(self):
        while self.running:
            sleep(self.config["virtual_second"])
            self._time += 1
            with self._updated_flag_lock:
                self._updated_flag = True

    def run_callbacks(self, callbacks, args=[]):
        for x in callbacks:
            th = threading.Thread(target=x, args=args)
            th.start()
            th.join()

    def check_update(self):
        while self.running:
            if self._updated_flag:
                self.run_callbacks(self.callback["on_update"], args=[int(self.time)])

                with self._updated_flag_lock:
                    self._updated_flag = False
            sleep(self._sleep_time)

    @property
    def time(self) -> int:
        return self._time if self.running else int(time())

    def __repr__(self) -> str:
        return datetime.fromtimestamp(self.time).strftime("%Y-%m-%d %H:%M:%S")

    def __del__(self):
        self.stop()


if __name__ == "__main__":

    config = {
        "loop_rate": 1000.0,
        "virtual_second": 1 / 3600,
    }
    vc = VirtualClock(config)

    vc.callback["on_start"].append(lambda: print("Clock started!"))
    vc.callback["on_update"].append(
        lambda x: print(f"{datetime.fromtimestamp(x).strftime('%Y-%m-%d %H:%M:%S')}")
    )
    vc.callback["on_stop"].append(lambda: print("Clock stopped!"))

    vc.start()

    sleep(5)

    vc.stop()
