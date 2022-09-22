import rospy

from datetime import datetime
import threading
from time import sleep, time

from std_msgs.msg import Bool
from rosgraph_msgs.msg import Clock


class VirtualClock:
    def __init__(self, config) -> None:
        super().__init__()

        self.config = config
        self._sleep_time = 1 / self.config.get("refresh_rate", 100.0)
        self._virtual_second = self.config.get("virtual_second", 60 / (8 * 60 * 60))
        self.loop_rate = 1 / self.config.get("loop_rate", 200)

        self._initial_time = (
            time()
            if not self.config.get("initial_time")
            else datetime.strptime(
                self.config.get("initial_time"),
                # f"{datetime.now().strftime('%Y-%m-%d')} {self.config.get('initial_time', '00:00:00')}",
                "%Y-%m-%d %H:%M:%S",
            ).timestamp()
        )
        self._time = self._initial_time

        self._launch_time = time()
        self._running = False

        self._th_update = None
        self._th_update_checker = None

        self.callback = {
            "on_start": [],
            "on_update": [],
            "on_stop": [],
        }

        self._clock_lock = threading.Lock()

        self._updated_flag_lock = threading.Lock()
        self._updated_flag = False

        self._clock_publisher = rospy.Publisher(
            "/clock", Clock, latch=False, queue_size=10
        )

        self._clock_status_publisher = rospy.Publisher(
            "/vclock/status", Bool, latch=True, queue_size=1
        )

        self._pub_th = threading.Thread(target=self.publish)
        self._pub_th.start()

    def start(self):
        # self._time = time()
        self.running = True

        self.run_callbacks(self.callback["on_start"])

        self._th_update_checker = threading.Thread(
            name="Update Callback Thread", target=self.check_update
        )
        self._th_update_checker.start()

        self._th_update = threading.Thread(name="Update Thread", target=self.run)
        self._th_update.start()

    def stop(self):
        if not self.running:
            return

        self.running = False

        if self._th_update:
            self._th_update.join()
            self._th_update = None

        if self._th_update_checker:
            self._th_update_checker.join()
            self._th_update_checker = None

        self.run_callbacks(self.callback["on_stop"])

    def reset(self):
        self.stop()
        self._time = self._initial_time

    def run(self):
        while self.running:
            sleep(self.loop_rate)
            self._time += self.loop_rate * (1 / self.virtual_second)
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

    def publish(self):
        while not rospy.is_shutdown():
            msg = Clock()
            msg.clock.secs = self.time
            self._clock_publisher.publish(msg)
            sleep(self.virtual_second)

    def publish_status(self):
        msg = Bool()
        msg.data = self.running
        self._clock_status_publisher.publish(msg)

    @property
    def running(self):
        return self._running

    @running.setter
    def running(self, running):
        self._running = running
        self.publish_status()

    @property
    def initial_time(self):
        return self._initial_time

    @initial_time.setter
    def initial_time(self, datetime_str):
        with self._clock_lock:
            self._initial_time = datetime.strptime(
                datetime_str, "%Y-%m-%d %H:%M:%S"
            ).timestamp()

    @property
    def virtual_second(self):
        return self._virtual_second

    @virtual_second.setter
    def virtual_second(self, value):
        with self._clock_lock:
            self._virtual_second = value

    @property
    def time(self) -> int:
        return int(self._time)
        # if self.running else self._time + (time() - self._launch_time)

    @time.setter
    def time(self, datetime_str):
        with self._clock_lock:
            self._time = datetime.strptime(
                datetime_str, "%Y-%m-%d %H:%M:%S"
            ).timestamp()

    def __repr__(self) -> str:
        return datetime.fromtimestamp(self.time).strftime("%Y-%m-%d %H:%M:%S")

    def display_time(self) -> str:
        return datetime.fromtimestamp(self.time).strftime("DAY %d %H:%M:%S")

    def __del__(self):
        self.stop()


if __name__ == "__main__":

    config = {
        "loop_rate": 1000.0,
        "virtual_second": 60 / (8 * 60 * 60),  # 1 real minute == 4 sim hours
        "initial_time": "2100-10-10 05:00:00",
        "refresh_rate": 200,
    }
    vc = VirtualClock(config)

    vc.callback["on_start"].append(lambda: print("Clock started!"))
    vc.callback["on_update"].append(
        lambda x: print(f"{datetime.fromtimestamp(x).strftime('%Y-%m-%d %H:%M:%S')}")
    )
    vc.callback["on_stop"].append(lambda: print("Clock stopped!"))

    vc.start()

    sleep(60)

    vc.stop()
