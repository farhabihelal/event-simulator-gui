import rospy

from datetime import datetime
import threading
from time import sleep, time

from event_simulator_gui.srv import (
    ClockCommand,
    ClockCommandRequest,
    ClockCommandResponse,
)
from rosgraph_msgs.msg import Clock


class VirtualClockServer:
    def __init__(self, config) -> None:
        super().__init__()

        self.config = config

        self.init_ros()

        self._virtual_second = self.config.get("virtual_second", 60 / (8 * 60 * 60))
        self._sleep_duration = 1 / self.config.get("loop_rate", 200)

        self._initial_time = (
            datetime.now()
            if not self.config.get("initial_time")
            else datetime.strptime(
                self.config.get("initial_time"),
                "%Y-%m-%d %H:%M:%S",
            )
        )

        self._datetime = self._initial_time
        self._time = self._datetime.timestamp()

        self._speed_multipliers = {
            "normal": 1,
            "fast": 2,
            "faster": 4,
            "fastest": 8,
            "insane": 16,
        }

        self._launch_time = time()
        self.running = False

        self._clock_publisher = rospy.Publisher(
            "/clock", Clock, latch=False, queue_size=10
        )

        self._th = None

        self._pub_th = threading.Thread(target=self.publish)
        self._pub_th.start()

    def init_ros(self):
        rospy.init_node("virtual_clock")
        rospy.Service("/vclock/cmd", ClockCommand, self.handle_clock_command)

    def start(self):
        self.running = True

        self._th = threading.Thread(target=self.update)
        self._th.start()

    def stop(self):
        if not self.running:
            return
        self.running = False

    def set_speed(self, multiplier=1):
        self.virtual_second = 60 / (multiplier * 4 * 60 * 60)

    def set_time(self, datetime_str):
        try:
            self._datetime = datetime.strptime(datetime_str, "%Y-%m-%d %H:%M:%S")
            self._time = self._datetime.timestamp()

        except Exception as e:
            rospy.logerr(e)

    def set_initial_time(self, datetime_str):
        try:
            self._initial_time = datetime.strptime(datetime_str, "%Y-%m-%d %H:%M:%S")

        except Exception as e:
            rospy.logerr(e)

    def reset(self):
        self.stop()
        self._time = self._initial_time.timestamp()

    def update(self):
        while self.running:
            sleep(self._sleep_duration)
            self._time += self._sleep_duration * (1 / self.virtual_second)

    def run(self):
        rospy.spin()

    def publish(self):
        while not rospy.is_shutdown():
            msg = Clock()
            msg.clock.secs = self.time
            self._clock_publisher.publish(msg)
            sleep(self._sleep_duration)

    def handle_clock_command(self, request):

        command = request.command.lower()
        parameter = request.parameter.lower()

        if command == "start":
            self.start()
        elif command == "stop":
            self.stop()
        elif command == "reset":
            self.reset()
        elif command == "set_time":
            self.set_time(parameter)
        elif command == "set_initial_time":
            self.set_initial_time(parameter)
        elif command == "set_speed":
            if parameter not in self._speed_multipliers.keys():
                rospy.logwarn(
                    f"Invalid parameter value={parameter}. Valid options are: {self._speed_multipliers.keys()}"
                )
                return

            multiplier = self._speed_multipliers.get(parameter, 1)
            self.set_speed(multiplier)

        return ClockCommandResponse()

    @property
    def virtual_second(self):
        return self._virtual_second

    @virtual_second.setter
    def virtual_second(self, value):
        self._virtual_second = value

    @property
    def time(self) -> int:
        return int(self._time)

    def from_datetime(self, datetime_str):
        self._datetime = datetime.strptime(datetime_str, "%Y-%m-%d %H:%M:%S")
        self._time = self._datetime.timestamp()

    def __repr__(self) -> str:
        return datetime.fromtimestamp(self.time).strftime("%Y-%m-%d %H:%M:%S")

    def __del__(self):
        self.stop()


if __name__ == "__main__":

    config = {
        "loop_rate": 1000.0,
        "virtual_second": 60 / (4 * 60 * 60),  # 1 real minute == 4 sim hours
        "initial_time": "2022-10-10 05:00:00",
    }
    vc = VirtualClockServer(config)
    vc.run()

    # vc.start()

    # sleep(60)

    # vc.stop()
