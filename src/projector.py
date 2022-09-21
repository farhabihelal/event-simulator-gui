from datetime import datetime
import os

from idmind_tabletop_msgs.msg import ProjectorCommand

from event_simulator_gui.srv import (
    ClockCommand,
    ClockCommandRequest,
    ClockCommandResponse,
)
from rosgraph_msgs.msg import Clock

import rospy


class EventDemoProjectorContronller:
    def __init__(self, config) -> None:

        self._config = config

        self._image_files = {}

        self.hours = {
            "morning": (5, 7),
            "noon": (8, 16),
            "evening": (17, 19),
            "night": (20, 4),
        }

        self._current_image = None
        self._current_speed = "normal"

        self._day_banner_lock = False

        self.load_image_files()

        self.init_ros()

    def init_ros(self):
        rospy.init_node("projector_controller")

        self._projector_pub = rospy.Publisher(
            "/idmind_tabletop/cmd_projector",
            ProjectorCommand,
            latch=False,
            queue_size=10,
        )

        self._clock_cmd_client = rospy.ServiceProxy("/vclock/cmd", ClockCommand)

        rospy.Subscriber("/clock", Clock, self.monitor_time, queue_size=1)
        rospy.Subscriber("/clock", Clock, self.monitor_clock_speed, queue_size=1)
        rospy.Subscriber("/clock", Clock, self.monitor_day, queue_size=1)

    def load_image_files(self, path=None):
        files = os.listdir(self._config["images_path"])

        for file in files:
            name, ext = os.path.splitext(file)

            if ext.lower() == ".png":
                self._image_files[name.lower()] = os.path.join(
                    self._config["images_path"], file
                )

    def monitor_time(self, msg):
        _datetime = datetime.fromtimestamp(msg.clock.secs)

        hour = _datetime.hour

        for key in self.hours:
            start, end = self.hours[key]

            if hour >= start and hour <= end:
                if not self._day_banner_lock:
                    return self.update_projector(self._image_files[key])

        if not self._day_banner_lock:
            return self.update_projector(self._image_files["night"])

    def monitor_clock_speed(self, msg):
        _datetime = datetime.fromtimestamp(msg.clock.secs)

        hour = _datetime.hour

        speed = "normal"

        if hour >= 0 and hour <= 5:
            speed = "fastest"

        if speed != self._current_speed:
            self.command_clock(command="set_speed", parameter=speed)
            self._current_speed = speed

    def monitor_day(self, msg):
        _datetime = datetime.fromtimestamp(msg.clock.secs)

        hour = _datetime.hour
        day = _datetime.day

        if not self._day_banner_lock and hour >= 0 and hour <= 2:
            self._day_banner_lock = True
            self.update_projector(self._image_files.get(f"day{day}_banner_v2"))
        else:
            self._day_banner_lock = False

    def command_clock(self, command="reset", parameter=""):

        req = ClockCommandRequest()

        req.command = command
        req.parameter = parameter

        self._clock_cmd_client(req)

    def update_projector(self, image_file):

        if not image_file or image_file == self._current_image:
            return

        msg = ProjectorCommand()
        msg.projector_file = image_file
        msg.loop = True

        self._projector_pub.publish(msg)

        self._current_image = image_file

    def run(self):
        rospy.spin()


if __name__ == "__main__":

    config = {
        "images_path": f"{os.path.dirname(__file__)}/../res/images",
    }

    controller = EventDemoProjectorContronller(config)
    controller.run()
