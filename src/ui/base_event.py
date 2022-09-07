import json
import logging
import random

import customtkinter as ctk


from strawberry_dialogue_knowrob.msg import Event, QueryType, OntologyType

import rospy


class BaseEventGUI(ctk.CTkFrame):
    def __init__(self, config={}, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self._config = config
        self._event = self._config["event"]

        self.enabled = True

        self.publisher = rospy.Publisher(
            "/strawberry/kb/event", Event, latch=False, queue_size=1
        )

    def publish(self):
        if self.enabled:
            event = Event()
            event.header = rospy.Header()
            event.event.payload = json.dumps(self.get_event())

            self.publisher.publish(event)

            logging.debug(f"Event published : {self._event['name']}")

        else:
            logging.debug(f"Publish Ignored : {self._event['name']} is disabled.")

    def get_event(self) -> dict:
        return {
            "id": self._event["id"],
            "parameters": self.get_event_parameters(),
        }

    def get_event_parameters(self):
        event_parameters = {}

        if self._event["name"] == "medicine-reminder":
            event_parameters = {
                "medicine_name": random.choice(
                    ["Vitamins", "Calcium Supplement", "Insulin", "Antibiotic"]
                ),
                "time": f"{random.randint(6, 23):02d}:{random.randint(0, 59):02d}",
                "user_id": "0",
            }

        elif self._event["name"] == "entering-home":
            event_parameters = {
                "user_id": "0",
            }

        elif self._event["name"] == "leaving-home":
            event_parameters = {
                "user_id": "0",
            }

        elif self._event["name"] == "rain-alert":
            event_parameters = {
                "items": random.choice(["umbrella", "rain coat"]),
                "time": f"{random.randint(0, 23) : 02d}:{random.randint(0, 59):02d}",
                "user_id": "0",
            }

        elif self._event["name"] == "noise_alert":
            event_parameters = {
                "source": random.choice(["garage", "kitchen", "basement", "main door"]),
                "user_id": "0",
            }

        elif self._event["name"] == "bill-payment":
            event_parameters = {
                "bill_type": random.choice(["electricity", "gas", "water", "internet"]),
                "billed_amount": f"{random.randint(50000, 1500000) / 100:0.2f}",
                "user_id": "0",
            }

        elif self._event["name"] == "health-checkup":
            event_parameters = {
                "hospital_name": f"{random.choice(['Tokyo', 'Saitama', 'Wako', 'Ikebukuro'])} General Hospital",
                "doctor_name": f"{random.choice(['Yamamoto', 'Takahashi', 'Tabata'])} {random.choice(['Ikeda', 'Yamaguchi', 'Minami'])}",
                "time": f"{random.randint(8, 17):02d}:{random.randint(0, 59):02d}",
                "user_id": "0",
            }

        elif self._event["name"] == "garbage-reminder":
            event_parameters = {
                "garbage_type": random.choice(
                    ["burnable", "pet bottles and can", "non-burnable", "heavy"]
                ),
                "user_id": "0",
            }

        elif self._event["name"] == "earthquake-alert":
            event_parameters = {
                "intensity": f"{random.randint(50, 99) / 10:0.2f}",
                "source": random.choice(["Fukuoka", "Chiba", "Fukushima", "Ibaraki"]),
                "instructions": random.choice(
                    [
                        "take shelter under heavy objects.",
                        "watch out for fire and falling objects.",
                    ]
                ),
                "user_id": "0",
            }

        elif self._event["name"] == "tsunami-alert":
            event_parameters = {
                "water_height": f"{random.randint(100, 1000) / 100:0.2f}",
                "instructions": random.choice(
                    [
                        "evacuate to nearest evacuation center immediately.",
                        "stay away from rivers and beaches.",
                    ]
                ),
                "user_id": "0",
            }

        elif self._event["name"] == "heat-alert":
            event_parameters = {
                "time": f"{random.randint(10, 16):02d}:{random.randint(0, 59):02d}",
                "items": random.choice(["water", "umbrella"]),
                "user_id": "0",
            }

        elif self._event["name"] == "breakfast-reminder":
            event_parameters = {
                "user_id": "0",
            }

        elif self._event["name"] == "dinner-reminder":
            event_parameters = {
                "user_id": "0",
            }

        return event_parameters
