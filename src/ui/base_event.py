from datetime import datetime
import json
import logging
import random
from uuid import uuid4

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
            self._event["hasEventSource"]["topic"], Event, latch=False, queue_size=1
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
        now = datetime.now()

        return {
            "id": uuid4().hex,
            "definitionId": self._event["id"],
            "hasTimeInterval": {
                "hasIntervalDate": f"{now.strftime('%Y-%m-%d')}",
                "hasIntervalTime": f"{now.strftime('%H-%M-%S')}",
            },
            # "hasActivity": self.get_event_activity(),
            "hasEventParameter": self.get_event_parameters(),
        }

    def get_event_activity(self):
        event_activity = {}

        if self._event["name"] == "entering-home":
            event_activity = {
                "id": uuid4().hex,
                "name": "entering-home",
            }

        elif self._event["name"] == "leaving-home":
            event_activity = {
                "id": uuid4().hex,
                "name": "leaving-home",
            }

        elif self._event["name"] == "breakfast-reminder":
            event_activity = {
                # "id": uuid4().hex,
                # "name": "breakfast",
                # "hasMeal": {
                #     "id": uuid4().hex,
                #     "hasFood": [
                #         {
                #             "id": uuid4().hex,
                #             "name": "Cereal",
                #         },
                #         {
                #             "id": uuid4().hex,
                #             "name": "Toast",
                #         },
                #     ],
                # },
            }

        elif self._event["name"] == "lunch-reminder":
            event_activity = {
                # "id": uuid4().hex,
                # "name": "lunch",
                # "hasMeal": {
                #     "id": uuid4().hex,
                #     "hasFood": [
                #         {
                #             "id": uuid4().hex,
                #             "name": "Burger",
                #         },
                #         {
                #             "id": uuid4().hex,
                #             "name": "Coke",
                #         },
                #     ],
                # },
            }

        elif self._event["name"] == "dinner-reminder":
            event_activity = {
                # "id": uuid4().hex,
                # "name": "dinner",
                # "hasMeal": {
                #     "id": uuid4().hex,
                #     "hasFood": [
                #         {
                #             "id": uuid4().hex,
                #             "name": "Fried Chicken",
                #         },
                #         {
                #             "id": uuid4().hex,
                #             "name": "Salad",
                #         },
                #         {
                #             "id": uuid4().hex,
                #             "name": "Rice",
                #         },
                #     ],
                # },
            }

        return event_activity

    def get_event_parameters(self) -> list:
        event_parameters = {
            "user_id": "0",
        }

        # if self._event["name"] == "medicine-reminder":
        #     event_parameters = {
        #         "medicine_name": random.choice(
        #             ["Vitamins", "Calcium Supplement", "Insulin", "Antibiotic"]
        #         ),
        #         "time": f"{random.randint(6, 23):02d}:{random.randint(0, 59):02d}",
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "entering-home":
        #     event_parameters = {
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "leaving-home":
        #     event_parameters = {
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "rain-alert":
        #     event_parameters = {
        #         "items": random.choice(["umbrella", "rain coat"]),
        #         "time": f"{random.randint(0, 23) : 02d}:{random.randint(0, 59):02d}",
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "noise_alert":
        #     event_parameters = {
        #         "source": random.choice(["garage", "kitchen", "basement", "main door"]),
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "bill-payment":
        #     event_parameters = {
        #         "bill_type": random.choice(["electricity", "gas", "water", "internet"]),
        #         "billed_amount": f"{random.randint(50000, 1500000) / 100:0.2f}",
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "health-checkup":
        #     event_parameters = {
        #         "hospital_name": f"{random.choice(['Tokyo', 'Saitama', 'Wako', 'Ikebukuro'])} General Hospital",
        #         "doctor_name": f"{random.choice(['Yamamoto', 'Takahashi', 'Tabata'])} {random.choice(['Ikeda', 'Yamaguchi', 'Minami'])}",
        #         "time": f"{random.randint(8, 17):02d}:{random.randint(0, 59):02d}",
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "garbage-reminder":
        #     event_parameters = {
        #         "garbage_type": random.choice(
        #             ["burnable", "pet bottles and can", "non-burnable", "heavy"]
        #         ),
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "earthquake-alert":
        #     event_parameters = {
        #         "intensity": f"{random.randint(50, 99) / 10:0.2f}",
        #         "source": random.choice(["Fukuoka", "Chiba", "Fukushima", "Ibaraki"]),
        #         "instructions": random.choice(
        #             [
        #                 "take shelter under heavy objects.",
        #                 "watch out for fire and falling objects.",
        #             ]
        #         ),
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "tsunami-alert":
        #     event_parameters = {
        #         "water_height": f"{random.randint(100, 1000) / 100:0.2f}",
        #         "instructions": random.choice(
        #             [
        #                 "evacuate to nearest evacuation center immediately.",
        #                 "stay away from rivers and beaches.",
        #             ]
        #         ),
        #         "user_id": "0",
        #     }

        # elif self._event["name"] == "heat-alert":
        #     event_parameters = {
        #         "time": f"{random.randint(10, 16):02d}:{random.randint(0, 59):02d}",
        #         "items": random.choice(["water", "umbrella"]),
        #         "user_id": "0",
        #     }

        if self._event["name"] == "breakfast-reminder":
            pass

        elif self._event["name"] == "lunch-reminder":
            pass

        elif self._event["name"] == "dinner-reminder":
            pass

        return [
            {"id": uuid4().hex, "hasKey": k, "hasValue": v}
            for k, v in event_parameters.items()
        ]
