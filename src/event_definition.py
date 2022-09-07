EVENT_TYPES = ["activity", "scheduled"]


EVENT_DEFINITIONS = [
    {
        "id": "0",
        "name": "medicine-reminder",
        "description": "Reminder for taking medicine.",
        "type": "scheduled",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-take-medicine",
            },
        },
        "parameters": {
            "schedule": "* *-*-* 21:30:00",
            "medicine_name": "calcium supplement",
        },
    },
    {
        "id": "1",
        "name": "entering-home",
        "description": "Alert when user has entered home.",
        "type": "activity",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-entering-home",
            },
        },
        "parameters": {},
    },
    {
        "id": "2",
        "name": "leaving-home",
        "description": "Alert when user is leaving.",
        "type": "activity",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-leaving-home",
            },
        },
        "parameters": {},
    },
    {
        "id": "3",
        "name": "rain-alert",
        "description": "Alerts related to weather.",
        "type": "activity",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-rain-alert",
            },
        },
        "parameters": {
            "time": "15:00",
            "items": ["umbrella"],
        },
    },
    {
        "id": "4",
        "name": "noise_alert",
        "description": "Alert when noise is detected.",
        "type": "activity",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-noise-alert",
            },
        },
        "parameters": {
            "source": "kitchen",
        },
    },
    {
        "id": "5",
        "name": "bill-payment",
        "description": "Reminder for paying bills.",
        "type": "scheduled",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-bill-payment",
            },
        },
        "parameters": {
            "schedule": "* 2022-09-03 22:00:00",
            "bills": [
                {
                    "type": "gas",
                    "amount": "4000",
                },
                {
                    "type": "electricity",
                    "amount": "7000",
                },
                {
                    "type": "water",
                    "amount": "3000",
                },
            ],
        },
    },
    {
        "id": "6",
        "name": "health-checkup",
        "description": "Reminder for taking medicine.",
        "type": "scheduled",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-health-checkup",
            },
        },
        "parameters": {
            "schedule": "* 2022-09-03 14:00:00",
            "checkup_type": "",
            "reservation": {
                "datetime": "",
                "number": "",
                "location": "",
                "doctor": "",
            },
        },
    },
    {
        "id": "7",
        "name": "garbage-reminder",
        "description": "Reminder for taking out garbage.",
        "type": "scheduled",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-garbage-reminder",
            },
        },
        "parameters": {
            "schedule": "* 2022-09-02 22:00:00",
            "garbage_type": "burnable",
        },
    },
    {
        "id": "8",
        "name": "earthquake-alert",
        "description": "Alert about earthquake.",
        "type": "activity",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-earthquake-alert",
            },
        },
        "parameters": {
            "intensity": "",
            "source": "",
            "source_distance": "",
            "tsunami_warning": "",
            "instruction": "",
        },
    },
    {
        "id": "9",
        "name": "tsunami-alert",
        "description": "Alert about tsunami.",
        "type": "activity",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-tsunami-alert",
            },
        },
        "parameters": {
            "water_height": "",
            "instruction": "",
        },
    },
    {
        "id": "10",
        "name": "heat-alert",
        "description": "Alerts related to high temp.",
        "type": "activity",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-heat-alert",
            },
        },
        "parameters": {
            "time_period": {
                "start": "15:00",
                "end": "15:00",
            },
            "temperature": "42.0",
            "items": ["umbrella"],
            "instruction": "",
        },
    },
    {
        "id": "11",
        "name": "breakfast-reminder",
        "description": "Breakfast reminder.",
        "type": "scheduled",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-breakfast-reminder",
            },
        },
        "parameters": {
            "schedule": "* *-*-* 08:00:00",
        },
    },
    {
        "id": "12",
        "name": "dinner-reminder",
        "description": "Dinner reminder",
        "type": "scheduled",
        "context": {
            "strawberry-ros-dialog": {
                "intent_name": "event-handler-dinner-reminder",
            },
        },
        "parameters": {
            "schedule": "* *-*-* 21:00:00",
        },
    },
    # {
    #     "id": "13",
    #     "name": "daily",
    #     "description": "Daily reminder",
    #     "type": "scheduled",
    #     "context": {
    #         "strawberry-ros-dialog": {
    #             "intent_name": "event-handler-daily-reminder",
    #         },
    #     },
    #     "parameters": {
    #         "datetime": "",
    #     },
    # },
    # {
    #     "id": "14",
    #     "name": "hourly",
    #     "description": "Hourly reminder",
    #     "type": "scheduled",
    #     "context": {
    #         "strawberry-ros-dialog": {
    #             "intent_name": "event-handler-hourly-reminder",
    #         },
    #     },
    #     "parameters": {
    #         "datetime": "",
    #     },
    # },
]


EVENT_UI_DEFINITIONS = {
    "0": {
        "image": "pills-solid-96",
    },
    "1": {
        "image": "house-solid-96",
    },
    "2": {
        "image": "person-walking-luggage-solid-96",
    },
    "3": {
        "image": "cloud-showers-heavy-solid-96",
    },
    "4": {
        "image": "audio-96",
    },
    "5": {
        "image": "money-bill-wave-solid-96",
    },
    "6": {
        "image": "hospital-solid-96",
    },
    "7": {
        "image": "trash-can-solid-96",
    },
    "8": {
        "image": "earthquakes-96",
    },
    "9": {
        "image": "house-tsunami-solid-96",
    },
    "10": {
        "image": "temperature-high-solid-96",
    },
    "11": {
        "image": "mug-saucer-solid-96",
    },
    "12": {
        "image": "utensils-solid-96",
    },
    # "13": {
    #     "image": "calendar-days-solid-96",
    # },
    # "14": {
    #     "image": "clock-solid-96",
    # },
}
