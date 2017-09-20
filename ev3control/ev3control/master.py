"""Master Client implementation."""
import time
import paho.mqtt.client as mqtt
from datetime import datetime

from .messages import *
from ev3control.utils import MASTER_COMMANDS, SLAVE_RESPONSES


def start_master():
    """Start MQTT client with setup that makes it a master."""
    client = mqtt.Client()
    client.connect("localhost", 1883, keepalive=60)
    client.subscribe(SLAVE_RESPONSES)
    return client


def publish_cmd(client, message):
    """Convenience wrapper around MQTT's publish method.

    :message: should be one of the types defined in messages.py
    """
    print("published")
    client.publish(
        topic=MASTER_COMMANDS,
        payload=repr(message) + ";" + datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S,%f')[:-3])


if __name__ == '__main__':
    from messages import *
    m = start_master()
    publish_cmd(m, ShowAttrMessage("test", "max_speed"))
