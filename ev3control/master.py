"""Master Client implementation."""
import time
import paho.mqtt.client as mqtt

from .messages import *


def start_master():
    """Start MQTT client with setup that makes it a master."""
    client = mqtt.Client()
    client.connect("localhost", 1883, keepalive=60)
    return client


def publish_cmd(client, message, delay=.2):
    """Convenience wrapper around MQTT's publish method.

    :message: should be one of the types defined in messages.py
    """
    client.publish(topic="commands", payload=repr(message))
    # If we chain multiple publish commands, we need delays between them
    time.sleep(delay)


if __name__ == '__main__':
    from messages import *
    m = start_master()
    publish_cmd(m, ShowAttrMessage("test", "max_speed"))
