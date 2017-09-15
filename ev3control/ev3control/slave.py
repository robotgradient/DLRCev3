"""MQTT client that listens for commands from a master and turns them into Ev3 commands"""
from functools import partial

import paho.mqtt.client as mqtt
from ev3dev.ev3 import *
from ev3dev.core import Motor
import logging

from .messages import *
from ev3control.utils import MASTER_COMMANDS, SLAVE_RESPONSES, decode_mqtt

MASTER_HOST = "localhost"
LOGGER_FORMAT = '%(asctime)-15s %(message)s'

logging.basicConfig(format=LOGGER_FORMAT, level=logging.DEBUG)


def dont_crash(func):
    """Stop-gap decorator for preventing the slave from crashing in cases of errors.

    TODO: replace exception printing with logging.
    """

    def robust(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            print(e)
            return None

    return robust


@dont_crash
def print_property(objects, obj_name, attr_name):
    return getattr(objects[obj_name], attr_name, "Not set")


@dont_crash
def set_property(objects, obj_name, attr_name, val):
    setattr(objects[obj_name], attr_name, val)


@dont_crash
def run_method(objects, obj_name, method_name, args):
    return getattr(objects[obj_name], method_name)(**args)


def publish_value(client, message, delay=0):
    """Convenience wrapper around MQTT's publish method.

    :message: should be one of the types defined in messages.py
    """
    print("publishing value now")
    client.publish(topic=SLAVE_RESPONSES, payload=repr(message))
    # If we chain multiple publish commands, we need delays between them
    time.sleep(delay)


def process_message(objects: dict, client, userdata, msg):
    """Callback for processing an MQTT message.

    Assumes the message payload can be evalueated to one of the message types
    defined in `messages` module.
    """
    message, time_stamp = decode_mqtt(msg).split(';')
    message = eval(message)
    logging.debug("Receiving message sent at {}".format(time_stamp))
    if isinstance(message, ShowAttrMessage):
        value = print_property(objects, *message)
        print("Object value: ", value)
        publish_value(client, PrintAttrMessage(message.obj_name, message.attr_name, value))
    elif isinstance(message, SetAttrMessage):
        print("Value before: ", print_property(objects, message.obj_name, message.attr_name))
        set_property(objects, *message)
        print("Value after:", print_property(objects, message.obj_name, message.attr_name))
    elif isinstance(message, RunMethodMessage):
        print('running method')
        run_method(objects, *message)
    elif isinstance(message, AddObjectMessage):
        print("adding object!")
        objects[message.obj_name] = eval(message.obj_init)
        print("new objects", objects)
    else:
        print("not a valid message type!")


def run_slave(host=MASTER_HOST):
    """Convenience function for setting up an MQTT client and running its listening loop.

    :param host: can be an IP or hostname.
    """
    client = mqtt.Client()
    client.connect(host, 1883, keepalive=60)
    all_objects = {}
    client.on_message = partial(process_message, all_objects)
    client.subscribe(MASTER_COMMANDS)
    print("Client is set up, gonna start listening now!")
    try:
        client.loop_forever()
    finally:
        # Gracefully shut down all motors if slave is interrupted
        for obj in all_objects.values():
            if isinstance(obj, Motor):
                obj.stop_action(action="brake")
