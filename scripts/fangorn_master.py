"""Example of a master script.

Run this *after* you have the slave client all set up and running (it will print that it's ready!)
Look at ev3control/messages.py to see what kinds of messages you can send currently
"""
from ev3control import start_master, publish_cmd
from ev3control.messages import *
from time import sleep

def callback(client, userdata, msg):
    print(msg.payload.decode())


m = start_master()
m.on_message = callback
m.loop_start()

# This tests how messages get dropped
publish_cmd(m, AddObjectMessage("test", 'ColorSensor("in1")'))
publish_cmd(m, ShowAttrMessage("test", 'reflected_light_intensity'))
publish_cmd(m, AddObjectMessage("test", 'LightSensor("in3")'))
publish_cmd(m, AddObjectMessage("test", 'LightSensor("in2")'))
publish_cmd(m, ShowAttrMessage("test", 'reflected_light_intensity'))
