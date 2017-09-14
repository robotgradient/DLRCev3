"""Example of a master script.

Run this *after* you have the slave client all set up and running (it will print that it's ready!)
Look at ev3control/messages.py to see what kinds of messages you can send currently
"""
from ev3control import start_master, publish_cmd
from ev3control.messages import *

m = start_master()
print("sending message")
# This message adds a device. The advantage of this is that you can control
# from the master how you name the devices and then use the same names to access them.
publish_cmd(m, AddObjectMessage("test", 'LargeMotor("outA")'))
print("sending message")
# this asks the client to show the value of a property.
publish_cmd(m, ShowAttrMessage("test", "min_speed"))


publish_cmd(m, RunMethodMessage("test", "run_timed", {"time_sp" : 3000, "speed_sp" : 500}))
