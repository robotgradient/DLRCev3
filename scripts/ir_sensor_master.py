"""Test infrared sensor"""
from ev3control import Robot

robot = Robot({"infraredSensor": 'InfraredSensor("in1")'}, cap=None)

print(robot.read_proximity_sensor())
