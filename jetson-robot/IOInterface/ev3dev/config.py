# NOTE: This config should be the same as on the Jetson!

BROKER_IP = "10.180.226.162"  # This is the IP address of your Jetson, connected to EV3 (check with ifconfig)
BROKER_PORT = 1883

ACTUATORS = 'actuators'

TOUCH_SENSOR_PORT = "in1"
INFRARED_SENSOR_PORT = "in2"
COLOR_SENSOR_PORT = "in3"

LARGE_MOTOR_PORT_1 = "outA"
LARGE_MOTOR_PORT_2 = "outB"
MEDIUM_MOTOR_PORT = "outC"
