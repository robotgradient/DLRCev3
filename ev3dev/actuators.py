import paho.mqtt.client as mqtt
import config
import ev3dev.ev3 as ev3

# See also https://github.com/rhempel/ev3dev-lang-python/blob/develop/ev3dev/core.py
actuators = [
    ev3.LargeMotor(config.LARGE_MOTOR_PORT_1),
    ev3.LargeMotor(config.LARGE_MOTOR_PORT_2),
    ev3.MediumMotor(config.MEDIUM_MOTOR_PORT),
]
actuator_names = [actuator.__str__() for actuator in actuators]
actuators_and_names_dict = {actuator_name: actuator for actuator_name, actuator in zip(actuator_names, actuators)}


def receive_actuator_properties(client):
    # Only a single time read messages, no separate thread.
    # This should invoke on_message for received messages
    client.loop_read()


# Define function that is called when message arrives:
# Distributes sensor data to corresponding sensor class instances
def _set_actuator_data(client, userdata, msg):
    actuator_name = msg.topic
    property_name, property_value = msg.payload.decode().rsplit('+', 1)
    setattr(actuators_and_names_dict[actuator_name], property_name, property_value)


def _subscribe_to_actuators(client, userdata, flags, rc):  # on_connect
    print("Connected with result code "+str(rc))
    for actuator in actuators:
        topic = actuator.__str__()
        client.subscribe(topic)


def initialize_actuator_client_functions(client):
    client.on_message = _set_actuator_data
    client.on_connect = _subscribe_to_actuators