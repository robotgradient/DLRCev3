# ev3dev
import paho.mqtt.client as mqtt
import config
import ev3dev.ev3 as ev3
from sensors import publish_sensor_data
from actuators import receive_actuator_properties,initialize_actuator_client_functions

client = mqtt.Client()
client.connect(config.BROKER_IP, config.BROKER_PORT, keepalive=60)
initialize_actuator_client_functions(client)

counter = 0
# Endless loop that keeps sending all sensor properties and updates once per loop the actuator properties.
while(1):
    publish_sensor_data(client=client)
    receive_actuator_properties(client=client)
    counter += 1
    if counter % 10:
        print('still running... {}'.format(counter))
client.disconnect()