import paho.mqtt.client as mqtt
import IOInterface.jetson.config as config
import ev3dev.ev3 as ev3
from IOInterface.jetson.sensors import Ev3Sensor


class Broker(object):
    """
    This class listens on the configured port to messages from ev3 and sets the
    corresponding entries in the sensor class.
    The Broker also sends out messages to the ev3 actuators.
    """
    def __init__(self, sensors_and_names_dict, ip_address, port):
        self.sensors_and_names_dict = sensors_and_names_dict
        self.client = mqtt.Client()
        self.client.on_connect = self._subscribe_to_sensors
        self.client.on_message = self._set_sensor_data
        self.ip_address = ip_address
        self.port = port

    # Define function that is called on connecting: Subscribes to all sensor topics
    def _subscribe_to_sensors(self, client, userdata, flags, rc):  # on_connect
        print("Connected with result code "+str(rc))
        for sensor_name in self.sensors_and_names_dict.keys():
            client.subscribe(sensor_name)

    # Define function that is called when message arrives:
    # Distributes sensor data to corresponding sensor class instances
    def _set_sensor_data(self, client, userdata, msg):  # on_message
        sensor = self.sensors_and_names_dict[msg.topic]
        property_name, property_value = msg.payload.decode().rsplit('+', 1)
        setattr(sensor, property_name, property_value)

    def start_listen_sensors(self):
        print("start listening...")
        self.client.loop_start()

    def stop_listen_sensors(self):
        print("stop listening...")
        self.client.loop_stop()

    def send_message(self, actuator_name, property_name, property_value):
        msg = "{}+{}".format(property_name, property_value)
        self.client.publish(topic=actuator_name, payload=msg)

    def connect(self):
        self.client.connect(self.ip_address, self.port, keepalive=60)

    def disconnect(self):
        self.client.disconnect()
