import paho.mqtt.client as mqtt
import IOInterface.jetson.config as config
import ev3dev.ev3 as ev3


class Actuator(object):
    def __init__(self):
        pass

    def get_actuator_name(self):
        raise ValueError('This function must be implemented in sub-class')


class Ev3Actuator(Actuator):
    """
    Actuator class for ev3dev actuators (motors).
    Sets all entries for @property of the given ev3 actuator (see properties on ev3dev python git repo).
    The broker sends these as messages to ev3dev

    For communication to work, ev3dev must run its main.py and a connection must be set up.
    Check out ev3dev homepage for setting up network connection and configure the correct port
    of BROKER_IP in config.py on both ev3 and jetson.

    Note: In this implementation, you cannot read current actuator state. (You will probably need this)
    """
    def __init__(self, ev3_actuator, broker, *args, **kwargs):
        super(Ev3Actuator, self).__init__(*args, **kwargs)
        self.broker = broker
        self.ev3_actuator = ev3_actuator
        self.name = self.ev3_actuator.__str__()
        self.properties_list = []
        for member, dtype in self.ev3_actuator.__class__.__dict__.items():
            if isinstance(dtype, property):
                # add property and initialize value with None
                self.properties_list.append(member)
                setattr(self, member, None)

    def set(self, property_name, property_value):
        """
        Sets Actuator properties, using the broker to send a message to the ev3 actuator.
        :param property_name: str: name of property to set
        :param property_value: value of othe property, in the corresponding dtype
        :return:
        """
        self.broker.send_message(self.get_actuator_name(), property_name, property_value)

    def get_actuator_name(self):
        return self.topic
