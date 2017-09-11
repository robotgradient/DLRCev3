import paho.mqtt.client as mqtt
import config
import ev3dev.ev3 as ev3

# ev3dev has a python interface for sensors.
# The sensor classes have properties which are updated under the hood.
# For simplicity, we will just constantly stream all @properties of a sensor-class
# from ev3 to jetson (and update the sensor class there).
# See also https://github.com/rhempel/ev3dev-lang-python/blob/develop/ev3dev/core.py
touch_sensor = ev3.TouchSensor(config.TOUCH_SENSOR_PORT)
infrared_sensor = ev3.InfraredSensor(config.INFRARED_SENSOR_PORT)
color_sensor = ev3.ColorSensor(config.COLOR_SENSOR_PORT)
sensors = [touch_sensor, infrared_sensor, color_sensor]

# Make dict where key is sensor instance and value
# is a list of all properties of this sensor
items_to_publish = {sensor: [] for sensor in sensors}
for sensor in sensors:
    for member, dtype in sensor.__class__.__dict__.items():
        if isinstance(dtype, property):
            items_to_publish[sensor].append(member)


# Create a function, that sends the properties (=sensor values)
# The sensor name used in ev3dev will be the "topic" and
# the message will be the property name and corresponding value, separated by a "_"
def publish_sensor_data(client):
    for sensor, property_names in items_to_publish.items():
        for property_name in property_names:
            sensor_name = sensor.__str__()
            property_value = getattr(sensor, property_name)
            msg = "{}+{}".format(property_name, property_value)
            client.publish(topic=sensor_name, payload=msg)



