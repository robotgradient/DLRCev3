from IOInterface.jetson.broker import Broker
import IOInterface.jetson.sensors as sensors
import IOInterface.jetson.actuators as actuators
import ev3dev.ev3 as ev3
import time
import IOInterface.jetson.config as config
import time
from IOInterface.jetson.sensors import IMU, OnBoardCamera


# Set up ev3 Sensors. We create ev3 instances and give them as argument to Ev3Sensor class,
# which reads all @property entries and makes corresponding variable in the Sensor class.
ev3_sensors = [
    ev3.TouchSensor(config.TOUCH_SENSOR_PORT),
    ev3.InfraredSensor(config.INFRARED_SENSOR_PORT),
    ev3.ColorSensor(config.COLOR_SENSOR_PORT),
]
sensors = [sensors.Ev3Sensor(ev3_sensor) for ev3_sensor in ev3_sensors]
sensors_and_names_dict = {sensor.name: sensor for sensor in sensors}

# Set up Broker for communication between Jetson and Ev3
broker = Broker(sensors_and_names_dict, config.BROKER_IP, config.BROKER_PORT)
broker.connect()
broker.start_listen_sensors()

# Set up ev3 Motors.
ev3_actuators = [
    ev3.LargeMotor(config.LARGE_MOTOR_PORT_1),
    ev3.LargeMotor(config.LARGE_MOTOR_PORT_2),
    ev3.MediumMotor(config.MEDIUM_MOTOR_PORT),
]
actuators = [actuators.Ev3Actuator(broker=broker, ev3_actuator=ev3_actuator) for ev3_actuator in ev3_actuators]
actuators_and_names_dict = {actuator.name: actuator for actuator in actuators}


# Test communication between Jetson and Ev3. Note that main.py on ev3 must be running.

print("Test Sensors")
print('Press the Touch Sensor')
for _ in range(100):
    print("{}: {}".format(sensors[0].name, sensors[0].is_pressed))
    time.sleep(0.2)


print("Test Motors")
broker.send_message(actuator_name='MediumMotor(outC)',
                    property_name='position_sp',
                    property_value=360)
time.sleep(0.1)
broker.send_message(actuator_name='MediumMotor(outC)',
                    property_name='run_to_rel_pos',
                    property_value='position_sp')

time.sleep(5)


print("Test Cam on Jetson")
cam = OnBoardCamera()
for _ in range(10):
    time.sleep(0.2)
    cam_data = cam.read()
    print("cam images shape: {}".format(cam_data))

print("Test IMU on Jetson")
imu = IMU()
for _ in range(10):
    time.sleep(0.2)
    imu_data = imu.read()
    print("imu data: {}".format(imu_data))

time.sleep(1)

