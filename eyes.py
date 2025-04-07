from gpiozero import MCP3008

""" Hello there, I added the sensor.py file in the end named eyes.py """

# Read from channel 0 (where IR sensor analog pin is connected)
ir_sensor1 = MCP3008(channel=0)
ir_sensor2 = MCP3008(channel=1)
ir_sensor3 = MCP3008(channel=2)
ir_sensor4 = MCP3008(channel=3)
ir_sensors = [ir_sensor1, ir_sensor2, ir_sensor3, ir_sensor4]

voltage = ir_sensor1.value * 3.3  # Convert to voltage
print(f"Voltage: {voltage:.2f} V")

# You can map voltage to distance based on the sensor's datasheet
# Example: ~0.4V to 2.5V maps to 10cm to 80cm for Sharp GP2Y0A21


def angled_sensor():
    pass
    """ 
    This is for the angled sensor at the front of the robot. 
    If is black tile or red tile, return is wall, if yellow is slope else is normal tiles
    then need to be updated into sensor values in the other function
    """


def ir_sensor():
    sensor_values = []
    for i in range(4):
        voltage1 = ir_sensors[i].value * 3.3
        sensor_values[i] = voltage1
        # Need to be updated later to convert the voltage into distance, unsure of model of ir sensor yet
    return sensor_values
