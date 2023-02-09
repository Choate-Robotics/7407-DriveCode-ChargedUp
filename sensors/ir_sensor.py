import wpilib
import math

class IrSensor():

  #GP2Y0A51SK0F 2cm to 15cm
    """
    Sharp GP2Y0A41SK0F is an analog IR sensor capable of measuring
    distances from 2cm to 15cm. Output distance is measured in
    centimeters.

    Distance is calculated using the following equation derived from
    the graph provided in the datasheet::

    12.84*x ^ -0.9824

    .. warning:: FRC Teams: the case on these sensors is conductive and 
    grounded, and should not be mounted on a metallic surface!
    """

    def __init__(self, port):
        """:param port: Analog port number"""
        self.distance_sensor = wpilib.AnalogInput(port)

    def getDistance(self):
        """
        :returns: distance in METERS. The output is 
        constrained to be between 1.75 and 13.125 cm.
        """

        # Don't allow zero/negative values
        v = max(self.distance_sensor.getVoltage(), 0.00001)
        d = 12.84 * math.pow(v, -0.9824)

        # Constrain output
        return max(min(d, 13.125), 1.75) * 100

    def getVoltage(self):
        return max(self.distance_sensor.getVoltage(), 0.00001)