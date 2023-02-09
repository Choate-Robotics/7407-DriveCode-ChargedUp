import wpilib
import math








class SharpIR2Y0A51:

  #GP2Y0A51SK0F 2cm to 15cm
    """
    Sharp GP2Y0A41SK0F is an analog IR sensor capable of measuring
    distances from 4cm to 40cm. Output distance is measured in
    centimeters.

    Distance is calculated using the following equation derived from
    the graph provided in the datasheet::

        12.84*x ^ -0.9824

    .. warning:: FRC Teams: the case on these sensors is conductive and
                 grounded, and should not be mounted on a metallic
                 surface!
    """

    def __init__(self, port):
        """:param port: Analog port number"""
        self.distance = wpilib.AnalogInput(port)

    def getDistance(self):
        """
        :returns: distance in centimeters. The output is constrained to
                  be between 4.5 and 35
        """

        # Don't allow zero/negative values
        v = max(self.distance.getVoltage(), 0.00001)
        d = 12.84 * math.pow(v, -0.9824)

        # Constrain output
        return max(min(d, 35.0), 4.5)