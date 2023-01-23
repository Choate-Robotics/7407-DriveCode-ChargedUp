import subsystem
import sensors
import wpilib


class Robot:
    pass


class Pneumatics:
    compressor = wpilib.Compressor(1, wpilib.PneumaticsModuleType.REVPH)

    @classmethod
    def get_compressor(cls):
        return cls.compressor.enabled(), cls.compressor.getCurrent()


class Sensors:
    pass
