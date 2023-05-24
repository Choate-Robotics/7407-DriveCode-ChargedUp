from wpilib import AddressableLED, PowerDistribution
import math
class ALEDS:
    '''Addressable LEDS from PWM RIO'''
    m_led: AddressableLED
    # m_ledBuffer: AddressableLED.LEDData
    
    def __init__(self, id: int, size: int):
        self.size = size
        self.id = id
        
    def init(self):
        self.m_rainbowFirstPixelHue = 0
        self.m_led = AddressableLED(self.id)
        self.m_led.setLength(self.size)
        self.m_ledBuffer = self.m_led.LEDData
        self.array = []
        for i in range(self.size):
            self.array.append(self.m_led.LEDData())
        self.m_led.setData(self.array)
        self.m_led.start()
        
    def disable(self):
        self.m_led.stop()
        
        
    def setStatic(self, red: int, green: int, blue: int):
        for i in range(self.size):
            self.array[i].setRGB(red, green, blue)
        self.m_led.setData(self.array)
        
    def setRainbow(self):
        for i in range(len(self.array)):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = math.floor((self.m_rainbowFirstPixelHue + (i * 180 / len(self.array))) % 180)
            # Set the value
            self.array[i].setHSV(hue, 255, 128)
    
        # Increase by to make the rainbow "move"
        self.m_rainbowFirstPixelHue += 3
        # Check bounds
        self.m_rainbowFirstPixelHue %= 180
        self.m_led.setData(self.array)
        
    def setTrack(self):
        pass
        
        
class SLEDS:
    '''Switchable LEDS from Switchable PDH'''
    
    def on(self):
        PowerDistribution.setSwitchableChannel(True)
        
    def off(self):
        PowerDistribution.setSwitchableChannel(False)