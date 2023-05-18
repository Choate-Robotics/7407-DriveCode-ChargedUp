from wpilib import AddressableLED, PowerDistribution

class ALEDS:
    '''Addressable LEDS from PWM RIO'''
    m_led: AddressableLED
    # m_ledBuffer: AddressableLED.LEDData
    
    def __init__(self, id: int, size: int):
        self.size = size
        self.id = id
        self.m_led = AddressableLED(id)
        self.m_led.setLength(size)
        self.m_ledBuffer = self.m_led.LEDData
        
    def enable(self):
        self.m_led.start()
        
    def disable(self):
        self.m_led.stop()
        
        
    def setStatic(self, r: int, g: int, b: int):
        for i in range(self.size):
            self.m_ledBuffer[i].setRGB(r= r, g= g, b= b)
        self.m_led.setData(self.m_ledBuffer)
        
    def setTrack(self):
        pass
        
        
class SLEDS:
    '''Switchable LEDS from Switchable PDH'''
    
    def on(self):
        PowerDistribution.setSwitchableChannel(True)
        
    def off(self):
        PowerDistribution.setSwitchableChannel(False)