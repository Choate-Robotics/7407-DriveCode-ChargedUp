from wpilib import AddressableLED, PowerDistribution
import math
class ALeds():
    '''Addressable LEDS from PWM RIO'''
    m_led: AddressableLED
    # active_mode: dict
    # speed: int
    # brightness: float
    # last_active_mode: dict
    # last_speed: int
    # last_brightness: float
    # m_ledBuffer: AddressableLED.LEDData
    class Type():
        
        def KStatic(r, g, b):
            return {
                'type': 1,
                'color': {
                    'r': r,
                    'g': g,
                    'b': b
                }
            }
        
        def KRainbow():
            return {
                'type': 2
            }
        
        def KTrack(r1, g1, b1, r2, g2, b2):
            return {
                'type': 3,
                'color': {
                    'r1': r1,
                    'g1': g1,
                    'b1': b1,
                    'r2': r2,
                    'g2': g2,
                    'b2': b2
                }
            }
        
        def KBlink(r,g,b):
            return {
                'type': 4,
                'color': {
                    'r': r,
                    'g': g,
                    'b': b
                }
            }
        
        def KClimb():
            return {
                'type': 5,
            }
    
    def __init__(self, id: int, size: int):
        self.size = size
        self.id = id
        self.speed = 5
        self.track_index = 0
        self.blink_index = 0
        self.active_mode = None
        self.last_active_mode = None
        self.last_brightness = None
        self.last_speed = None
        self.brightness = 1
        self.track_speed = 1 #out of 5
        
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
        
        
    def enable(self):
        self.m_led.start()
    
    def disable(self):
        self.m_led.stop()
        
    def run(self):
        pass
    
    def storeCurrent(self):
        self.last_active_mode = self.active_mode
        self.last_speed = self.speed
        self.last_brightness = self.brightness
    
    def setLED(self, type, brightness: float = 1.0, speed: int = 5):
        try:
            self.storeCurrent()
            self.active_mode = type
            self.speed = speed
            self.brightness = brightness
        except Exception:
            pass
        
    def getLED(self):
        if self.active_mode == None:
            return {
                'type': 0,
                'color': {
                    'r': 0,
                    'g': 0,
                    'b': 0
                }
            }
        else:
            return self.active_mode
        
    def setLast(self):
        self.active_mode = self.last_active_mode
        self.speed = self.last_speed
        self.brightness = self.last_brightness
                
    def cycle(self): 
        '''
        cycles through LED array
        this should be called periodically
        '''
        try:
            match self.active_mode['type']:
                case 1:
                    color = self.active_mode['color']
                    self._setStatic(color['r'], color['g'], color['b'])
                case 2:
                    self._setRainbow()
                case 3:
                    color = self.active_mode['color']
                    self._setTrack(color['r1'], color['g1'], color['b1'], color['r2'], color['g2'], color['b2'])
                case 4:
                    color = self.active_mode['color']
                    self._setBlink(color['r'], color['g'], color['b']) 
                case 5:
                    self._setClimb()
        except Exception:
            pass
        
    def _setStatic(self, red: int, green: int, blue: int):
        for i in range(self.size):
            self.array[i].setRGB(red, green, blue)
        self.m_led.setData(self.array)
        
    def _setRainbow(self):
        for i in range(len(self.array)):
            # Calculate the hue - hue is easier for rainbows because the color
            # shape is a circle so only one value needs to precess
            hue = math.floor((self.m_rainbowFirstPixelHue + (i * 180 / len(self.array))) % 180)
            # Set the value
            self.array[i].setHSV(hue, 255, 128)
    
        # Increase by to make the rainbow "move"
        self.m_rainbowFirstPixelHue += self.speed
        # Check bounds
        self.m_rainbowFirstPixelHue %= 180
        self.m_led.setData(self.array)

        
    def _setTrack(self, r1, g1, b1, r2, g2, b2):
        for i in range(len(self.array)):
            self.array[i].setRGB(r1, g1, b1)
            
            for i in range(0, len(self.array), 6):
                if i <= len(self.array):
                    for j in range(3):
                        if i + self.track_index + j >= len(self.array):
                            self.array[(i + self.track_index + j) - len(self.array)].setRGB(r2, g2, b2)
                        else:
                            self.array[i + self.track_index + j].setRGB(r2,g2,b2)
            
        
        self.track_index += 1 
        
        if self.track_index > len(self.array): 
            self.track_index = 0
        
        self.m_led.setData(self.array)
        
    def _setBlink(self, r,g,b):
        if self.blink_index / 10 <= .5:
            for i in range(len(self.array)):
                self.array[i].setRGB(r, g, b)
        else:
            for i in range(len(self.array)):
                self.array[i].setRGB(0,0,0)
        
        self.blink_index += 1
        if self.blink_index > 10:
            self.blink_index = 0   
        self.m_led.setData(self.array)

    def _setClimb(self):
        for i in range(len(self.array)):
            self.array[i].setRGB(0, 225, 0)
        
        for i in range(12):
            self.array[i].setRGB(225, 0, 0)
        self.m_led.setData(self.array)
class SLEDS:
    '''Switchable LEDS from Switchable PDH'''
    
    def on(self):
        PowerDistribution.setSwitchableChannel(True)
        
    def off(self):
        PowerDistribution.setSwitchableChannel(False)