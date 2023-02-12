from wpilib import SerialPort

class LEDS():
    
    class style:
        """
        The style of the LEDs
        
        Members:
        KSolid: Solid 
        KBlink: Blinking
        KTrack: Dash 
        KRainbow: Rainbow
        """
        KSolid = "S"
        KBlink = "B"
        KTrack = "T"
        KRainbow = "R"
        
    class rgb():
        
        def __init__(self, red: int, green: int, blue: int) -> None:
            
            self.red = red
            self.green = green
            self.blue = blue
            
        def rgb_to_hex(self):
            return '#%02x%02x%02x' % (self.red, self.green, self.blue)
    
        
    def led_command(self, style: style, color1: rgb, color2: rgb = None, color3: rgb = None, color4: rgb = None):
        return style + "|" + color1.rgb_to_hex() + "-" + color2.rgb_to_hex() + "-" + color3.rgb_to_hex() + "-" + color4.rgb_to_hex()
    
    def write_config(self):
        """writes the config to the LEDs.
        """
        self.leds.write(bytes(self.config, 'utf-8'), len(self.config))
    
    def __init__(self, config: led_command = None) -> None:
        """
        initializes the LEDs and sets the config to the default config. 
        Connects via USB to the roboRIO on baud rate 9600.
        Intended for use with Arduino or similar microcontroller to offload LEDS from the roboRIO.
        """
        self.leds = SerialPort(9600, SerialPort.Port.kUSB)
        if not config:
            self.config = "S|#FFA500-#FFA500-#0000FF-#0000FF"
        else:
            self.config = config
        self.write_config()
            
    
    
    def set_leds(self, style: style, color1: rgb, color2: rgb = None, color3: rgb = None, color4: rgb = None):
        """sets the LEDs to a certain color and style. Can be used with 1-4 colors for all combinations of styles.

        Args:
            style (style): sets the style of the LEDs
            color1 (rgb): the first color
            color2 (rgb, optional): the second color. Defaults to None.
            color3 (rgb, optional): the third color. Defaults to None.
            color4 (rgb, optional): the fourth color. Defaults to None.
        """
        #set the colors to the last color if they are not set
        
        if color2 is None:
            color2 = color1
            
        if color3 is None:
            color3 = color2
        
        if color4 is None:
            color4 = color3

        #create the string

        final_string = self.led_concat(style, color1, color2, color3, color4)
        
        #turn string to bytes
        
        color = bytes(final_string, 'utf-8')
        
        self.leds.write(color, len(color))
    
    def off(self):
        """turns off the LEDs
        """
        self.set_leds(self.style.KSolid, self.rgb(0, 0, 0))
        
    def solid(self, color: rgb):
        """sets the LEDs to a single solid color

        Args:
            color (rgb): the color
        """
        self.set_leds(self.style.KSolid, color)
        
    def blink(self, color: rgb):
        """sets the LEDs to blink a single color

        Args:
            color (rgb): the color
        """
        self.set_leds(self.style.KBlink, color)
        
    def rainbow(self):
        """sets the LEDs to rainbow mode
        """
        self.set_leds(self.style.KRainbow, self.rgb(0, 0, 0))
        
    def track(self, color: rgb):
        """sets the LEDs to track using a single color

        Args:
            color1 (rgb): the color
        """
        self.set_leds(self.style.KTrack, color)
        
    def set_config(self, config: led_command):
        """sets the config of the LEDs

        Args:
            config (str): the config
        """
        self.config = config
        
    def get_config(self):
        """gets the config of the LEDs

        Returns:
            str: the config
        """
        return self.config
    