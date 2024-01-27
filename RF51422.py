
from utime import sleep

class ANT51422:
    """Class for reading gyro rates and acceleration data from an MPU-6050 module via I2C."""

    def __init__(self, UartCon, Reset_Pin: machine.Pin, Slp_Pin: machine.Pin):
        self.Uart = UartCon
        self.ResetPin = Reset_Pin
        self.CSPin = Slp_Pin
        self.ResetPin.value(1)
        self.CSPin.value(1)

    def Ant51422Reset(self):
        # De-Assert RESET pin
        print("Ant51422 Reset")
        self.ResetPin.value(0)
        sleep(1)
        self.ResetPin.value(1)

    def Ant51422Sleep(self):
        # De-Assert CS pin to sleep
        if self.CSPin.value() != 0:
            print("Ant51422 Sleep")
            self.CSPin.value(0)

    def Ant51422Wake(self):
        # Assert CS pin to wake
        if self.CSPin.value() != 1:
            print("Ant51422 Wake")
            self.CSPin.value(1)

    def PrintReadUart(self,data):
        if data == None:
            print("none")
            return
        for rdata in data:
            print(hex(rdata),", ",end='')
        print('')

    def SetAntSample(self):
        #self.Uart.write('hello')  # write 5 bytes
        sleep(1)
        # Set Network Key
        #print("Set Network Key")
        #self.Uart.write(bytes([0x09,0x46,0x01,0xb9,0xa5,0x21,0xfb,0xbd,0x72,0xc3,0x45]))
        sleep(1)
        self.PrintReadUart(self.Uart.readline())
        # Set Channel Number (channel_assign)
        print("Set Channel Number (channel_assign)")
        #self.Uart.write(bytes([0x03,0x42,0x01,0x10,0x01]))
        self.Uart.write(bytes([0x03,0x42,0x00,0x10,0x00]))
        sleep(1)
        self.PrintReadUart(self.Uart.readline())
        # Set Channel ID  0xb for power meter, 0x7b for speed sensor 
        print("Set Channel ID")
        #self.Uart.write(bytes([0x05,0x51,0x00,0xfa,0x8c,0x8b,0x45]))
        self.Uart.write(bytes([0x05,0x51,0x00,0x01,0x00,0x80,0x10]))
        sleep(1)
        self.PrintReadUart(self.Uart.readline())    
        # Set Channel RF frequency
        print("Set Channel RF frequency")
        self.Uart.write(bytes([0x02,0x45,0x01,0x39]))
        sleep(1)
        self.PrintReadUart(self.Uart.readline())    
        # Set Channel period
        print("Set Channel period")
        self.Uart.write(bytes([0x03,0x43,0x01,0x86,0x1f]))
        sleep(1)
        self.PrintReadUart(self.Uart.readline())    
        # Fill first frame
        print("Fill first frame")
        self.Uart.write(bytes([0x09,0x4e,0x01,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08]))
        sleep(2)
        self.PrintReadUart(self.Uart.readline())    
        # Open Channel
        print("Open Channel")
        self.Uart.write(bytes([0x01,0x4b,0x01]))
        sleep(1)
        self.PrintReadUart(self.Uart.readline())    