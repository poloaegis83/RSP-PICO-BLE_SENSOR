from machine import Pin, ADC, PWM, 
from utime import sleep
import bluetooth
#from Ble_Simple_Peri import BLESimplePeripheral
from Ble_Advertising import advertising_payload

from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE_NO_RESPONSE = const(0x0004)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)
_FLAG_INDICATE = const(0x0020)

_UART_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_READ | _FLAG_NOTIFY,
)
_UART_RX = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    _FLAG_WRITE | _FLAG_WRITE_NO_RESPONSE,
)
_UART_SERVICE = (
    _UART_UUID,
    (_UART_TX, _UART_RX),
)

POWER_UUID    = bluetooth.UUID(0x1818)
POWER_CHAR    = (bluetooth.UUID(0x2A63), _FLAG_READ | _FLAG_NOTIFY,)
POWER_SERVICE = (POWER_UUID, (POWER_CHAR,),)

HR_UUID = bluetooth.UUID(0x180D)
HR_CHAR = (bluetooth.UUID(0x2A37), _FLAG_READ | _FLAG_NOTIFY,)
HR_SERVICE = (HR_UUID, (HR_CHAR,),)

BLE_SERVICE = (HR_SERVICE,_UART_SERVICE,)

_ENV_SENSE_UUID = bluetooth.UUID(0x181A)
_TEMP_CHAR = (
    bluetooth.UUID(0x2A6E),
    _FLAG_READ | _FLAG_NOTIFY | _FLAG_INDICATE,
)
_ENV_SENSE_SERVICE = (
    _ENV_SENSE_UUID,
    (_TEMP_CHAR,),
)

Power_Revolutions = 0  # this is counter for power measurements Revolutions field
Power_Timestamp   = 0  # this is Timestamp for power measurements Timestamp field

def get_povwe_values():
    adc = ADC(Pin(28))
    power = adc.read_u16()*0.01 +50
    return int(power)

def get_heart_rate():
    adc = ADC(Pin(28))
    heart = adc.read_u16()*0.01 +50
    return int(heart)

class BLESimplePeripheral:
    def __init__(self, ble, name):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ( (self.hr_handle,),) = self._ble.gatts_register_services((POWER_SERVICE,))
        #( (self.hr_handle,),) = self._ble.gatts_register_services((HR_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = advertising_payload(name=name, services=[POWER_UUID])
        #self._payload = advertising_payload(name=name, services=[HR_UUID])
        self._advertise()

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            print("New connection", conn_handle)
            self._connections.add(conn_handle)
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            print("Disconnected", conn_handle)
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            self._advertise()
        #elif event == _IRQ_GATTS_WRITE:
        #    conn_handle, value_handle = data
        #    value = self._ble.gatts_read(value_handle)
        #    if value_handle == self._handle_rx and self._write_callback:
        #        self._write_callback(value)

    #def send(self, data):
    #    for conn_handle in self._connections:
    #        self._ble.gatts_notify(conn_handle, self._handle_tx, data)

    def is_connected(self):
        return len(self._connections) > 0

    def _advertise(self, interval_us=50000):
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def on_write(self, callback):
        self._write_callback = callback
    
    def update_heart_rate(self, notify=False, indicate=False):
        heart = get_heart_rate()
        heart_values =bytearray([0x00,heart])
        self._ble.gatts_write(self.hr_handle, heart_values)
        if notify or indicate:
            for conn_handle in self._connections:
                if notify:
                    # Notify connected centrals.
                    self._ble.gatts_notify(conn_handle, self.hr_handle)
                if indicate:
                    # Indicate connected centrals.
                    self._ble.gatts_indicate(conn_handle, self.hr_handle)

    def update_power_data(self, notify=False, indicate=False):
        global Power_Revolutions,Power_Timestamp
        flags = 0x20
        Power_Revolutions = Power_Revolutions + 1
        Power_Timestamp   = Power_Timestamp + 1
        PowerV = get_povwe_values()
        heart_values =bytearray([flags & 0xff ,flags>>8 & 0xff,PowerV & 0xff,PowerV >>8 & 0xff,Power_Revolutions & 0xff,Power_Revolutions>>8 & 0xff,Power_Timestamp & 0xff,Power_Timestamp>>8 & 0xff]) # 8 bytes data per package
        self._ble.gatts_write(self.hr_handle, heart_values)
        if notify or indicate:
            for conn_handle in self._connections:
                if notify:
                    # Notify connected centrals.
                    self._ble.gatts_notify(conn_handle, self.hr_handle)
                if indicate:
                    # Indicate connected centrals.
                    self._ble.gatts_indicate(conn_handle, self.hr_handle)



pin = Pin("LED", Pin.OUT)

# Create a Bluetooth Low Energy (BLE) object
ble = bluetooth.BLE()

# Initialize the LED state to 0 (off)
led_state = 0

# Create an instance of the BLESimplePeripheral class with the BLE object
sp = BLESimplePeripheral(ble,"heart-rate123")

# Define a callback function to handle received data
def on_rx(data):
    print("Data received: ", data)  # Print the received data
    global led_state  # Access the global variable led_state
    if data == b'toggle\r\n':  # Check if the received data is "toggle"
        pin.value(not led_state)  # Toggle the LED state (on/off)
        led_state = 1 - led_state  # Update the LED state


Press_flag = 0

def callback1(pin):
    global Press_flag
    if Press_flag == 0:
        Press_flag = 1
    else:
        Press_flag = 0
    msg="pushbutton pressed\n"
    # Send the message via BLE
    #sp.send(msg)

Button   = machine.Pin(15, machine.Pin.IN,Pin.PULL_UP)
Button.irq (trigger=Button.IRQ_FALLING, handler=callback1) #interrupt
LightSensor = ADC(Pin(28))
counter = 0
# Start an infinite loop
while True:
    #if sp.is_connected():  # Check if a BLE connection is established
    #    sp.on_write(on_rx)  # Set the callback function for data reception
    if not sp.is_connected():
        pin.toggle()
    if counter % 10 == 0:
        #sp.update_heart_rate(notify=True, indicate=False)
        sp.update_power_data(notify=True, indicate=False)
    counter = counter + 1
    sleep(0.1)

'''
pin = Pin("LED", Pin.OUT)

LightLevel = 1000

led_g    = machine.Pin(1, machine.Pin.OUT)
led_y    = machine.Pin(5, machine.Pin.OUT)
led_r    = machine.Pin(9, machine.Pin.OUT)
Button   = machine.Pin(15, machine.Pin.IN,Pin.PULL_UP)
Button.irq (trigger=Button.IRQ_FALLING, handler=callback1) #interrupt
LightSensor = ADC(Pin(28))

pwm_Green  = PWM(Pin(1), freq=1000, duty_u16=5000)
pwm_Yellow = PWM(Pin(5), freq=1000, duty_u16=5000)
pwm_Red    = PWM(Pin(9), freq=1000, duty_u16=5000)

pin.toggle()
counter = 0
OnOff_B = 0

Mode = 0
print("LED starts flashing...")
while True:
    try:
        #print("GP15 Button = ",Button.value())
        LightLevel = LightSensor.read_u16()

        if Press_flag == 0: # button press
            pin.on()
            LightLevel = 66500 - LightLevel*2
            if LightLevel < 0:
                LightLevel = 0
        elif Press_flag == 1:
            pin.off()
            LightLevel = LightLevel*3
            if LightLevel > 65535:
                LightLevel = 65535

        print("GP27 Light = ",LightLevel)
 
        #led_g.value(1)
        #led_y.value(0)
        #led_r.value(0)
        if counter % 3 == 0:
            pwm_Green.duty_u16(LightLevel)
        else:
            pwm_Green.duty_u16(0)
        if counter % 3 == 1:
            pwm_Yellow.duty_u16(LightLevel)
        else:
            pwm_Yellow.duty_u16(0)
        if counter % 3 == 2:
            pwm_Red.duty_u16(LightLevel)
        else:
            pwm_Red.duty_u16(0)
        sleep(0.3) # sleep 1sec
        counter = counter+1

    except KeyboardInterrupt:
        break
pin.off()
print("Finished.")


#def BleSetup():
'''