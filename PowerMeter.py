from machine import Pin, ADC
from utime import sleep
import bluetooth
from Ble_Advertising import advertising_payload
from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)

_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)

_LAST_DATA = const(30)

POWER_UUID    = bluetooth.UUID(0x1818)
POWER_CHAR    = (bluetooth.UUID(0x2A63), _FLAG_READ | _FLAG_NOTIFY,)
POWER_SERVICE = (POWER_UUID, (POWER_CHAR,),)

Power_Revolutions = 0  # this is counter for power measurements Revolutions field
Power_Timestamp   = 0  # this is Timestamp for power measurements Timestamp field

sensing_time = 40 # in ms
X_fact = 1.0 # force/vlotage curve
LastVlot   = []  # compare last _LAST_DATA Vlotage data
LastPower  = []  # compare last _LAST_DATA Power data

def Smoothing_poewr():
    None # "moving average" ?

def get_sensor_vaule(): # get voltage vaule from amp
    adc = ADC(Pin(28))
    sensor_v = adc.read_u16()
    LastVlot.append(sensor_v)
    if len(LastVlot) == _LAST_DATA: # if length = _LAST_DATA pop first one
        LastVlot.pop(0)
    return int(sensor_v)

def calcute_cadence():
    # see Last Vlot to calcute cadence
    Postive = 0
    PostiveLast = -1
    step = 0
    for Volt in LastVlot:
        if Volt > 0:
            Postive = 1
        if Volt < 0:
            Postive = 0
        if (Postive == 0 and PostiveLast == 1) or (Postive == 1 and PostiveLast == 0):
            # one roation
            step = step + 1
        PostiveLast = Postive
    cadence = _LAST_DATA * sensing_time / step # total time / step (roation times)
    print("cadence=",cadence)
    return cadence

def caculate_power():
    volt = get_sensor_vaule()
    force = volt * X_fact
    rpm = calcute_cadence()
    power = force * rpm
    return power

def get_power_values():
    power = caculate_power()
    LastPower.append(power)
    if len(LastPower) == _LAST_DATA: # if length = _LAST_DATA pop first one
        LastPower.pop(0)
    #smooth_power = Smoothing_poewr()
    return int(power)

class BlePowerMeter:
    def __init__(self, ble, name):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ( (self.hr_handle,),) = self._ble.gatts_register_services((POWER_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = advertising_payload(name=name, services=[POWER_UUID])
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

    def is_connected(self):
        return len(self._connections) > 0

    def _advertise(self, interval_us=50000):
        print("Starting advertising")
        self._ble.gap_advertise(interval_us, adv_data=self._payload)
    
    def update_power_data(self, notify=False, indicate=False):
        global Power_Revolutions,Power_Timestamp
        flags = 0x20
        Power_Revolutions = Power_Revolutions + 1
        Power_Timestamp   = Power_Timestamp + 1
        PowerV = get_power_values()
        Power_values =bytearray([flags & 0xff ,flags>>8 & 0xff,PowerV & 0xff,PowerV >>8 & 0xff,Power_Revolutions & 0xff,Power_Revolutions>>8 & 0xff,Power_Timestamp & 0xff,Power_Timestamp>>8 & 0xff]) # 8 bytes data per package
        self._ble.gatts_write(self.hr_handle, Power_values) # send power data by ble
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
# Create an instance of the BlePowerMeter class with the BLE object
blepm = BlePowerMeter(ble,"PowerMeter_longhao")

counter = 0
# Start an infinite loop
while True:
    if not blepm.is_connected():
        pin.toggle()
    if counter % 10 == 0:
        blepm.update_power_data(notify=True, indicate=False)
    counter = counter + 1
    sleep(0.1)