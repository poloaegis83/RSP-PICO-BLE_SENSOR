from machine import Pin, ADC
from utime import sleep
import bluetooth
from Ble_Advertising import advertising_payload
from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)

_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)

POWER_UUID    = bluetooth.UUID(0x1818)
POWER_CHAR    = (bluetooth.UUID(0x2A63), _FLAG_READ | _FLAG_NOTIFY,)
POWER_SERVICE = (POWER_UUID, (POWER_CHAR,),)

Power_Revolutions = 0  # this is counter for power measurements Revolutions field
Power_Timestamp   = 0  # this is Timestamp for power measurements Timestamp field

def get_povwe_values():
    adc = ADC(Pin(28))
    power = adc.read_u16()*0.01 +50
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