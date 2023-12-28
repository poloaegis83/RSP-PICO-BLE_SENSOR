from machine import Pin, ADC
from utime import sleep
import bluetooth
from Ble_Advertising import advertising_payload
from micropython import const

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)

_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)

HR_UUID = bluetooth.UUID(0x180D)
HR_CHAR = (bluetooth.UUID(0x2A37), _FLAG_READ | _FLAG_NOTIFY,)
HR_SERVICE = (HR_UUID, (HR_CHAR,),)

def get_heart_rate():
    adc = ADC(Pin(28))
    heart = adc.read_u16()*0.01 +50
    return int(heart)

class BleHeartRate:
    def __init__(self, ble, name):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ( (self.hr_handle,),) = self._ble.gatts_register_services((HR_SERVICE,))
        self._connections = set()
        self._write_callback = None
        self._payload = advertising_payload(name=name, services=[HR_UUID])
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

pin = Pin("LED", Pin.OUT)
# Create a Bluetooth Low Energy (BLE) object
ble = bluetooth.BLE()
# Create an instance of the BleHeartRate class with the BLE object
BleHr = BleHeartRate (ble,"HeartRate_longhao")

counter = 0
# Start an infinite loop
while True:

    if not BleHr.is_connected():
        pin.toggle()
    if counter % 10 == 0:
        BleHr.update_heart_rate(notify=True, indicate=False)
    counter = counter + 1
    sleep(0.1)