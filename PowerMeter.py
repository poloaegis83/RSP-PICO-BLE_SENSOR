from machine import Pin, ADC
from utime import sleep
import bluetooth
from Ble_Advertising import advertising_payload
from micropython import const
import random

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)

_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)

_LAST_DATA = const(30)

POWER_UUID    = bluetooth.UUID(0x1818)
POWER_CHAR           = (bluetooth.UUID(0x2A63), _FLAG_READ | _FLAG_NOTIFY,)
POWER_FEATURE_CHAR   = (bluetooth.UUID(0x2A65), _FLAG_READ ,)
SENSOR_LOCATION_CHAR = (bluetooth.UUID(0x2A5D), _FLAG_READ ,)
POWER_SERVICE = (POWER_UUID, (POWER_CHAR,POWER_FEATURE_CHAR,SENSOR_LOCATION_CHAR),)

Power_Revolutions = 0  # this is counter for power measurements Revolutions field
Power_Timestamp   = 0  # this is Timestamp for power measurements Timestamp field

Crank_Length = 0.1725

sensing_time = 40 # in ms, 25 times per sec(25Hz)
X_fact = 0.012 # force/vlotage curve
#LastVlot   = []  # compare last _LAST_DATA Vlotage data
LastPower  = []   # compare last _LAST_DATA Power data
#volt_count = 0
#volt_postive = 0  # 1 postive, 2 nagtive
cali_offset = 0

def calibration():
    global cali_offset
    VoltA = 0
    t = 23
    index = t
    while(index > 0):
        adc = ADC(Pin(28))
        VoltA = VoltA + adc.read_u16()
        index = index-1
        sleep(0.3)
    calibration_bottom = 10 # percent
    cali_offset = VoltA/t
    cali_offset = cali_offset + (calibration_bottom/100)*cali_offset
    print("calibration offset = ",cali_offset)


EMA = 0
def Smoothing_power(Pdata):
    global EMA
    if EMA == 0:
        EMA = Pdata
    EMA = (Pdata*4 + EMA)/5
    NewPdata = EMA
    print("Smoothing_power = ",NewPdata)
    return NewPdata
    None # "moving average" ?

def get_sensor_vaule(): # get voltage vaule from amp
    global volt_count, volt_postive
    adc = ADC(Pin(28))
    sensor_v = adc.read_u16()
    return sensor_v

def AngularVelocity():
    # get from IMU
    AngV = 360 + random.random()*50
    #AngV = 520  # degree/s (rpm ~= 90)
    #print("AngularVelocity=",AngV)
    #return cadence
    return AngV

def caculate_power():

    volt = get_sensor_vaule()
    if volt < cali_offset:
        volt = 0
    else:
        volt = volt - cali_offset
    force = volt * X_fact
    #print("force N= ",force,"vlot=",volt)
    AngV = AngularVelocity()

    Perimeter = 2 * 3.14159 * Crank_Length # 2πr

    #print("Perimeter= ",Perimeter)
    power = force * Perimeter * (AngV/360) # P = Force * 2πr * (angle traveled/360, per sec)
  
    power = power * 2
    #print("power = force * (distance)  =",power,"Watts")
    rpm = AngV * 60 / 360
    #print("rpm =",rpm)
    #Smoothing_power(power)

    return power

def get_power_values(): # In a second
    # get average power in a second
    global LastPower
    PowerOut = sum(LastPower)/len(LastPower)
    print("sum power / (",len(LastPower),")=",sum(LastPower),"average = ",PowerOut)
    smooth_power = Smoothing_power(PowerOut)
    return int(PowerOut)

class BlePowerMeter:
    def __init__(self, ble, name):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ( (self.pm_handle,self.pf_handle,self.sl_handle,),) = self._ble.gatts_register_services((POWER_SERVICE,))
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
    
    def update_power_data(self):
        global LastPower
        power = caculate_power()
        # update power data (40ms) into power list, full of list is 25 data
        if len(LastPower) == 25:
            LastPower.append(power)
            LastPower.pop(0)
            #print("sum power25 =",sum(LastPower),"average = ",sum(LastPower)/len(LastPower))
        else:
            LastPower.append(power)
            #print("sum power / (",len(LastPower),")=",sum(LastPower),"average = ",sum(LastPower)/len(LastPower))

    def Update_Power_feature(self):
        Features = bytearray([0x00,0x00,0x00,0x08])
        self._ble.gatts_write(self.pm_handle, Features) # send power data by ble

    def SendPowerData(self, notify=True, indicate=False):
        global Power_Revolutions,Power_Timestamp
        flags = 0x20
        Power_Revolutions = Power_Revolutions + 1
        Power_Timestamp   = Power_Timestamp + 1
        PowerV = get_power_values()
        Power_values =bytearray([flags & 0xff ,flags>>8 & 0xff,PowerV & 0xff,PowerV >>8 & 0xff,Power_Revolutions & 0xff,Power_Revolutions>>8 & 0xff,Power_Timestamp & 0xff,Power_Timestamp>>8 & 0xff]) # 8 bytes data per package
        self._ble.gatts_write(self.pm_handle, Power_values) # send power data by ble
        if notify or indicate:
            for conn_handle in self._connections:
                if notify:
                    # Notify connected centrals.
                    self._ble.gatts_notify(conn_handle, self.pm_handle)
                if indicate:
                    # Indicate connected centrals.
                    self._ble.gatts_indicate(conn_handle, self.pm_handle)

pin = Pin("LED", Pin.OUT)
# Create a Bluetooth Low Energy (BLE) object
ble = bluetooth.BLE()
# Create an instance of the BlePowerMeter class with the BLE object
blepm = BlePowerMeter(ble,"PowerMeter_longhao")

counter = 0
print("calibration Start")
calibration()
print("calibration Done")
# Start an infinite loop
while True:
    #if not blepm.is_connected():
    #    pin.toggle()
    if blepm.is_connected():
        sleep(0.5)
        blepm.Update_Power_feature()
    blepm.update_power_data()
    if counter % 25 == 0 and counter != 0:
        blepm.SendPowerData(notify=True, indicate=False)
    counter = counter + 1
    sleep(0.04)