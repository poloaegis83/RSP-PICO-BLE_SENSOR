from machine import Pin, ADC, Timer, I2C
from utime import sleep
import bluetooth
from Ble_Advertising import advertising_payload
from micropython import const
import random


_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)

_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)
_FLAG_WRITE = const(0x0008)
_FLAG_INDICATE = const(0x0020)

POWER_UUID           =  bluetooth.UUID(0x1818)
POWER_CHAR           = (bluetooth.UUID(0x2A63), _FLAG_NOTIFY,)
POWER_CONTROL_CHAR   = (bluetooth.UUID(0x2A66), _FLAG_WRITE | _FLAG_INDICATE,)
POWER_FEATURE_CHAR   = (bluetooth.UUID(0x2A65), _FLAG_READ ,)
SENSOR_LOCATION_CHAR = (bluetooth.UUID(0x2A5D), _FLAG_READ ,)
POWER_SERVICE = (POWER_UUID, (POWER_CHAR,POWER_FEATURE_CHAR,SENSOR_LOCATION_CHAR,POWER_CONTROL_CHAR),)

Crank_Revolutions = 0  # this is counter for power measurements Crank Revolutions field
#Crank_Revolutions_L = 0
Crank_Timestamp   = 0  # this is Timestamp for power measurements Last Crank Event Time field
#Crank_Timestamp_L = 0
#Crank_Revolutions_out = 0
#rpm = 0
Crank_Length = 0.1725

angle_count = 0
crank_event_counter = 0

sensing_time = 50 # in ms, 20 times per sec(20Hz)
X_fact = 0.012 # force/vlotage curve
#LastVlot   = []  # compare last _LAST_DATA Vlotage data
LastPower  = []   # compare last _LAST_DATA Power data
#volt_count = 0
#volt_postive = 0  # 1 postive, 2 nagtive
cali_offset = 0
gyro_offset = [0,0,0]

chip_freq = 125000000

debug_gyro_z = 0

global mpu

class MPU6050:
    """Class for reading gyro rates and acceleration data from an MPU-6050 module via I2C."""

    def __init__(self, i2c:machine.I2C, address:int = 0x68):
        """
        Creates a new MPU6050 class for reading gyro rates and acceleration data.
        :param i2c: A setup I2C module of the machine module.
        :param address: The I2C address of the MPU-6050 you are using (0x68 is the default).
        """
        self.address = address
        self.i2c = i2c
        self.first_round = 0
        self.gyro_fs_sel = 0

    def wake(self) -> None:
        """Wake up the MPU-6050."""
        self.i2c.writeto_mem(self.address, 0x6B, bytes([0x01]))

    def sleep(self) -> None:
        """Places MPU-6050 in sleep mode (low power consumption). Stops the internal reading of new data. Any calls to get gyro or accel data while in sleep mode will remain unchanged - the data is not being updated internally within the MPU-6050!"""
        self.i2c.writeto_mem(self.address, 0x6B, bytes([0x40]))

    def Set_low_power_gyroZ_only(self) -> None:
        #Set CYCLE bit to 1 and TEMP_DIS to 1
        self.i2c.writeto_mem(self.address, 0x6B, bytes([0x28]))
        # LP_WAKE_CTRL = b'10'(20Hz), disable Accelerometer XYZ and gyroscope XY into standby mode (set to 1)
        self.i2c.writeto_mem(self.address, 0x6C, bytes([0xBE]))
        #self.i2c.writeto_mem(self.address, 0x6C, bytes([0x7E]))
    def who_am_i(self) -> int:
        """Returns the address of the MPU-6050 (ensure it is working)."""
        return self.i2c.readfrom_mem(self.address, 0x75, 1)[0]

    def read_temperature(self) -> float:
        """Reads the temperature, in celsius, of the onboard temperature sensor of the MPU-6050."""
        data = self.i2c.readfrom_mem(self.address, 0x41, 2)
        raw_temp:float = self._translate_pair(data[0], data[1])
        temp:float = (raw_temp / 340.0) + 36.53
        return temp

    def read_gyro_range(self) -> int:
        """Reads the gyroscope range setting."""
        return self._hex_to_index(self.i2c.readfrom_mem(self.address, 0x1B, 1)[0])

    def write_gyro_range(self, range:int) -> None:
        """Sets the gyroscope range setting."""
        self.i2c.writeto_mem(self.address, 0x1B, bytes([self._index_to_hex(range)]))

    def read_gyro_data(self) -> tuple[float, float, float]:
        """Read the gyroscope data, in a (x, y, z) tuple."""
        # set the modified based on the gyro range (need to divide to calculate)
        gr:int = self.read_gyro_range()
        modifier:float = None
        if gr == 0:
            modifier = 131.0
        elif gr == 1:
            modifier = 65.5
        elif gr == 2:
            modifier = 32.8
        elif gr == 3:
            modifier = 16.4

        # read data
        data = self.i2c.readfrom_mem(self.address, 0x43, 6) # read 6 bytes (gyro data)
        x:float = (self._translate_pair(data[0], data[1])) / modifier
        y:float = (self._translate_pair(data[2], data[3])) / modifier
        z:float = (self._translate_pair(data[4], data[5])) / modifier

        return (x, y, z)

    def read_gyro_data_z(self) -> tuple[float, float, float]:
        """Read the gyroscope data, in  z."""
        # set the modified based on the gyro range (need to divide to calculate)
        if self.first_round == 0:
            self.gyro_fs_sel:int = self.read_gyro_range()
            self.first_round = 1
        modifier:float = None
        if self.gyro_fs_sel == 0:
            modifier = 131.0
        elif self.gyro_fs_sel == 1:
            modifier = 65.5
        elif self.gyro_fs_sel == 2:
            modifier = 32.8
        elif self.gyro_fs_sel == 3:
            modifier = 16.4

        # read data
        data = self.i2c.readfrom_mem(self.address, 0x47, 2) # read 2 bytes (gyro data)
        z:float = (self._translate_pair(data[0], data[1])) / modifier

        return z

    def read_accel_range(self) -> int:
        """Reads the accelerometer range setting."""
        return self._hex_to_index(self.i2c.readfrom_mem(self.address, 0x1C, 1)[0])

    def write_accel_range(self, range:int) -> None:
        """Sets the gyro accelerometer setting."""
        self.i2c.writeto_mem(self.address, 0x1C, bytes([self._index_to_hex(range)]))

    def read_accel_data(self) -> tuple[float, float, float]:
        """Read the accelerometer data, in a (x, y, z) tuple."""

        # set the modified based on the gyro range (need to divide to calculate)
        ar:int = self.read_accel_range()
        modifier:float = None
        if ar == 0:
            modifier = 16384.0
        elif ar == 1:
            modifier = 8192.0
        elif ar == 2:
            modifier = 4096.0
        elif ar == 3:
            modifier = 2048.0

        # read data
        data = self.i2c.readfrom_mem(self.address, 0x3B, 6) # read 6 bytes (accel data)
        x:float = (self._translate_pair(data[0], data[1])) / modifier
        y:float = (self._translate_pair(data[2], data[3])) / modifier
        z:float = (self._translate_pair(data[4], data[5])) / modifier

        return (x, y, z)

    def read_lpf_range(self) -> int:
        return self.i2c.readfrom_mem(self.address, 0x1A, 1)[0]

    def write_lpf_range(self, range:int) -> None:
        """
        Sets low pass filter range.
        :param range: Low pass range setting, 0-6. 0 = minimum filter, 6 = maximum filter.
        """

        # check range
        if range < 0 or range > 6:
            raise Exception("Range '" + str(range) + "' is not a valid low pass filter setting.")

        self.i2c.writeto_mem(self.address, 0x1A, bytes([range]))


    #### UTILITY FUNCTIONS BELOW ####

    def _translate_pair(self, high:int, low:int) -> int:
        """Converts a byte pair to a usable value. Borrowed from https://github.com/m-rtijn/mpu6050/blob/0626053a5e1182f4951b78b8326691a9223a5f7d/mpu6050/mpu6050.py#L76C39-L76C39."""
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value   

    def _hex_to_index(self, range:int) -> int:
        """Converts a hexadecimal range setting to an integer (index), 0-3. This is used for both the gyroscope and accelerometer ranges."""
        if range== 0x00:
            return 0
        elif range == 0x08:
            return 1
        elif range == 0x10:
            return 2
        elif range == 0x18:
            return 3
        else:
            raise Exception("Found unknown gyro range setting '" + str(range) + "'")

    def _index_to_hex(self, index:int) -> int:
        """Converts an index integer (0-3) to a hexadecimal range setting. This is used for both the gyroscope and accelerometer ranges."""
        if index == 0:
            return 0x00
        elif index == 1:
            return 0x08
        elif index == 2:
            return 0x10
        elif index == 3:
            return 0x18
        else:
            raise Exception("Range index '" + index + "' invalid. Must be 0-3.")

def calibration_gyro_offset(mpu):
    gx = 0
    gy = 0
    gz = 0
    index = 10
    index1 = index
    while index1 > 0:
        gyro = mpu.read_gyro_data()
        gx += gyro[0]
        gy += gyro[1]
        gz += gyro[2]
        sleep(0.4)
        index1 -= 1
    gyro_offset[0] = gx / index
    gyro_offset[1] = gy / index
    gyro_offset[2] = gz / index
    print("calibration_gyro_offset = (x,y,z) ",gyro_offset[0],",",gyro_offset[1],",",gyro_offset[2])

def calibration():
    global cali_offset
    VoltA = 0
    t = 20
    index = t
    adc = ADC(Pin(28))
    while(index > 0):  
        VoltA += adc.read_u16()
        index -= 1
        sleep(0.3)
    calibration_bottom = 15 # percent
    cali_offset = VoltA/t
    cali_offset += (calibration_bottom/100)*cali_offset
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
    #AngV = 500 + random.random()*70
    #global debug_gyro_z
    #print("before read")
    gyro = mpu.read_gyro_data_z()
    gyro_z = gyro - gyro_offset[2]
    if abs(gyro_z) < 1.5:
        gyro_z = 0
    if gyro_z <= 0:  # take native gyro_z only, and reverse it to postive values
        gyro_z = abs(gyro_z)
    else:
        gyro_z = 0

    #print("Gyro-Z = ",gyro_z,"count = ",debug_gyro_z)
    #debug_gyro_z += 1

    AngV = gyro_z
    #AngV = 520  # degree/s (rpm ~= 90)
    #print("AngularVelocity=",AngV)
    #return cadence
    return AngV

def calculate_power():
    global angle_count, Crank_Revolutions, Crank_Timestamp, crank_event_counter
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
    power = force * Perimeter * (AngV/360) * (sensing_time/1000) # P = Force * 2πr * (angle traveled/360, per 50 ms)
  
    if power < 1:
        power = 0
    power = power * 2

    #print("power = force * (distance)  =",power,"Watts")
    #rpm = AngV * 60 / 360

    # calculate crank event
    angle_count += AngV * (sensing_time/1000) # in 50ms
    crank_event_counter += 1

    if angle_count > 360:       # crank event (one rotation)
        Crank_Timestamp += int((crank_event_counter*sensing_time)/1000 * 1024)
        crank_event_counter = 0
        while angle_count > 360:    # crank event (one rotation)
            angle_count -= 360
            Crank_Revolutions += 1
            print("Crank_Revolutions =",Crank_Revolutions)

    #print("angle_count =",angle_count)
    #print("rpm =",rpm)
    #Smoothing_power(power)

    return power

def get_power_values(): # In a second
    # get average power in a second
    global LastPower
    PowerOut = sum(LastPower)
    #print("sum power / (",len(LastPower),")=",sum(LastPower),"average = ",PowerOut)
    smooth_power = Smoothing_power(PowerOut)
    return int(smooth_power)

class BlePowerMeter:
    def __init__(self, ble, name):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ( (self.pm_handle,self.pf_handle,self.sl_handle,self.pcp_handle,),) = self._ble.gatts_register_services((POWER_SERVICE,))
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
        power = calculate_power()
        # update power data (50ms) into power list, full of list is 20 data
        if len(LastPower) == (1000/sensing_time):
            LastPower.append(power)
            LastPower.pop(0)
            #print("sum power25 =",sum(LastPower),"average = ",sum(LastPower)/len(LastPower))
        else:
            LastPower.append(power)
            #print("sum power / (",len(LastPower),")=",sum(LastPower),"average = ",sum(LastPower)/len(LastPower))

    def SendPowerfeature(self): # https://github.com/oesmith/gatt-xml  , for bit field
        Features = bytearray([0x08,0x00,0x00,0x00])
        self._ble.gatts_write(self.pf_handle, Features) # send power data by ble

    def SendSensorLocation(self): #https://github.com/oesmith/gatt-xml , for bit field
        Location = bytearray([0x05]) #Left Crank
        self._ble.gatts_write(self.sl_handle, Location) # send power data by ble

    def SendPowerFlagOnly(self):
        flags = 0x20 #https://github.com/oesmith/gatt-xml , for bit field
        Power_values =bytearray([flags & 0xff ,flags>>8 & 0xff,0x00,0x00,0x00,0x00,0x00,0x00])
        self._ble.gatts_write(self.pm_handle, Power_values)
        for conn_handle in self._connections:
            self._ble.gatts_notify(conn_handle, self.pm_handle)

    def SendPowerData(self, notify=True, indicate=False):
        #global Crank_Revolutions, Crank_Revolutions_L, Crank_Revolutions_out,Crank_Timestamp, Crank_Timestamp_L, rpm, angle_count
        flags = 0x20 #https://github.com/oesmith/gatt-xml , for bit field
        #revo = 0
        #if angle_count > 360:
        #    Crank_Timestamp   += ((angle_count/360)*60)*   #((angle_count/360)*60) rpm
        #while angle_count > 360:
        #    angle_count -= 360
        #    Crank_Revolutions += 1
            #Crank_Timestamp   += 1024

        #if Crank_Revolutions_L != 0:
        #    revo = int(math.floor(Crank_Revolutions) - math.floor(Crank_Revolutions_L)) # compare wtih last time, revo = Crank_Revolutions difference between 2 sensing
        #print("Crank_Revolutions",Crank_Revolutions,"Crank_Revolutions_L = ",Crank_Revolutions_L,"rpm/60 = ",(rpm/60))
        #Crank_Revolutions_L = Crank_Revolutions

        #Crank_Revolutions_out = Crank_Revolutions_out + revo

        #if revo != 0:
        #    Crank_Timestamp   = int(Crank_Timestamp + (revo/rpm * 60) * 1024) # Crank_Timestamp (Last Even times) unit = 1/1024 sec
                                                                              # for calculate (timestamp crank Revolutions add times(revo) / rpm) *60 = seconds traveled in revo.
        #print("rpm =",rpm,"Crank_Revolutions_out = ",Crank_Revolutions_out, "revo =",revo, "Timestamp = ",Crank_Timestamp)
        #print("Crank_Revolutions = ",Crank_Revolutions, "Timestamp = ",Crank_Timestamp)
        PowerV = get_power_values()
        print("PowerV = ",PowerV)
        Power_values =bytearray([flags & 0xff ,flags>>8 & 0xff,PowerV & 0xff, PowerV >>8 & 0xff,Crank_Revolutions & 0xff,Crank_Revolutions>>8 & 0xff,Crank_Timestamp & 0xff,Crank_Timestamp>>8 & 0xff]) # 8 bytes data per package
        self._ble.gatts_write(self.pm_handle, Power_values) # send power data by ble
        if notify or indicate:
            for conn_handle in self._connections:
                if notify:
                    # Notify connected centrals.
                    self._ble.gatts_notify(conn_handle, self.pm_handle)
                if indicate:
                    # Indicate connected centrals.
                    self._ble.gatts_indicate(conn_handle, self.pm_handle)
button_mode = 0

def Buttoncallback1(t):
    global button_mode
    print("button_mode=",button_mode)
    if button_mode == 0:
        button_mode = 1
    elif button_mode == 1:
        button_mode = 0

pin = Pin("LED", Pin.OUT)

Button   = machine.Pin(15, machine.Pin.IN,Pin.PULL_UP)
Button.irq (trigger=Button.IRQ_FALLING, handler=Buttoncallback1) #interrupt


i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)
mpu = MPU6050(i2c)

#i2c.writeto_mem(0x68, 0x6B, bytes([0x01]))
mpu.wake()
mpu.write_gyro_range(2)
mpu.Set_low_power_gyroZ_only()

machine.freq(chip_freq)
print("set machine.freq",machine.freq())

calibration_gyro_offset(mpu)

'''
while True:
    gyro = mpu.read_gyro_data()
    #gyro_z = gyro[2]
    gyro_z = gyro[2] - gyro_offset[2]
    if abs(gyro_z) < 1.5:
        gyro_z = 0
    print("pre-gyro_z",gyro_z)
    if gyro_z <= 0:  # take native gyro_z only, and reverse it to postive values
        gyro_z = abs(gyro_z)
    else:
        gyro_z = 0

    print("Gyro: " + str(gyro),"Gyro-Z = ",gyro_z)

    sleep(0.4)
'''
# Create a Bluetooth Low Energy (BLE) object
ble = bluetooth.BLE()
# Create an instance of the BlePowerMeter class with the BLE object
blepm = BlePowerMeter(ble,"PowerMeter_longhao")

counter = 0

print("calibration Start")
calibration()
print("calibration Done")
# Start an infinite loop
connected = 0

def update_power_event(t):
    global connected
    if connected or button_mode == 1:
        #print("update_power_event1")
        blepm.update_power_data()
        #print("update_power_event2")
def send_power_event(t):
    global connected
    print("angle_count =",angle_count)
    if connected or button_mode == 1:
        #print("send_power_event1")
        blepm.SendPowerData(notify=True, indicate=False)
        #print("send_power_event2")
    else:
        blepm.SendPowerfeature()
        blepm.SendSensorLocation()
        blepm.SendPowerFlagOnly()

TimeEvent1 = Timer(-1)
TimeEvent2 = Timer(-1)

TimeEvent1.init(period=sensing_time, mode=Timer.PERIODIC, callback=update_power_event)
TimeEvent2.init(period=1000, mode=Timer.PERIODIC, callback=send_power_event)

while True:
    if blepm.is_connected() and connected == 0:
        pin.value(0)
        sleep(0.5)
        #blepm.SendPowerfeature()
        #blepm.SendSensorLocation()
        connected = 1
    elif not blepm.is_connected():
        #pin.value(1)
        connected = 0
    machine.idle()
    #blepm.update_power_data()
    #if counter % 25 == 0 and counter != 0:
    #blepm.SendPowerData(notify=True, indicate=False)
    #counter = counter + 1
    #sleep(0.04)