from machine import Pin, ADC, PWM, 
from utime import sleep
import bluetooth
from Ble_Simple_Peri import BLESimplePeripheral

pin = Pin("LED", Pin.OUT)

# Create a Bluetooth Low Energy (BLE) object
ble = bluetooth.BLE()

# Initialize the LED state to 0 (off)
led_state = 0

# Create an instance of the BLESimplePeripheral class with the BLE object
sp = BLESimplePeripheral(ble)

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
    sp.send(msg)

Button   = machine.Pin(15, machine.Pin.IN,Pin.PULL_UP)
Button.irq (trigger=Button.IRQ_FALLING, handler=callback1) #interrupt
LightSensor = ADC(Pin(28))

# Start an infinite loop
while True:
    if sp.is_connected():  # Check if a BLE connection is established
        sp.on_write(on_rx)  # Set the callback function for data reception






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