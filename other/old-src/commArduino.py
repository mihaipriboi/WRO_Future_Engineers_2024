import time
from pyb import UART
import pyb

# UART 3, and baudrate.
uart = UART(3, 19200)

red_led = pyb.LED(1)
green_led = pyb.LED(2)
blue_led = pyb.LED(3)

red_led.on()
blue_led.on()
time.sleep(1)
red_led.off()
blue_led.off()
time.sleep(1)

while ( True ):
    if ( uart.any() == 0 ):
        uart.write('-70\n')
        # red_led.on()
        # time.sleep(0.5)
        # red_led.off()
        # time.sleep(0.5)
# send message:
#    else:
#        msg = ''
#        while ( uart.any() > 0 ):
#            msg += uart.read().decode('utf-8')
#        print(msg)
#    print("hello world\n")
    time.sleep(0.1)
