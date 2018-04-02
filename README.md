# RPi.GPIO-Odroid  
  
RPi.GPIO port with support for RPi, Odroid C1, Odroid C2, and Odroid XU4  
  

## Based on:  
  
RPi.GPIO by Ben Croston  
  https://sourceforge.net/projects/raspberry-gpio-python/  

Hardkernel WiringPi port  
  https://github.com/hardkernel/wiringPi  

swkim01's Odroid C1 RPi.GPIO port  
  https://github.com/swkim01/RPi.GPIO-OdroidC1  
  
  
## License  
RPi.GPIO is distributed under MIT license, but wiringPi uses LGPL 3.  Since this project  
contains code from both projects, it is licensed under the slightly more restrictive LGPL v3.  
  
  
## Status  
Working for simple IO on C1, XU4, RPi  
Likely working on C2, but no device to test  
PWM, events, analog read, ... not implemented  
  
  
## Building  
1) Install build-essential, python, python-dev, git  
2) git clone https://github.com/jfath/RPi.GPIO-Odroid.git  
3) cd RPi.GPIO-Odroid  
4) sudo python setup.py clean --all  
5) sudo python setup.py build install  
  

## Usage  
Simple test app:  
  
```
#Read state of GPIO output on GPIO input  
#Use jumper wire from pin 13 to pin 31  
#XU4 without shifter-shield pin 13 to pin 19  
import RPi.GPIO as GPIO  
import time  
  
LedPinW = 27    # pin13, bcm27  
LedPinR = 6    # pin31, bcm6  
  
def setup():  
  GPIO.setmode(GPIO.BCM)       # Number GPIOs by BCM chip numbering scheme  
  GPIO.setup(LedPinR, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # Set LedPinR mode input  
  GPIO.setup(LedPinW, GPIO.OUT)   # Set LedPinW mode to output  
  GPIO.output(LedPinW, GPIO.HIGH) # Set LedPinW pin high  
  
def blink():  
  while True:  
    GPIO.output(LedPinW, GPIO.HIGH)  # LedPinW high  
    time.sleep(2)  
    pstate=GPIO.input(LedPinR)  # Read LedPinR  
    print("*****Pin state (LedPinW HIGH) ", pstate, "*****\n")  
    time.sleep(2)  
    GPIO.output(LedPinW, GPIO.LOW) # LedPinW low  
    time.sleep(2)  
    pstate=GPIO.input(LedPinR)  # Read LedPinR  
    print("*****Pin state (LedPinW LOW) ", pstate, "*****\n")  
    time.sleep(2)  
  
def shutdown():  
  GPIO.output(LedPinW, GPIO.LOW)   # LedPinW low  
  GPIO.setup(LedPinW, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)   # LedPinW input  
  GPIO.cleanup()  
  
if __name__ == '__main__':     # Program start  
  print('To read output correctly, jumper pin 13 (bcm27) to pin 31 (bcm6)')
  print('Press Ctrl-C to exit') 
  setup()  
  print("Hardware information: ", GPIO.RPI_INFO)
  try:  
    blink()  
  except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, shut down cleanly  
    shutdown()
 
```
  
  
## Notes  
Apps require root (eg 'sudo python testapp.py')  
When using BCM mode, RPi BCM numbers are passed to GPIO.xxx and translated internally to Odroid GPIO numbers  
Compare RPi connector pinout / BCM chart and Odroid pinout to match RPi BCM with Odroid pins  
Odroid XU4 pin numbers use shifter-shield numbers  
  
