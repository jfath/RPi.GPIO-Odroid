import RPi.GPIO as GPIO
import time

LedPinW = 27    # pin13, bcm27
LedPinR = 6    # pin31, bcm6

def setup():
  GPIO.setmode(GPIO.BCM)       # Numbers GPIOs by chip numbering scheme
  GPIO.setup(LedPinR, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # Set LedPin's mode is input
  GPIO.setup(LedPinW, GPIO.OUT)   # Set LedPin's mode is output
  GPIO.output(LedPinW, GPIO.HIGH) # Set LedPin high(+3.3V) to turn on led

def blink():
  while True:
    GPIO.output(LedPinW, GPIO.HIGH)  # led on
    time.sleep(2)
    pstate=GPIO.input(LedPinR)
    print("*****Input pin state (Output HIGH) ", pstate, "*****\n")
    time.sleep(2)
    GPIO.output(LedPinW, GPIO.LOW) # led off
    time.sleep(2)
    pstate=GPIO.input(LedPinR)
    print("*****Input pin state (Output LOW) ", pstate, "*****\n")
    time.sleep(2)

def destroy():
  GPIO.output(LedPinW, GPIO.LOW)   # led off
  GPIO.setup(LedPinW, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)   # Set LedPin's mode is input
  GPIO.cleanup()                  # Release resource

if __name__ == '__main__':     # Program start from here
  print('To read output correctly, jumper pin 13 (bcm27) to pin 31 (bcm6)')
  print('Press Ctrl-C to exit') 
  setup()
  try:
    blink()
  except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
    destroy()
