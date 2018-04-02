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
