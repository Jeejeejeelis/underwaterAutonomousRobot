#    /======== WIRES ===========
# 2 // GROUND             BLUE
# 3 // Upper limit switch GREEN
# 4 // Lower limit switch YELLOW
# 5 // Mystery            ORANGE
# 6 // Motor down         RED
# 7 // Motor up           BROWN
# 8 //==========================

import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
 
# GPIO pin setup
up_pin = 5 # brown
down_pin = 6 # red
encoder_pin = 27
up_limit = 26 # green
down_limit = 19 # yellow
GPIO.setup(up_pin, GPIO.OUT)
GPIO.setup(down_pin, GPIO.OUT)
GPIO.setup(encoder_pin, GPIO.IN)
GPIO.setup(down_limit, GPIO.IN)
GPIO.setup(up_limit, GPIO.IN)

def move_up():
  while True:
    if GPIO.input(up_limit):
      GPIO.output(up_pin, False)
      GPIO.output(down_pin, False)
    else:
      GPIO.output(up_pin, True)
      GPIO.output(down_pin, False)

def move_down():
  while True:
    if GPIO.input(down_limit):
      GPIO.output(up_pin, False)
      GPIO.output(down_pin, False)
    else:
      GPIO.output(up_pin, False)
      GPIO.output(down_pin, True)

def main():
  time.sleep(2)
  move_down()
  time.sleep(2)
  move_up()

main()

