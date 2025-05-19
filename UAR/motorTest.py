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
 
# GPIO pin setup
up_pin = 5 # brown
down_pin = 6 # red
encoder_pin = 27
up_limit = 26 # green
down_limit = 19 # yellow

def setup():
  print("setup")
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(up_pin, GPIO.OUT)
  GPIO.setup(down_pin, GPIO.OUT)
  GPIO.setup(encoder_pin, GPIO.IN)
  GPIO.setup(down_limit, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(up_limit, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.output(up_pin, False)
  GPIO.output(down_pin, False)
  print(f"down: {down_limit}, up: {up_limit}")

def move_up():
  print("moving up")
  while True:
    if GPIO.input(up_limit):
      print("up limit hit, stopping")
      GPIO.output(up_pin, False)
      GPIO.output(down_pin, False)
    else:
      GPIO.output(up_pin, True)
      GPIO.output(down_pin, False)

def move_down():
  print("moving down")
  while True:
    if GPIO.input(down_limit):
      print("down limit hit, stopping")
      GPIO.output(up_pin, False)
      GPIO.output(down_pin, False)
    else:
      GPIO.output(up_pin, False)
      GPIO.output(down_pin, True)

def main():
  try:
    setup()
    time.sleep(2)
    move_down()
    time.sleep(2)
    move_up()
  except KeyboardInterrupt:
    return
  finally:
    print("exiting")
    GPIO.cleanup()

main()

