import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
 
# GPIO pin setup
up_pin = 17
down_pin = 18
encoder_pin = 27
up_limit = 22
down_limit = 23
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
  move_up()
  time.sleep(2)
  move_down()

main()

