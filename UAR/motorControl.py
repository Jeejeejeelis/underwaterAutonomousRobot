# Python version of the motorControl code for the underwater robot

import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

# GPIO pin setup
up_pin = 17
down_pin = 18
encoder_pin = 24
up_limit = 22
down_limit = 23
GPIO.setup(up_pin, GPIO.OUT)
GPIO.setup(down_pin, GPIO.OUT)
GPIO.setup(encoder_pin, GPIO.IN)
GPIO.setup(down_limit, GPIO.IN)
GPIO.setup(up_limit, GPIO.IN)

GPIO.add_event_detect(encoder_pin, GPIO.BOTH, callback=callback1)

direction = 0 # -1 up, 1 down
goal = 0    # desired number of encoder rotations
current = 0 # current number of eoncoder rotations
moving = False # flag to indicate if motor is running

def move_down():
  GPIO.output(up_pin, False)
  GPIO.output(down_pin, True)

def move_up()
  GPIO.output(up_pin, True)
  GPIO.output(down_pin, False)

def stop()
  GPIO.output(up_pin, False)
  GPIO.output(down_pin, False)

# callback function called each encoder pulse
# stops motor if position reached, 
# otherwise increments or decrements position counter
def callback1(encoder_pin):
  if direction == 1:
    if current >= goal:
      stop()
      moving = False
    else:
      current += 1:
  elif direction == -1:
    if current <= goal:
      stop()
      moving = False
    else:
      current -= 1:

def selectDir():
  i = 0
  flag = False
  if current == goal:
    i = 0
    stop()
  elif current < goal:
    i = 1
    move_down()
    flag = True
  elif current > goal:
    i = -1
    move_up()
    flag = True
  moving = flag
  return i

def main():
  while True:
    if not moving:
      # read goal value from user input
      # replace this later with real implementation
      goal = input("set goal:\n")
      direction = selectDir()

main()

