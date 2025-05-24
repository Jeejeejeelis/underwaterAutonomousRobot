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
up_pin = 5 # brown, pwm input for motor controller, now run only at full speed
down_pin = 6 # red, now assuming true for up, false for down
encoder_pin = 24
up_limit = 19 # green (Assuming BCM 19 is up_limit)
down_limit = 26 # yellow (Assuming BCM 26 is down_limit)

# Global variable to store encoder ticks
encoder_ticks = 0

def encoder_tick_callback(channel):
  """Callback function executed on each encoder tick."""
  global encoder_ticks
  encoder_ticks += 1
  #print(f"Tick! Total: {encoder_ticks}")

def setup():
  print("setup")
  GPIO.setwarnings(False)
  GPIO.setmode(GPIO.BCM)
  GPIO.setup(up_pin, GPIO.OUT, initial=GPIO.LOW)
  GPIO.setup(down_pin, GPIO.OUT, initial=GPIO.LOW)
  GPIO.setup(encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(down_limit, GPIO.IN, pull_up_down=GPIO.PUD_UP)
  GPIO.setup(up_limit, GPIO.IN, pull_up_down=GPIO.PUD_UP)

  GPIO.add_event_detect(encoder_pin, GPIO.BOTH, callback=encoder_tick_callback, bouncetime=2) 
  GPIO.output(up_pin, False)
  GPIO.output(down_pin, False)
  time.sleep(1)
  print(f"Initial limit switch states: Down: {GPIO.input(down_limit)}, Up: {GPIO.input(up_limit)}")
  print("GPIO setup complete.")

def move_up():
  print("Moving up...")
  start_time = time.time()
  while True:
    if not GPIO.input(up_limit):
      print("Up limit hit, stopping motor.")
      GPIO.output(up_pin, False)
      GPIO.output(down_pin, False)
      return
    else:
      GPIO.output(up_pin, True)
      GPIO.output(down_pin, False)

def move_down():
  print("Moving down...")
  start_time = time.time()
  while True:
    if not GPIO.input(down_limit):
      print("Down limit hit, stopping motor.")
      GPIO.output(up_pin, False)
      GPIO.output(down_pin, False)
      return
    else:
      GPIO.output(up_pin,False)
      GPIO.output(down_pin, True)

def main():
  global encoder_ticks
  try:
    setup()
    print(f"Initial encoder ticks: {encoder_ticks}")

    # Move to the down limit first
    print("\nMoving to the down limit switch...")
    move_down()
    print(f"Ticks accumulated after moving to down limit: {encoder_ticks}")

    # Reset ticks to specifically measure the upward travel
    print("\nResetting tick count.")
    encoder_ticks = 0
    #time.sleep(1)

    # Move from down limit to up limit and count ticks
    print(f"Moving from down limit to up limit. Current ticks: {encoder_ticks}")
    move_up()
    print(f"\n--- Encoder ticks from down limit to up limit: {encoder_ticks} ---")

    

  except KeyboardInterrupt:
    print("\nKeyboard interrupt detected. Stopping motor and cleaning up.")
  except Exception as e:
    print(f"An error occurred: {e}")
  finally:
    print("Exiting program.")
    GPIO.output(up_pin, False) # Ensure motor is off
    GPIO.output(down_pin, False)
    GPIO.cleanup()

if __name__ == '__main__':
  main()
