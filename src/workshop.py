from esp32 import hall_sensor
from machine import Pin
from time import sleep

from helpers import is_door_open


if __name__ == '__main__':
    # init door
    door_state = is_door_open()
    
    # init led
    led = Pin(32, Pin.OUT, Pin.PULL_DOWN)
    led.value(door_state)

    while True:
        # check state of the door
        if door_state != is_door_open():
            door_state = not door_state  # is_door_open()
            led.value(door_state)
            
            if door_state == True:
                print('>> Door has been opened.')
            else:
                print('>> Door has been closed.')
        
        sleep(0.5)