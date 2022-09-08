from esp32 import hall_sensor
from machine import Pin
from time import sleep

from helpers import is_door_open, was_touch, get_temperature


if __name__ == '__main__':
    # init door
    door_state = is_door_open()
    
    # init led
    led = Pin(32, Pin.OUT, Pin.PULL_DOWN)
    led.value(door_state)
    
    # init touchpad
    touch_state = was_touch(14)

    while True:
        # check state of the door
        if door_state != is_door_open():
            door_state = not door_state  # is_door_open()
            led.value(door_state)
            
            if door_state == True:
                print('>> Door has been opened.')
            else:
                print('>> Door has been closed.')
                
        # check the state of touch pad
        if touch_state != was_touch(14):
            touch_state = not touch_state  # was_touch(tp)
            
            if touch_state is True:
                print('>> Touch detected.')
                
        # print temperature
        print(f'{get_temperature()}°C')
        
        sleep(0.5)