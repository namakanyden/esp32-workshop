from machine import Pin
from time import sleep
from umqtt.robust import MQTTClient

from helpers import is_door_open, was_touch, get_temperature, do_connect, get_current_weather


if __name__ == '__main__':
    # connect to wifi
    do_connect('ssid', 'password')
    
    # connecting to mqtt broker
    client = MQTTClient('moj-esp32-mikrokontroler', 'broker.hivemq.com')
    client.connect()
    
    # init door
    door_state = is_door_open()
    
    # init led
    led = Pin(21, Pin.OUT, Pin.PULL_DOWN)
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
                led.on()
            else:
                print('>> Door has been closed.')
                led.off()
                
        # check the state of touch pad
        if touch_state != was_touch(14):
            touch_state = not touch_state  # was_touch(tp)
            
            if touch_state is True:
                print('>> Touch detected.')
                data = get_current_weather('bratislava')
                print(f'>> Aktuálna teplota v meste {data["location"]} ({data["country"]}) je {data["temp"]}°C')
                
        # print temperature
        print(f'{get_temperature()}°C')
        client.publish('pycon/sk/2022/mirek/temp', str(get_temperature()))
        
        sleep(0.5)

