from esp32 import hall_sensor, raw_temperature
from machine import RTC, TouchPad, Pin
import urequests


APPID = '9e547051a2a00f2bf3e17a160063002d'
TOUCH_TRESHOLD = 50
HALL_TRESHOLD = 100


def is_door_open():
    """
    Returns True, if door is open, or False otherwise.
    """
    return hall_sensor() < HALL_TRESHOLD


def get_temperature():
    """
    Returns inner temperature converted to Celsius as float.
    """
    return (raw_temperature() - 32.0) / 1.8


def was_touch(pin):
    """
    Returns True, if touch on given pin was detected, or False otherwise.
    """
    touchpad = TouchPad(Pin(pin))
    return touchpad.read() < TOUCH_TRESHOLD


def do_connect(ssid, password):
    """
    Connects to wifi and after successfull connection will synchronize the time over NTP.
    """
    import network
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        log('Connecting to network...')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    log(f'Network config: {wlan.ifconfig()}')

    # set time and date with NTP
    import ntptime
    log('Synchronizing time...')
    ntptime.settime()
    rtc = RTC()
    now = rtc.datetime()
    log(f'Current time: {now[0]}-{now[1]:02}-{now[2]:02}T{now[4]:02}:{now[5]:02}:{now[6]:02}Z')

    return wlan


def get_current_weather(location):
    """
    Gets the current weather conditions for given location.
    """
    url = f'http://api.openweathermap.org/data/2.5/weather?units=metric&q={location}&appid={APPID}'
    response = urequests.get(url)
    data = response.json()
    response.close()

    return {
        'location': data['name'],
        'country': data['sys']['country'],
        'temp': data['main']['temp']
    }


def log(message):
    rtc = RTC()
    now = rtc.datetime()
    print(f'{now[0]}-{now[1]:02}-{now[2]:02}T{now[4]:02}:{now[5]:02}:{now[6]:02}Z {message}')


def map_range(value, in_min, in_max, out_min, out_max):
  return (value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min
