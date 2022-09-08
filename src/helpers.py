from esp32 import hall_sensor, raw_temperature
from machine import RTC

    
def map_range(value, in_min, in_max, out_min, out_max):
  return (value - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


def is_door_open():
    return hall_sensor() < 100


def get_temperature():
    return (raw_temperature() - 32.0) / 1.8


def do_connect(ssid, password):
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


def log(message):
    rtc = RTC()
    now = rtc.datetime()
    print(f'{now[0]}-{now[1]:02}-{now[2]:02}T{now[4]:02}:{now[5]:02}:{now[6]:02}Z {message}')
    
    