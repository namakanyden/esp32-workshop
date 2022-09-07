# ESP32 MicroPython Workshop

![ESP32 Running MicroPython](https://www.usinainfo.com.br/blog/wp-content/uploads/2019/06/IMG_2897-ok-1920x1080.jpg)

Mikrokontrolér _ESP32_ je cenovo dostupné a namakané zariadenie vhodné pre oblasť _IoT_ vybavené _WiFi_ a _Bluetooth LE_. Čo je však úplne fantastické, má dostatok pamäte na to, aby ste do neho nahrali firmvér s jazykom _MicroPython_. Na tomto workshope si spolu vytvoríme jednoduché _IoT_ riešenie, na ktorom ukážeme silu mikrokontroléra _ESP32_ a jednoduchosť jeho programovania vďaka jazyku _MicroPython_.

## Ciele

1. 

## Krok 1. Čo budeme potrebovať?

Ešte predtým, ako sa pustíme do tvorby aplikácie, si pripravíme prostredie pre prácu. Konkrétne budeme potrebovať:

- editor kódu [Thonny](https://thonny.org) (alebo ľubovoľný iný editor kódu jazyka Python)
- dosku s mikrokontrolérom _ESP32_, zapojenie podľa schémy a USB kábel na prepojenie dosky s počítačom

Ak používate OS Windows a chcete pracovať s mikrokontrolérom _ESP32_, musíte si nainštalovať ešte ovládač pre *CP210x USB to UART Bridge* napríklad [odtiaľto](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers?tab=downloads).

## Krok 2. Predstavenie mikrokontroléra ESP32

![Rozloženie pinov na doske s mikrokontrolérom ESP32](https://raw.githubusercontent.com/AchimPieters/esp32-homekit-camera/master/Images/ESP32-38%20PIN-DEVBOARD.png)

Samotný mikrokontrolér obsahuje tieto senzory:

* hallova sonda/senzor
* kapacitný dotykový senzor
* senzor vnútornej (pracovnej) teploty

A presne na prácu s týmito senzormi sa pozrieme počas tohto workshopu. Vytvoríme si jednoduchý scenár, pomocou ktorého budeme monitorovať teplotu v miestnosti a vstup do nej:

* Teplomer bude predstavovať aktuálnu teplotu v miestnosti. Tú budeme v (ne)pravidelných intervaloch posielať pomocou protokolu MQTT ďalšej službe.
* Hallovu sondu budeme používať ako snímač otvorených/zatvorených dverí. Na tento účel nám pomôže ešte externý magnet. Priložením magnetu k hallovej sonde budeme simulovať zatvorenie dverí. Oddialením magneta sa dvere otvoria. A okrem toho - keď budú dvere otvorené, rozsvieti sa LED dióda. Keď sa naopak dvere zatvoria, LED dióda zhasne.
* Po zistení dotyku na kapacitnom dotykovom senzore stiahneme aktuálne počasie v zvolenej lokalite. Aktuálnu teplotu vypíšeme do konzoly (na sériovú linku).

Schéma zapojenia pre tento scenár vyzerá takto:

**Upozornenie:** Vo svojich zapojeniach **NIKDY** nezapájajte LED diódy bez ochranného rezistora!!!

## Krok 2. Prvé kroky

Aby ste mohli používať editor Thonny na programovanie mikrokontrolérov _ESP32_, potrebujete si skontrolovať nastavenie interpretéra. To vykonáte v menu `Run > Interpreter`. Tu si zo zoznamu vyberte položku `MicroPython (ESP32)`. Ako posledné si nastavte port, na ktorom je vaše zariadenie pripojené. Tu môžete vybrať voľbu pre automatickú detekciu, ktorá tento problém vyrieši za vás.

Po pripojení zariadenia a správnej konfigurácii sa vám v okne _Shell_ zobrazí REPL režim jazyka _MicroPython_ na mikrokontroléri _ESP32_. Komunikácia ide cez sériovú linku.

Ak chcete vedieť, aké moduly máte k dispozícii, môžete si ich všetky nechať vypísať príkazom `help('modules')`:

```python
>>> help('modules')
__main__          inisetup          ucollections      urandom
_boot             math              ucryptolib        ure
_onewire          micropython       uctypes           urequests
_thread           neopixel          uerrno            uselect
_uasyncio         network           uhashlib          usocket
_webrepl          ntptime           uheapq            ussl
apa106            onewire           uio               ustruct
btree             uarray            ujson             usys
builtins          uasyncio/__init__ umachine          utime
cmath             uasyncio/core     umqtt/robust      utimeq
dht               uasyncio/event    umqtt/simple      uwebsocket
esp               uasyncio/funcs    uos               uzlib
esp32             uasyncio/lock     upip              webrepl
flashbdev         uasyncio/stream   upip_utarfile     webrepl_setup
framebuf          ubinascii         uplatform
gc                ubluetooth        upysh
Plus any modules on the filesystem
```

**Poznámka:** V zozname modulov si môžete všimnúť, že mnohé z nich majú v názve prefix s písmenom `u` (symbol pre _mikro_). Častokrát sa jedná o štandardný modul jazyka _Python_ upravený pre použitie v jazyku _MicroPython_. To napríklad znamená, že ak potrebujete pracovať s formátom _JSON_ a ste zvyknutí na prácu so štanradným modulom `json`, v jazyku _MicroPython_ máte k dispozícii jeho _"micro"_ verziu s názvom `ujson`. Interpreter jazyka _MicroPython_ tiež zvláda mnohé moduly importovať na základe ich názvov zo štandardnej knižnice. Takže miesto:

```python
import ujson
```

môžete modul importovať aj

```python
import json
```

## Krok 3. Blikanie LED diódou v režime REPL

LED diódu máme pripojenú ku pin-u s číslom _32_. Budeme ju používať na signalizáciu stavu dverí, čo znamená, že LED dióda bude svietiť vtedy, keď budú dvere otvorené a diódu zhasneme, keď sa dvere zatvoria. Pri predstavení práce s LED diódou si ukážeme silu, ktorú nám ponúka _REPL_ režim jazyka _MicroPython_.

Začneme tým, že z balíka `machine` importujeme triedu `Pin`:

```python
>>> from machine import Pin
```

Vytvoríme objekt triedy `Pin`, ktorý bude reprezentovať _LED_ diódu. Konštruktor triedy `Pin` má tieto parametre:

* `id` - povinný parameter, ktorý identifikuje pin
* `mode` - režim pin-u, ktorý môže byť
  * `Pin.IN` - pin pre vstup, a
  * `Pin.OUT` - pin pre výstup
* `pull` - špecifikuje, či má mať pin pripojený (slabý) pull rezistor a jeho hodnota môže byť
  * `Pin.PULL_UP` - zapnutý pull up rezistor
  * `Pin.PULL_DOWN` - zapnutý pull down rezistor
  * `None` - žiadny pull up alebo pull down rezistor


**Poznámka:** Trieda `Pin` má výrazne viac možností konfigurácie a práce. Pre podrobnejšie informácie sa pozrite do [dokumentácie](https://docs.micropython.org/en/latest/library/machine.Pin.html).

Výsledný kód pre vytvorenie výstupného pin-u na pin-e č. _32_ s pripojeným pull-down rezistorom bude vyzerať nasledovne:

```python
>>> led = Pin(32, Pin.OUT, Pin.PULL_DOWN)
```

Pre prácu s diódou, resp. všeobecne s digitálnym pin-om, máme k dispozícii niekoľko metód. Ak chceme získať aktuálny stav pin-u, zavoláme metódu `value()`:

```python
>>> led.value()
0
```

**Poznámka:** Pin sme pri jeho vytváraní nijako neinicializovali, takže hodnota na pin-e v skutočnosti nemusí byť `0`.

Pomocou metódy `value()` však môžeme aj hodnotu na pin zapísať. Ak chceme _LED_ diódu pripojenú na tento pin zasvietiť, do metódy `value()` zapíšeme ako parameter hodnotu `1`:

```python
>>> led.value(1)
```

Ak následne chceme _LED_ diódu zhasnúť, do metódy `value()` vložíme ako parameter hodnotu `0`:

```python
>>> led.value(0)
```

Trieda `Pin` však umožňuje nad svojimi inštanciami volať aj priamo metódu `on()` na zasvietenie (alternatíva pre `value(1)`) a metódu `off()` pre zhasnutie (alternatíva pre `value(0)`).

```python
>>> led.on()
>>> led.off()
```

## Krok 4. Hallova sonda/senzor

Mikrokontrolér _ESP32_ je vybavený _hallovým senzorom_, ktorý detekuje prítomnosť magnetického poľa. Jeho sila je vyjadrená hodnotou, ktorú tento senzor vráti - čím je pole väčšie/silnejšie, tým je aj hodnota senzora vyššia.

Silu magnetického senzoru v okolí mikrokontroléra _ESP32_ prečítame zavolaním funkcie `hall_sensor()`, ktorá sa nachádza v module `esp32`:

```python
>>> from esp32 import hall_sensor()
>>> hall_sensor()
44
```

Platí, že ak v okolí Hallovho senzora nie je magnetické pole, bude vrátená hodnota nízka. Ak sa však v okolí bude magnetické pole nachádzať, senzor bude vracať vysoké hodnoty.

Správanie senzora si môžeme otestovať veľmi jednoducho - vytvoríme nekonečnú slučku a v pravidelných intervaloch budeme odčítavať hodnoty z _hallovho senzora_. Túto slučku môžeme uložiť do súboru `playground.py`, ktorý budeme použiť na experimenty a výsledok môžeme zobrazovať pomocou _plotter_-a.

```python
from esp32 import hall_sensor
from time import sleep

while True:
    print(hall_sensor())
    sleep(0.5)
```

![Vykreslenie priebehu hodnôt z hallovho senzora](images/hall.sensor-plotter.png)

V našom scenári budeme _hallov senzor_ používať ako senzor otvorených dverí. Vytvoríme preto funkciu `is_door_open()`, ktorá nám na základe veľkosti magnetického poľa vráti aktuálny stav dveri:

* vráti hodnotu `True`, ak sú dvere otvorené (v tomto prípade senzor vráti nízku hodnotu), alebo
* vráti hodnotu `False`, ak sú dvere zatvorené (v tomto prípade senzor vráti vysokú hodnotu).

Funkciu zapíšeme do súboru s menom `workshop.py`, v ktorom budeme náš scenár programovať:

```python
from esp32 import hall_sensor

def is_door_open():
    return hall_sensor() < 150
```

**Poznámka:** Hodnotu, pri ktorej budeme považovať dvere za zatvorené, si musí každý nastaviť sám, nakoľko citlivosť senzora a sila použitého magnetu môže byť na rozličných zariadeniach iná.

## Krok 5. Superloop

Je načase dať veci dokopy a vytvoriť senzor otvorených dverí využívajúci _hallov senzor_ a LED diódu. Aktualizujeme teda obsah súboru `workshop.py`.

```python
from esp32 import hall_sensor
from machine import Pin
from time import sleep


def is_door_open():
    return hall_sensor() < 100


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
                print('Door has been opened.')
            else:
                print('Door has been closed.')
        
        sleep(0.5)
```

## Krok 6. Vnútorný senzor teploty

Mikrokontrolér _ESP32_ obsahuje aj vnútorný senzor teploty. Ten síce nemeria vonkajšiu teplotu okolia, ale pracovnú teplotu mikrokontroléra. Za istých okolností ho je však možné použiť aj ako senzor teploty prostredia. To je však možné len v tom prípade, ak okolitá teplota je vyššia ako pracovná teplota mikrokontroléra. A tá je v prípade mikrokontroléra _ESP32_ častokrát viac ako _40°C_.

Aktuálnu pracovnú teplotu mikrokontroléra vieme odmerať zavolaním funkcie `raw_temperature()`, ktorá sa nachádza v module `esp32`:

```python
>>> from esp32 import raw_temperature
>>> raw_temperature()
117
```

Hodnota, ktorú funkcia vráti je vo _Fahrenheitoch_. Ak ju chceme v stupňoch celzia, musíme ju skonvertovať podľa vzťahu:

```python
temp_c = value  - 32 .0 / 1.8
```

Aj hodnoty teplomera môžeme vizualizovať pomocou plotter-a vypisovaním nameraných hodnôt v nekonečnej slučke. Nakoľko však pracovná teplota bude prevyšovať _40°C_, jej zmenu budeme dosahovať ťažšie. Mikrokontrolér napr. môžete skúsiť priložiť ku vetráku vášho laptopu.

Pre potreby nášho scenára si vytvoríme samostatnú funkciu `get_temperature()`, ktorá nám vždy vráti hodnotu teploty už prevedenú na stupne celzia:

```python
from esp32 import raw_temperature

def get_temperature():
    return (raw_temperature() - 32.0) / 1.8
```

## Krok 7. Supperloop Update I.

Našu hlavnú slučku môžeme aktualizovať vypísaním pracovnej teploty pri každej aktualizácii:

```python
while True:
    ...
    print(f'{get_temperature()}°C')
	sleep(0.5)
```

## Krok x. Kapacitný senzor dotyku

Mikrokontrolér _ESP32_ obsahuje _10_ kapacitných dotykových pinov (na obrázku s rozložením pin-ov sú označené ako `TOUCH0` až `TOUCH9`). Tieto piny dokážu detekovať zmeny vo všetkom, čo obsahuje elektrický náboj. Napríklad aj v ľudskej koži. To znamená, že tieto piny dokážu detekovať to, keď sa ich dotýkate napríklad prstom.

Ak chceme pracovať s kapacitným senzorom dotyku na týchto pin-och, musíme vytvoriť inštanciu triedy `TouchPad`, ktorá sa nachádza v module `machine`. Jej jediným parametrom je objekt typu `Pin`, ktorým identifikujeme konkrétny pin. Zmenu kapacity následne zistíme zavolaním metódy `.read()` nad vytvoreným objektom typu `TouchPad` .

```python
>>> from machine import Pin, TouchPad
>>> tp = TouchPad(Pin(14))
>>> tp.read()
647
```

V prípade, že bol detekovaný dotyk, hodnota, ktorú senzor vráti, bude nízka (rádovo jednotky až desiatky). V prípade, že dotyk detekovaný nebol, hodnota senzora bude vysoká (rádovo stovky).

Pre potrebu nášho scenára si vytvoríme pomocnú funkciu `was_touch()`, ktorá vráti hodnotu `True`, ak bol detekovaný pohyb. V opačnom prípade vráti hodnotu `False`.

```python
def was_touch(touch_pad):
    return tp.read() < 50
```

## Krok x. 

```python
```





## Krok 8. Pripojenie do siete

Pre pripojenie k WiFi použijeme mierne modifikovanú verziu funkcie `do_connect()`, ktorú odporúčajú použiť aj autori dokumentácie jazyka _MicroPython_ pre mikrokontrolér _ESP32_. Pôvodnú funkciu môžete nájsť na [tejto adrese](http://docs.micropython.org/en/latest/esp32/quickref.html#networking). Zmeny sú len mierne:

* funkcia má parametere pre SSID a heslo do WiFi siete
* po úspešnom pripojení dôjde ku synchronizácii hodín pomocou protokolu NTP

```python
def do_connect(ssid, password):
    import network, ntptime, machine
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('>> Connecting to network...')
        wlan.connect(ssid, password)
        while not wlan.isconnected():
            pass
    print('>> Network config:', wlan.ifconfig())
    
    # set time and date with NTP
    print('>> Synchronizing time...')
    ntptime.settime()
    rtc = machine.RTC()
    now = rtc.datetime()
    print(f'>> Current time: {now[0]}-{now[1]:02}-{now[2]:02}T{now[4]:02}:{now[5]:02}:{now[6]:02}Z')
```

## Krok X. MQTT

HiveMQ Web Client: http://www.hivemq.com/demos/websocket-client/

```python
from umqtt.robust import MQTTClient

client = MQTTClient(ubinascii.hexlify(unique_id()), 'broker.hivemq.com')
client.connect()
    
client.publish('pycon/sk/2022/mirek/door', str(int(door_state)))
```

## Krok X. HTTP

```python
def get_current_weather(location):
    url = f'http://api.openweathermap.org/data/2.5/weather?units=metric&q={location}&appid={settings.APPID}'
    response = requests.get(url)
    return response.json()
```

## Ďalšie zdroje

* [MicroPython](https://micropython.org/) - domovská stránka projektu _MicroPython_
* [Quick reference for the ESP32](http://docs.micropython.org/en/latest/esp32/quickref.html) - Skrátená dokumentácia jazyka _MicroPython_ pre dosku s mikrokontrolérom _ESP32_.
* [Random Nerd Tutorials](https://randomnerdtutorials.com/) - Portál venovaný nie len programovaniu mikrokontroléra _ESP32_ v jazyku _MicroPython_.
* [ESP32 Labs](https://github.com/namakanyden/esp32-labs) - Niekoľko labov pre začiatočníkov v jazyku _MicroPython_ s mikrokontrolérom _ESP32_.

## TODO

* Zmeni sa pracovna teplota ak zmenim pracovnu frekvenciu mikrokontrolera?