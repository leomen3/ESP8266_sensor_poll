 ###########################################################################
# 0) turn off WiFi
# 1) poll sensors
# 2) report data to RPi via JSON text over Serial/USB



# pinout
# D0 - Digital Humidity Temperature
# A0 (ADC) - Gas Detector A0
##########################################################################

import machine
import utime
import network
import esp
import ujson
from machine import UART
from dht import DHT11
import ads1x15
from machine import I2C, Pin, Timer
DEBUG = False

CARBON_MONOXIDE_ADC_THRESH = 3000
GAS_ALL_ADC_THRESH = 4500

#Logging constants
LOG_DELAY = 5*60    # minimal number of seconds to wait/
                            # between subsequent sensor Logging
SAMPLE_DELAY = 10

#Define MQTT topics:
TEMPERATURE_CHIPA = "/sensor/Chipa/temperature"
HUMIDITY_CHIPA = "/sensor/Chipa/humidity"
CARBON_MONOXIDE_CHIPA = "/sensor/Chipa/CO"
GAS_ALL_CHIPA = "/sensor/Chipa/All_Gas"
SENSOR_PREFIX = "SENSOR:"
EOL = "\n"

#ADC channels
MQ7_CHANNEL = 0
MQ135_CHANNEL = 1


#Init I2C multi-channel ADC to poll gas sensors
i2c = I2C(scl=Pin(5), sda=Pin(4), freq=400000)
addr = 72
ads = ads1x15.ADS1115(i2c, addr)


def toggleGPIO(p):
    p.value(not p.value())


def GPIO_On(p):
    p.value(1)


def GPIO_Off(p):
    p.value(0)


def getHumidityTemp():
    try:
        dhtSensor.measure()
        temper = dhtSensor.temperature()
        humid = dhtSensor.humidity()
    except:
        (temper, humid) = (None, None)

    return temper, humid


def pushSample(sample, topic):
    """ Send the sensor value over UART"""
    buf=ujson.dumps((SENSOR_PREFIX,sample,topic))
    buf = buf+EOL
    uart.write(buf)
    utime.sleep(1)

#Generic Init
print ("Initializing...")
rtc = machine.RTC()
uart = UART(0, 115200)                         # init with given baudrate
uart.init(115200, bits=8, parity=None, stop=1) # init with given parameters
dhtSensor = DHT11(machine.Pin(16))  # D0 pin on WEMOS board. DHT signal pin


while True:
    utime.sleep(SAMPLE_DELAY)  # Sleep at least one second between each measurement

    # sample the temperature and humidity sensor
    temper, humid = getHumidityTemp()
    print ("Temperature[degC]: ", temper, ", Humidity[%]: ", humid, 
    "time[s]: ")
    pushSample(temper, TEMPERATURE_CHIPA)
    pushSample(humid, HUMIDITY_CHIPA)

    #sample gas sensirs
    gasC0 = ads.read(MQ7_CHANNEL)
    gasAll = ads.read(MQ135_CHANNEL)
    print("Analog CO gas value: ",gasC0)
    print("Analog All gas value: ",gasAll)
    alertCO = gasC0 > CARBON_MONOXIDE_ADC_THRESH
    alertAll = gasAll > GAS_ALL_ADC_THRESH
    if alertCO:
        print("ALERT!!!!! CARBON MONOXIDE LEVEL ABOVE THRESHOLD")

    if alertAll:
        print("ALERT!!!!! DANGEROUS GAS LEVEL ABOVE THRESHOLD")

    pushSample(gasC0,CARBON_MONOXIDE_CHIPA)
    pushSample(gasAll,GAS_ALL_CHIPA)