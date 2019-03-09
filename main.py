 ###########################################################################
# 0) turn off WiFi
# 1) poll sensors
# 2) report data to RPi via JSON text over Serial/USB



# pinout
# D0 - PIR Motion Sensor
# D2 - Digital Humidity Temperature
# D1 - Sound Detector Dig0ut
# A0 (ADC) - Gas Detector A0
##########################################################################

import machine
import utime
import network
import esp
import ujson
from machine import UART
from dht import DHT11
DEBUG = False

CARBON_MONOXIDE_ADC_THRESH = 300

#Logging constants
LOG_DELAY = 5*60    # minimal number of seconds to wait/
                            # between subsequent sensor Logging
SAMPLE_DELAY = 2

MOTION_LOG_DELAY = 5*60   # minimal number of seconds to wait/
                            # between subsequent motion Logging

#Define MQTT topics:
TEMPERATURE_CHIPA = "/sensor/Chipa/temperature"
HUMIDITY_CHIPA = "/sensor/Chipa/humidity"
MOVEMENT_CHIPA = "/sensor/Chipa/motion"
CARBON_MONOXIDE_CHIPA = "/sensor/Chipa/CO"
SOUND_LEVEL_CHIPA = "/sensor/Chipa/sound"
SENSOR_PREFIX = "SENSOR:"
EOL = "\n"

#global variable, change only in the appropriate function
soundDetected = False
motionDetected = False
lastCOLogTime = 0
lastMotionLogTime = 0
lastTemperatureLogTime = 0
lastMotionTime = 0
lastSoundTime = 0


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


def pir_pin_interrupt():
    global motionDetected, lastMotionTime
    lastMotionTime = utime.time()
    print("MOTION DETECTED")
    motionDetected = True


def sound_pin_interrupt():
    global soundDetected, lastSoundTime
    lastSoundTime = utime.time()
    print("SOUND DETECTED")
    soundDetected = True


def handleDHT(temper, humid,currentTime):
    global lastTemperatureLogTime
    print ("Temperature[degC]: ", temper, ", Humidity[%]: ", humid, 
           "time[s]: ", currentTime)
    if (lastTemperatureLogTime == 0) or \
            currentTime - lastTemperatureLogTime > LOG_DELAY:
        print ("LOGGING temperature and humidity at: ", currentTime)
        lastTemperatureLogTime = currentTime
        pushSample(temper,TEMPERATURE_CHIPA)
        pushSample(humid,HUMIDITY_CHIPA)


def handlePIR(currentTime):
    global lastMotionTime, lastMotionLogTime
    print ("Movement detected at: ", lastMotionTime)
    if (lastMotionLogTime == 0) or \
            (currentTime - lastMotionLogTime) > LOG_DELAY:
        print ("LOGGING motion at: ", currentTime)
        lastMotionLogTime = currentTime
        if not DEBUG:
            pushSample(lastMotionTime, "/sensor/Chipa/motion")

def handleCO(gasA0,gasDetected,currentTime):
    global lastCOLogTime
    print("Analog gas value: ",gasA0, " digital gas threshold value: ", gasDetected)
    alert = gasA0 > CARBON_MONOXIDE_ADC_THRESH
    if alert:
        print("ALERT!!!!! CARBON MONOXIDE LEVEL ABOVE THRESHOLD")

    if alert or (lastCOLogTime == 0) or \
            (currentTime - lastCOLogTime) > LOG_DELAY:
        print ("LOGGING Carbon Monoxidde at: ", currentTime)
        lastCOLogTime = currentTime
        pushSample(gasA0,CARBON_MONOXIDE_CHIPA)


#Generic Init
print ("Initializing...")
rtc = machine.RTC()
uart = UART(0, 115200)                         # init with given baudrate
uart.init(115200, bits=8, parity=None, stop=1) # init with given parameters

dhtSensor = DHT11(machine.Pin(4))  # D2 pin on NodeMCU board. DHT signal pin
pirSig = machine.Pin(16, machine.Pin.IN)          # D0 pin on NodeMCU. PIR signal pin

#Carbon Monoxide sensor
#Init ADC
adc = machine.ADC(0)
#Threshold pin
gasD0 = machine.Pin(1)



#sound sensor
# soundSig = machine.Pin(5, machine.Pin.IN)    
# soundSig.irq(handler=lambda p: sound_pin_interrupt(), trigger=machine.Pin.IRQ_RISING)

print("starting telemetry")
for i in range(1000):
    #a = str(i)
    buf=ujson.dumps((SENSOR_PREFIX,SOUND_LEVEL_CHIPA,i))
    buf = buf+EOL
    uart.write(buf)
    utime.sleep(1)
    #uart.write(EOL)



##while True:
''' ###old code. uncomment when done
    if DEBUG:
        #loop reading and printing all sensor inputs
        utime.sleep(3)  # sleep for 3 seconds
        print("reading Temp&Humid")
        temper, humid = getHumidityTemp()
        print ("Temperature[degC]: ", temper, ", Humidity[%]: ", humid)
        print("reading Gas analog and threashold")
        gasA0 = adc.read()
        gasDetected = gasD0.value() == 1
        print("Analog gas value: ",gasA0, " digital gas threshold value: ", gasDetected)
        if gasA0 > CARBON_MONOXIDE_ADC_THRESH:
            print("ALERT!!!!! CARBON MONOXIDE LEVEL ABOVE THRESHOLD")
        if motionDetected:   
            motionDetected = False
            handlePIR()

    else:
        #gotTime, curr_tm = getDateTime()  # get time 
        utime.sleep(SAMPLE_DELAY)  # Sleep at least one second between each measurement
        try:
            currentTime = utime.time()
        except:
            print('getting updated time from internet failed')

        # sample the temperature and humidity sensor
        temper, humid = getHumidityTemp()
        handleDHT(temper, humid,currentTime)
        
        #sample carbon monoxide
        gasA0 = adc.read()
        gasDetected = gasD0.value() == 1
        handleCO(gasA0,gasDetected,currentTime)

        #sample sound level 
        soundHigh = (soundSig == 1)

        # if detected motion on PIR sensor
        if motionDetected:   
            motionDetected = False
            handlePIR(currentTime)

        if soundDetected:
            soundDetected = False
            handleSound(currentTime) '''