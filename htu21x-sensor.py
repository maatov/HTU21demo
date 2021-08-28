import machine
import time
import binascii
import utime
from machine import Timer

#tim = Timer(period=5000, mode=Timer.ONE_SHOT, callback=lambda t:print(1))
#tim.init(period=2000, mode=Timer.PERIODIC, callback=lambda t:print(2))

#setup on-board led
led = machine.Pin(25, machine.Pin.OUT)
led.high()

#setup I2C pins
sdaPIN=machine.Pin(12,machine.Pin.PULL_UP)
sclPIN=machine.Pin(13,machine.Pin.PULL_UP)
#sdaPIN=machine.Pin(12)
#sclPIN=machine.Pin(13)
#i2c = machine.I2C(0,sda=machine.Pin(12), scl=machine.Pin(13), freq=100000)
i2c = machine.I2C(0,sda=sdaPIN, scl=sclPIN, freq=100000)

#setup on-board temp.sensor
sens_temp = machine.ADC(4)
internal_temp_correction_pin = machine.ADC(0)

print('Scanning i2c bus')
devices = []
for i in range(5):
    #max 5times (my HTU doesn't respond to this scan)
    devices = i2c.scan()
    print('devices found ', len(devices), devices)
    if len(devices)==0:
        sdaPIN.off()
        sclPIN.off()
        i2c = machine.I2C(0,sda=sdaPIN, scl=sclPIN, freq=100000)
        time.sleep(1.0)
    else:
        break;

for device in devices:
    print("Decimal address: ",device," | Hexa address: ",hex(device))

print(dir(i2c))

"""
Command Code Comment
Trigger Temperature Measurement 0xE3 Hold master
Trigger Humidity Measurement 0xE5 Hold master
Trigger Temperature Measurement 0xF3 No Hold master
Trigger Humidity Measurement 0xF5 No Hold master
Write user register 0xE6
Read user register 0xE7
Soft Reset 0xFE
"""


#0x40 should be mine HTU21 sensor
def restart(htu_i2c):
    #reset
    while True:
        try:
            time.sleep(0.33)
            rv = htu_i2c.writeto_mem(0x40,0xFE,b"")
            print('reset',rv)
            time.sleep(1.5)
            break
        except OSError as e:
            print(type(e),e)
            continue
    return

def read_user_register(htu_i2c):
    try:
        rv = htu_i2c.readfrom_mem(0x40,0xe7,1)
        if len(rv)==1:
            print('user register: ', rv)
        return rv
    except OSError as ose:
        print(type(ose),ose)
    return 0xFF
    
#restart module
restart(i2c)

#user reg
UserReg = read_user_register(i2c)
rtc = machine.RTC()

#def main loop
def mainloop(alarmnfo=None):
    print('mainloop',alarmnfo)
    led.high()
    while True:
        try:
            #tt = time.localtime()
            #timeval = "{}/{}/{} {}:{}:{}".format(tt[0],tt[1],tt[2],tt[3],tt[4],tt[5])
            timeval = ""
            print(rtc.datetime())
            
            #set temp capture (hold master mode)
            rv = i2c.readfrom_mem(0x40,0xE3,3)
            #print('TMP sens.value',binascii.hexlify(rv))
            #print('status',hex(rv[1] & 0x03))
            #print('value', rv[0] )
            #conversion into 14-bit value
            sen_temp_val = ((rv[0] << 8) | ((rv[1] >> 2) << 2))
            #print('sensorv value 16b (status bits cleared)',sen_temp_val)
            Temp = (-46.85 + 175.72*sen_temp_val/(1<<16))
            print(timeval, 'Temperature [celsius]:', Temp)
            time.sleep(0.0)

            #read humidity
            rv = i2c.readfrom_mem(0x40,0xE5,3)
            #print('RH sens.value',binascii.hexlify(rv))
            #print('status',hex(rv[1] & 0x03))
            sen_hum_val = ((rv[0] <<8) | ((rv[1]>>2)<<2))
            #print('sensor hum. value ', sen_hum_val)
            RH = (-6.0 + 125.0*(sen_hum_val / (1<<16)))
            print(timeval,"relative humidity:",RH)
            time.sleep(0.0)
            
            if False:
                #read temp from internal temp.sensor (vobec nefunguje dobre...)
                sensval = sens_temp.read_u16()
                correction = internal_temp_correction_pin.read_u16()
                #sensval -= correction
                #print('correction ', correction, ' termometer val ', sensval)
                #print('ADC reading',sensval)
                voltage = sensval * 3.3 / (1<<16)
                #print('voltage',voltage)
                #voltage -= 0.01
                TempInternal = 27 - (voltage - 0.706) / 0.001721
                print('temp. from internal ', TempInternal)
            
            break
        except OSError as ose:
            print(type(ose),ose)
            print('restarting module')
            restart(i2c)
            continue
        finally:
            pass
    led.low()

tim = Timer(period=1*1000, mode=Timer.ONE_SHOT, callback=mainloop)
tim2 = Timer()
tim2.init(period=60*1000, mode=Timer.PERIODIC, callback=mainloop)

#ends in endless loop
print('entering endless loop')

