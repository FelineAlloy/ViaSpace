import machine
import sdcard
import uos
import time
import adafruit_bno055
import adafruit_bmp280
from machine import UART, Pin

##### outiside IMU sensor #####

oIMU_SDA = 2
oIMU_SCL = 3

oIMU_i2c = machine.I2C(0,
                      scl=machine.Pin(IMU_SCL),
                      sda=machine.Pin(IMU_SDA),
                      freq=400000)
oIMU_sensor = adafruit_bno055.BNO055_I2C(i2c)

##### inside IMU sensor #####

iIMU_SDA = 4
iIMU_SCL = 5

iIMU_i2c = machine.I2C(0,
                      scl=machine.Pin(IMU_SCL),
                      sda=machine.Pin(IMU_SDA),
                      freq=400000)
iIMU_sensor = adafruit_bno055.BNO055_I2C(i2c)
 
##### Temp and pressure sensor #####

temp_SDA = 0
temp_SCL = 1

temp_i2c = machine.I2C(0,
                      scl=machine.Pin(IMU_SCL),
                      sda=machine.Pin(IMU_SDA),
                      freq=400000)
temp_sensor = adafruit_bmp280.Adafruit_BMP280_I2C(i2c)

# change this to match the location's pressure (hPa) at sea level
temp_sensor.sea_level_pressure = 1013.25

##### Radio #####

UART_TX = 12
UART_RX = 13

uart = UART(1, 9600, rx=Pin(UART_RX), tx=Pin(UART_TX) , bits=8, parity=None, stop=1, timeout=1)

##### SD card #####

# Assign chip select (CS) pin (and start it high)
cs = machine.Pin(9, machine.Pin.OUT)

# Intialize SPI peripheral (start with 1 MHz)
spi = machine.SPI(1,
                  baudrate=1000000,
                  polarity=0,
                  phase=0,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=machine.Pin(10),
                  mosi=machine.Pin(11),
                  miso=machine.Pin(8))

# Initialize SD card
sd = sdcard.SDCard(spi, cs)

# Mount filesystem
vfs = uos.VfsFat(sd)
uos.mount(vfs, "/sd")


##### Main #####

cnt = 0
while True:
    with open("/sd/data.txt", "a") as file:
        file.write("Reading nr: %" % cnt)
        
        file.write("\nAccelerometer outside: %0.4f m/s^2" % oIMU_sensor.acceleration)
        file.write("Magnetometer outside: %0.4f microteslas" % oIMU_sensor.magnetic)
        file.write("Gyroscope outside: %0.4f rad/sec" % oIMU_sensor.gyro)
        file.write("Euler angle outside: %0.4f" % oIMU_sensor.euler)
        file.write("Quaternion outside: %0.4f" % oIMU_sensor.quaternion)
        file.write("Linear acceleration outside: %0.4f m/s^2" % oIMU_sensor.linear_acceleration)
        file.write("Gravity outside: %0.4f m/s^2" % oIMU_sensor.gravity)
        
        file.write("\nAccelerometer inside: %0.4f m/s^2" % oIMU_sensor.acceleration)
        file.write("Magnetometer inside: %0.4f microteslas" % oIMU_sensor.magnetic)
        file.write("Gyroscope inside: %0.4f rad/sec" % oIMU_sensor.gyro)
        file.write("Euler angle inside: %0.4f" % oIMU_sensor.euler)
        file.write("Quaternion inside: %0.4f" % oIMU_sensor.quaternion)
        file.write("Linear acceleration inside: %0.4f m/s^2" % oIMU_sensor.linear_acceleration)
        file.write("Gravity inside: %0.4f m/s^2" % oIMU_sensor.gravity)
        
        file.write("\nTemperature: %0.4f C" % temp_sensor.temperature)
        file.write("Pressure: %0.4f hPa" % temp_sensor.pressure)
        file.write("Altitude = %0.4f meters" % temp_sensor.altitude)
        
        file.write("\n")
    
    cnt += 1
    time.sleep(1)
