import RPi.GPIO as GPIO
import smbus
from time import sleep

# setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# some MPU6050 registers and their addresses
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
DATA_REGISTERS = {
    'ACCEL_XOUT' : 0x3B,
    'ACCEL_YOUT' : 0x3D,
    'ACCEL_ZOUT' : 0x3F,
    'GYRO_XOUT' : 0x43,
    'GYRO_YOUT' : 0x45,
    'GYRO_ZOUT' : 0x47
    
    }
DEV_ADDR = 0x68


bus = smbus.SMBus(1)
    
    
def MPU_Init():

    # write to sample rate regiser
    bus.write_byte_data(DEV_ADDR, SMPLRT_DIV, 7)
    
    # write to power management regiser
    bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 1)
    
    # write to Configuration regiser
    bus.write_byte_data(DEV_ADDR, CONFIG, 0)
    
    # write to Gyro Configuration regiser
    bus.write_byte_data(DEV_ADDR, GYRO_CONFIG, 24)
    
    # write to Interrupt enable regiser
    bus.write_byte_data(DEV_ADDR, INT_ENABLE, 1)
    
def read_raw_data(addr):
    # Accel and gyro data are 16-bit
    
    high = bus.read_byte_data(DEV_ADDR, addr)
    low = bus.read_byte_data(DEV_ADDR, addr+1)
    
    # concatenate higher and lwoer value
    value = ((high << 8) | low)
    
    # to get signed value from MPU6050
    value = value - 65536 if value > 32768 else value
    
    
    return value
    

MPU_Init()

while True:
    try:
        measured_values = {reg:read_raw_data(addr) for reg, addr in DATA_REGISTERS.items()}
        for i, pair in enumerate(measured_values.items()):
            reg, value = pair
            
            if i < 3:
               measured_values[reg] = round(value / 16384.0, 3)
            else:
               measured_values[reg] = round(value / 131.0, 3
                                            )
        
        # gyro_x = read_raw_data(GYRO_XOUT) / 131.0
        # gyro_y = read_raw_data(GYRO_YOUT) / 131.0
        # gyro_z = read_raw_data(GYRO_ZOUT) / 131.0
        
        print(measured_values)
        sleep(1)
        
    except Exception as e:
        print(e)
    
    
    
    
    
    

