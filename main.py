from rov import ROV
import RPi.GPIO as GPIO   


GPIO.setmode(GPIO.BCM)
config = {
    'gps':{
        'name': "GT-U7",
        'port': "/dev/ttyAMA0",
        'baudrate': 9600,
        'timeout': 0.5,
        'wait_for_init': False
    },
    'imu':{
        'PWR_MGMT_1': 0x6B,
        'SMPLRT_DIV': 0x19,
        'CONFIG' : 0x1A,
        'GYRO_CONFIG': 0x1B,
        'INT_ENABLE': 0x38,
        'DEV_ADDR': 0x68,
        'bus': '1'
    },
    'motor1':{
        'EN':24,
        'IN1':22,
        'IN2':23
        },
    'motor2':{
        'EN':27,
        'IN1':25,
        'IN2':26
        }
}

rov = ROV(config)

rov.start()
rov.control_motors(50,50)
rov.control_motors(75,25)
rov.control_motors(25,75)