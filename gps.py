import RPi.GPIO as GPIO
import serial
import string
import pynmea2
from time import sleep

# setup GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

PORT = "/dev/ttyAMA0"
BAUDRATE= 9600
TIMEOUT = 0.5

ser = serial.Serial(PORT,baudrate=BAUDRATE)

print(f"Reading GPS data from GT-U7 on {PORT}\n\t-Baud rate = {BAUDRATE}\n\t-Timeout = {TIMEOUT}")

while True:
    dataout = pynmea2.NMEAStreamReader()
    newdata = ser.readline()

    try:
        tag = newdata[0:6].decode('ascii')
    except Exception as e:
        print(e)
        continue

    decoded_newdata = newdata.decode('ascii')

    if tag != "$GPRMC":
        continue

    newmsg=pynmea2.parse(decoded_newdata)
    lat=round(newmsg.latitude,3)
    lng=round(newmsg.longitude,3)

    coordinates = (lat,lng)

    print(coordinates)
