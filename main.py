from threading import Thread
from sensors import GPS,IMU

gps = GPS("GT-U7", "/dev/ttyAMA0", 9600,0.5,False)
imu = IMU(0x6B,0x19,0x1A,0x1B,0x38,0x68)

print(gps,imu)

while True:
    new_pos = gps.measure()
    new_inertial_state = imu.measure()

    print(new_pos,new_inertial_state)