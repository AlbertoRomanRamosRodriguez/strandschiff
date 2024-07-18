import RPi.GPIO as GPIO          
from time import sleep

GPIO.setmode(GPIO.BCM)

# motor A
ena = 24
in1 =22
in2 = 23
GPIO.setup(ena,GPIO.OUT)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)

# motor B
enb = 27
in3 =25
in4 = 26
GPIO.setup(enb,GPIO.OUT)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)

# Configure PWM velocity
power = 25

v_a=GPIO.PWM(ena,1000) # velocity_a
v_a.start(power)

v_b=GPIO.PWM(enb,1000) # velocity_b
v_b.start(power)

def main(power:int) -> None:

   GPIO.output(in1,GPIO.LOW)
   GPIO.output(in2,GPIO.HIGH)
   GPIO.output(in3,GPIO.LOW)
   GPIO.output(in4,GPIO.HIGH)
   sleep(2)
   
   GPIO.output(in1,GPIO.HIGH)
   GPIO.output(in2,GPIO.LOW)
   GPIO.output(in3,GPIO.HIGH)
   GPIO.output(in4,GPIO.LOW)
   sleep(2)
   
   v_a.ChangeDutyCycle(power)
   v_b.ChangeDutyCycle(power)
   
   print("\n")
   print("The default speed & direction of motor is LOW & Forward.....")
   print("r-run s-stop f-forward b-backward l-low m-medium h-high e-exit")
   print("\n")    

while True:
   power += 25 if power != 100 else -100
   main(power)