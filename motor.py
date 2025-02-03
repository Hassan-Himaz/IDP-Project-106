from machine import Pin, PWM
from time import sleep
class Motor():
    def __init__(self,dir_pin,pwm_pin):
        self.m1Dir = Pin(dir_pin , Pin.OUT)   # set pin left wheel
        self.pwm1 = PWM(Pin(pwm_pin))          
        self.pwm1.freq(1000)
        self.pwm1.duty_u16(0)
    def off(self):
        self.pwm1.duty_u16(0)
    def Forward(self,power):
        self.m1Dir.value(0) # forward = 0 reverse = 1 motor 1
        self.pwm1.duty_u16(int(65535*(power)/100)) # speed range 0-100 motor 1
    def Reverse(self,power):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535*power/100))
#example usage
motor=Motor()
while True:
    motor.Forward()

    sleep(1)
    motor.Reverse()
    sleep(1)


