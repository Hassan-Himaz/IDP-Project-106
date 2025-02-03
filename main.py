#main script of the robot
from time import sleep
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
    def forward(self,power):
        self.m1Dir.value(0) # forward = 0 reverse = 1 motor 1
        self.pwm1.duty_u16(int(65535*(power)/100)) # speed range 0-100 motor 1
    def reverse(self,power):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535*power/100))

paths = {
    ('st','da'):['L','S','R','L','S','R','L','S','R','L','S','R','L','L','L','L','L','RR','LOAD'],
    ('st','db'):['R','RL','LOAD'],
    ('da','ha'):['L','S','RL','DROP'],
    ('da','hb'):[],
    ('da','hc'):[],
    ('da','hd'):[],
    ('db','ha'):[],
    ('db','hb'):[],
    ('db','hc'):[],
    ('db','hd'):[],
    ('ha','da'):[],
    ('ha','db'):[],
    ('hb','da'):[],
    ('hb','db'):[],
    ('hc','da'):[],
    ('hc','db'):[],
    ('hd','da'):[],
    ('hd','db'):[],

}
motor_left = Motor(4,5)
motor_right = Motor(7,6)
class temp():
    def __init__(self):
        self.value  = 0

flls = Pin(17, Pin.IN, Pin.PULL_DOWN)#far left line sensor
#lls = LineSensor() #left line sensor
#rls = LineSensor()
lls = Pin(12, Pin.IN, Pin.PULL_DOWN)
rls = Pin(11, Pin.IN, Pin.PULL_DOWN)
frls = Pin(16, Pin.IN, Pin.PULL_DOWN)

led = Pin(14, Pin.OUT)

def find_type_of_line(): #tells us if we are on a line, veering off, at a junction(left,right,T)
    print(flls.value(),lls.value(),rls.value(),frls.value())
    if flls.value() == 0 and lls.value() == 0 and rls.value() == 0 and frls.value() == 0:
        return 'ONLINE'
    elif flls.value() == 0 and lls.value() == 1 and rls.value() == 0 and frls.value() == 0:
        return 'OFFRIGHT'
    elif flls.value() == 0 and lls.value() == 0 and rls.value() == 1 and frls.value() == 0:
        return 'OFFLEFT'
    elif flls.value() == 1 and lls.value() == 1 and rls.value() == 0 and frls.value() == 0:
        return 'LEFTTURN'
    elif flls.value() == 0 and lls.value() == 0 and rls.value() == 1 and frls.value() == 1:
        return 'RIGHTTURN'
    elif flls.value() == 1 and lls.value() == 1 and rls.value() == 1 and frls.value() == 1:
        return 'TJUNCTION'
    else:
        return 'TJUNCTION'
    #then other logic that turns the bot

def move_forward(time=0.05):
    motor_left.forward(100)
    motor_right.forward(100)
    sleep(time)
    motor_left.off()
    motor_right.off()

def turn(direction):
    if direction == 'R':
        motor_left.forward(100)
        motor_right.forward(100)
        sleep(0.5)
        motor_left.forward(100)
        motor_right.reverse(100)
        sleep(0.70)
        motor_right.off()
        motor_left.off()
    else:
        motor_left.forward(100)
        motor_right.forward(100)
        sleep(0.5)
        motor_right.forward(100)
        motor_left.reverse(100)
        sleep(0.70)
        motor_left.off()
        motor_right.off()

def adjust(direction):
    if direction == 'L':
        motor_left.forward(100)
        motor_right.forward(60)
        sleep(0.05)
        motor_left.off()
        motor_right.off()
    elif direction == 'R':
        motor_left.forward(60)
        motor_right.forward(100)
        sleep(0.05)
        motor_left.off()
        motor_right.off()
def reverse(direction):
    pass
#main loop:

path = ('st','da')
while True:
    for instruction in paths[path]:
        #where robot is
        fulfilled = False
        while fulfilled == False:
            state = find_type_of_line()
            if state == 'ONLINE':
                move_forward()
            elif state == 'OFFTHEGRID':
                break
            elif state == 'OFFRIGHT':
                adjust('R')
            elif state == 'OFFLEFT':
                adjust('L')
            elif state == 'LEFTTURN':
                if instruction == 'R':
                    turn('R')
                    fulfilled = True
                elif instruction == 'RR':
                    reverse()
                    fulfilled = True
                if instruction == 'L':
                    turn('L')
                    fulfilled = True
                elif instruction == 'RL':
                    reverse()
                    fulfilled = True
                elif instruction == 'S':
                    move_forward(0.5)
                    fulfilled = True
            elif state == 'RIGHTTURN':
                if instruction == 'R':
                    turn('R')
                    fulfilled = True
                elif instruction == 'RR':
                    reverse()
                    fulfilled = True
                if instruction == 'L':
                    turn('L')
                    fulfilled = True
                elif instruction == 'RL':
                    reverse()
                    fulfilled = True
                elif instruction == 'S':
                    move_forward(0.5)
                    fulfilled = True
            elif state == 'TJUNCTION':
                if instruction == 'R':
                    turn('R')
                    fulfilled = True
                elif instruction == 'RR':
                    reverse()
                    fulfilled = True
                if instruction == 'L':
                    turn('L')
                    fulfilled = True
                elif instruction == 'RL':
                    reverse()
                    fulfilled = True
                elif instruction == 'S':
                    move_forward(0.5)
                    fulfilled = True



