#main script of the robot
from motor import Motor

from utime import sleep
paths = {
    ('st','da'):['L','RR','LOAD'],
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
motor_left = Motor()
motor_right = Motor()


flls = LineSensor() #far left line sensor
lls = LineSensor() #left line sensor
rls = LineSensor()
frls = LineSensor()

def find_type_of_line(): #tells us if we are on a line, veering off, at a junction(left,right,T)
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
        return 'OFFTHEGRID'
    #then other logic that turns the bot

def move_forward():
    motor_left.forward(30)
    motor_right.forward(30)
    sleep(0.1)
    motor_left.off()
    motor_right.off()

def turn(direction):
    pass

def adjust(direction):
    if direction == 'L':
        motor_left.forward(30)
        motor_right.forward(20)
        sleep(0.1)
        motor_left.off()
        motor_right.off()
    elif direction == 'R':
        motor_left.forward(20)
        motor_right.forward(30)
        sleep(0.1)
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
            state = find_type_of_line()j
            match state: #check we are in python 3.10
                case 'ONLINE':
                    move_forward()
                case 'OFFTHEGRID':
                    break
                case 'OFFRIGHT':
                    adjust('R')
                case 'OFFLEFT':
                    adjust('L')
                case 'LEFTTURN':
                    if instruction == 'L':
                        turn()
                        fulfilled = True
                    elif instruction == 'RL':
                        reverse()
                        fulfilled = True
                    else:
                        move_forward()
                case 'RIGHTTURN':
                    if instruction == 'R':
                        turn()
                        fulfilled = True
                    elif instruction == 'RR':
                        reverse()
                        fulfilled = True
                    else:
                        move_forward()
                case 'TJUNCTION':
                    pass