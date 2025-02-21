#main script of the robot 
from time import sleep
from machine import Pin, PWM, ADC
from time import sleep
import struct
import machine
from micropython import const
from machine import I2C, Pin
import struct
import time



class Motor():


    def __init__(self,dir_pin,pwm_pin):
        self.m1Dir = Pin(dir_pin , Pin.OUT) # set pin left wheel
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

TINY_CODE_READER_I2C_ADDRESS = 0x0C
TINY_CODE_READER_DELAY = 0.1
TINY_CODE_READER_LENGTH_OFFSET = 0
TINY_CODE_READER_LENGTH_FORMAT = "H"
TINY_CODE_READER_MESSAGE_OFFSET = TINY_CODE_READER_LENGTH_OFFSET + struct.calcsize(TINY_CODE_READER_LENGTH_FORMAT)
TINY_CODE_READER_MESSAGE_SIZE = 254
TINY_CODE_READER_MESSAGE_FORMAT = "B" * TINY_CODE_READER_MESSAGE_SIZE
TINY_CODE_READER_I2C_FORMAT = TINY_CODE_READER_LENGTH_FORMAT + TINY_CODE_READER_MESSAGE_FORMAT
TINY_CODE_READER_I2C_BYTE_COUNT = struct.calcsize(TINY_CODE_READER_I2C_FORMAT)
# Set up for the Pico, pin numbers will vary according to your setup.
i2c = machine.I2C(1,
                  scl=machine.Pin(19), # yellow
                  sda=machine.Pin(18), # blue
                  freq=400000)



def scan():
    try:
        for x in range(10): #these can be changed
            print('trying')
            sleep(TINY_CODE_READER_DELAY)
            read_data = i2c.readfrom(TINY_CODE_READER_I2C_ADDRESS,
                                     TINY_CODE_READER_I2C_BYTE_COUNT)
            print('raw data',read_data)
            message_length, = struct.unpack_from(TINY_CODE_READER_LENGTH_FORMAT, read_data,
        TINY_CODE_READER_LENGTH_OFFSET)
            message_bytes = struct.unpack_from(TINY_CODE_READER_MESSAGE_FORMAT, read_data,
        TINY_CODE_READER_MESSAGE_OFFSET)
            try:
                message_string = bytearray(message_bytes[0:message_length]).decode("utf-8")
                print(message_string)
                return(message_string)
            except:
                print("Couldn't decode as UTF 8")
                pass
        return ''
    except:
        return 'A'

led_pin = Pin(22,Pin.OUT)
button_pin = Pin(14, Pin.IN, Pin.PULL_DOWN)
paths = { #needs to be updated so that we have backout aswell
    ('st', 'da'): ['S', 'R', 'WR', 'LOAD'], #CHECKED
    ('da', 'ha'): ['L', 'S', 'R', 'UNLOAD'], #CHECKED (in first competition)
    ('ha', 'da'): ['RBO','S', 'WR', 'LOAD'], #UPDATED, NEEDS CHECK
    ('da', 'hb'): ['S', 'L', 'L', 'UNLOAD'], #CHECKED (before first competition)
    ('hb', 'da'): ['LBO', 'WR', 'S', 'LOAD'], #CHECKED
    ('da', 'hc'): ['S', 'L', 'S', 'R','L','UNLOAD'], # CHECKED
    ('hc', 'da'): ['LBO','WL', 'S', 'WR','S', 'LOAD'], #CHECKED
    ('da', 'hd'): ['S', 'S', 'WL', 'L', 'UNLOAD'], # CHECKED
    ('hd', 'da'): ['LBO', 'WR', 'S', 'S', 'LOAD'], #CHECKED
    ('db','ha'):['R','L','UNLOAD'],
    ('db','hb'):['S', 'R', 'S', 'R', 'UNLOAD'],
    ('db','hc'):['S', 'R', 'L', 'L', 'UNLOAD'],
    ('db','hd'):['S', 'S', 'WR', 'S', 'R', 'UNLOAD'],
    ('ha','db'):['LBO', 'WL', 'LOAD'],
    ('hb','db'):['RBO', 'S', 'WL', 'S', 'LOAD'],
    ('hc','db'):['LBO', 'WR', 'WL', 'S', 'LOAD'],
    ('hd','db'):['RBO', 'S', 'WL', 'S', 'S', 'LOAD'],
    ('ha', 'st'): ['RBO', 'R','STOP'],
    ('hb', 'st'): ['LBO', 'WR', 'R', 'L', 'STOP'],
    ('hc', 'st'): ['LBO', 'WL', 'S', 'WR', 'R', 'L', 'STOP'],
    ('hd', 'st'): ['LBO', 'WR', 'S', 'R', 'L', 'STOP']
}
motor_left = Motor(4,5)
motor_right = Motor(7,6)
linear_actuator = Motor(0,1)
flls = Pin(17, Pin.IN, Pin.PULL_DOWN)#far left line sensor
lls = Pin(12, Pin.IN, Pin.PULL_DOWN)
rls = Pin(26, Pin.IN, Pin.PULL_DOWN)
frls = Pin(16, Pin.IN, Pin.PULL_DOWN)



def find_type_of_line(): #tells us if we are on a line, veering off, at a junction(left,right,T)
    print(flls.value(),lls.value(),rls.value(),frls.value())
    if flls.value() == 0 and lls.value() == 0 and rls.value() == 0 and frls.value() == 0:
        return 'ONLINE'
    elif flls.value() == 0 and lls.value() == 1 and rls.value() == 0 and frls.value() == 0:
        return 'OFFRIGHT'
    elif flls.value() == 0 and lls.value() == 0 and rls.value() == 1 and frls.value() == 0:
        return 'OFFLEFT'
    else:
        return 'TJUNCTION'


def move_forward(time=0.03):
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
        sleep(0.75)
        motor_right.off()
        motor_left.off()
    elif direction == 'L':
        motor_left.forward(100)
        motor_right.forward(100)
        sleep(0.5)
        motor_right.forward(100)
        motor_left.reverse(100)
        sleep(0.75)
        motor_left.off()
        motor_right.off()
    if direction == 'WR':
        motor_left.forward(100)
        motor_right.forward(10)
        sleep(1.63)
        motor_right.off()
        motor_left.off()
    elif direction == 'WL':
        motor_left.forward(10)
        motor_right.forward(100)
        sleep(1.57)
        motor_left.off()
        motor_right.off()
    elif direction == 'UR':
        motor_left.forward(100)
        motor_right.reverse(100)
        sleep(1.7) #need to check this
        motor_left.off()
        motor_right.off()
    elif direction == 'RBO': # "back out"
        state = frls.value()
        while state == 0:
            state = frls.value()
            motor_left.reverse(0)
            motor_right.reverse(100)
            sleep(0.02)
            motor_left.off()
            motor_right.off()
        state = rls.value()
        while state == 0:
            state = rls.value()
            motor_left.reverse(20)
            motor_right.reverse(100)
            sleep(0.02)
        motor_left.reverse(20)
        motor_right.reverse(100)
        sleep(0.27)
        motor_left.off()
        motor_right.off()
        move_forward(0.3)
    elif direction == 'LBO': # "back out"
        state = flls.value()
        while state == 0:
            state = flls.value()
            motor_right.reverse(0)
            motor_left.reverse(100)
            sleep(0.02)
            motor_left.off()
            motor_right.off()
        state = lls.value()
        while state == 0:
            state = lls.value()
            motor_right.reverse(20)
            motor_left.reverse(100)
            sleep(0.02)
            motor_left.off()
            motor_right.off()
        motor_right.reverse(20)
        motor_left.reverse(100)
        sleep(0.3)
        motor_left.off()
        motor_right.off()
        move_forward(0.3)
    elif direction == 'UL':
        motor_right.forward(100)
        motor_left.reverse(100)
        sleep(1.70) #need to check this
        motor_left.off()
        motor_right.off()


def adjust(direction, intensity=1):
    if direction == 'L':
        motor_left.forward(100*intensity)
        motor_right.forward(70*intensity)
        sleep(0.03)
        motor_left.off()
        motor_right.off()
    elif direction == 'R':
        motor_left.forward(70*intensity)
        motor_right.forward(100*intensity)
        sleep(0.03)
        motor_left.off()
        motor_right.off()


def move_reverse(time=0.05):
    motor_left.reverse(100)
    motor_right.reverse(100)
    sleep(time)
    motor_left.off()
    motor_right.off()


def find_next_location(longtext): #converts qr string to an actionable location
    return 'h' + longtext[0].lower()


def lift():
    linear_actuator.reverse(100)
    sleep(3)
    linear_actuator.off()


def drop():
    linear_actuator.forward(100)
    sleep(3.5)
    linear_actuator.off()
    pass


def load(current_location = 'da', spot_number = 1):
    print('loading')

    state = find_type_of_line()
    while state != 'TJUNCTION':
        state = find_type_of_line()
        move_reverse()
    QR = scan()
    tries = 0
    while QR == '':
        #can add error repitition later
        state = find_type_of_line()
        if state == 'ONLINE':
            move_forward(0.3)
        elif state == 'TJUNCTION': #if we get to the junction that the thing is placed on we can ignore the ultrasound
            move_forward(0.3)
        elif state == 'OFFRIGHT':
            adjust('R',intensity=0.7)
        elif state == 'OFFLEFT':
            adjust('L',intensity=0.7)
        QR = scan()
        tries += 1
        if tries > 300: #we are not finding a code so go to other depot
            #added line as qr code no work
            print('stop trying')
            QR = 'B'

    location = find_next_location(QR)
    move_forward(0.5)
    count = 0
    while True: #change distance later
            state = find_type_of_line()
            if state == 'ONLINE':
                move_forward()
            elif state == 'TJUNCTION': #if we get to the junction that the thing is placed on we can ignore the ultrasound
                count += 1
                if count == 3 and spot_number == 4:
                    move_forward(0.5)
                    
                    if current_location == 'db':
                        linear_actuator.reverse(100)
                        turn('UL')
                        sleep(0.5)
                        linear_actuator.off()
                    else:
                        linear_actuator.reverse(100)
                        turn('UR')
                        sleep(0.5)
                        linear_actuator.off()
                    temp_count = 0
                    while temp_count < 10:
                        state = find_type_of_line()
                        temp_count += 1
                        if state == 'OFFLEFT':
                            adjust('L')
                        elif state == 'OFFRIGHT':
                            adjust('R')
                        else:
                            move_forward()
                    return location
                if count == spot_number:
                    linear_actuator.reverse(100)
                    move_reverse(0.05)
                    if current_location == 'db':
                        
                        turn('UL')
                        sleep(0.45)
                        linear_actuator.off()

                    else:
                      
                        turn('UR')
                        sleep(0.45)
                        linear_actuator.off()
                    temp_count = 0
                    return location
                move_forward(0.2)
            elif state == 'OFFRIGHT':
                adjust('R')
            elif state == 'OFFLEFT':
                adjust('L')
    #lift()
    if current_location == 'db':
        turn('UL')
    else:
        turn('UR')
    temp_count = 0
    while temp_count < 6:
        state = find_type_of_line()
        temp_count += 1
        if state == 'OFFLEFT':
            adjust('L')
        elif state == 'OFFRIGHT':
            adjust('R')
        else:
            move_forward
    return location


def unload():
    linear_actuator.forward(100)
    state = find_type_of_line()
    while state != 'TJUNCTION':
        state = find_type_of_line()
        if state == 'ONLINE':
            move_forward()
        elif state == 'OFFRIGHT':
            adjust('R')
        elif state == 'OFFLEFT':
            adjust('L')
    sleep(1.5)
    move_reverse(0.3) #we reverse so we no longer are touching the block
    linear_actuator.off()

def stop():
    led_pin.value(0)
    state = find_type_of_line()
    while state != 'TJUNCTION':
        state = find_type_of_line()
        if state == 'ONLINE':
            move_forward()
        elif state == 'OFFRIGHT':
            adjust('R')
        elif state == 'OFFLEFT':
            adjust('L')
    move_forward(0.8)
#main loop:
led_pin.value(0)

def main():
    path = ('st','da')
    led_pin.value(0)
    current_block_number = 1
    while True: #needs to be simplified with new junction detection, needs to count packages delivered
        current_location = path[1] #where we will end up after completing the path
        for instruction in paths[path]:
            fulfilled = False
            print(instruction)
            if instruction == 'LOAD':
                fulfilled = True
            elif instruction == 'UNLOAD':
                fulfilled = True
            elif instruction == 'LBO':
                turn('LBO')
                fulfilled = True
            elif instruction == 'RBO':
                turn('RBO')
                fulfilled = True
            elif instruction == 'STOP':
                stop()
                return ''
                fulfilled = True
            while fulfilled == False:
                state = find_type_of_line()
                if state == 'ONLINE':
                    move_forward()
                elif state == 'OFFRIGHT':
                    adjust('R')
                elif state == 'OFFLEFT':
                    adjust('L')
                elif state == 'TJUNCTION':
                    led_pin.value(1)
                    if instruction == 'R':
                        turn('R')
                        fulfilled = True
                    if instruction == 'L':
                        turn('L')
                        fulfilled = True
                    elif instruction == 'S':
                        move_forward(0.5)
                        fulfilled = True
                    elif instruction == 'WR':
                        turn('WR')
                        fulfilled = True
                    elif instruction == 'WL':
                        turn('WL')
                        fulfilled = True
            if instruction == 'LOAD':
                print(load)
                steps_to_load = (current_block_number%4)
                if steps_to_load == 0:
                    steps_to_load = 4
                temp = load(current_location, spot_number = steps_to_load)
                
                if current_location == 'db':
                    if temp == 'hd':
                        temp = 'ha'

                
                path = (current_location,temp)
                fulfilled = True
            elif instruction == 'UNLOAD':
                unload()
                current_block_number += 1
                if current_block_number > 8:
                    next_location = 'st'
                elif current_block_number > 4:
                    next_location = 'db'
                else:
                    next_location = 'da'
                path = (current_location,next_location)
                fulfilled = True
drop()
while True:
    led_pin.value(0)
    if button_pin.value() != 1:
        sleep(0.05)
    else:
        main()

