##############################################
#
# MAIN ROBOT SCRIPT
#
##############################################

# Importing Required Modules
from time import sleep
from machine import Pin, PWM, ADC
from time import sleep
import struct
import machine
from micropython import const
from machine import I2C, Pin
import struct
import time

# Class for the motors, used by both the wheels and the linear actuator. 
# Contains generic functions to stop or apply power to motors. 
class Motor():

    def __init__(self,dir_pin,pwm_pin):
        # two pins are used to control the motors: to specify the direction and speed.
        self.m1Dir = Pin(dir_pin , Pin.OUT) 
        # initialize the PWM pin to control motor speed.
        self.pwm1 = PWM(Pin(pwm_pin))
        self.pwm1.freq(1000)
        self.pwm1.duty_u16(0)

    def off(self):
        self.pwm1.duty_u16(0)

    # forward() and reverse() take in a parameter for motor power between 0 and 100. 
    # note that motor torque output is not necessarily linear with respect to this power value.
    def forward(self,power):
        self.m1Dir.value(0) # forward = 0 reverse = 1 motor 1
        self.pwm1.duty_u16(int(65535*(power)/100)) # speed range 0-100 motor 1

    def reverse(self,power):
        self.m1Dir.value(1)
        self.pwm1.duty_u16(int(65535*power/100))

# Constants used for the QR code reader 
TINY_CODE_READER_I2C_ADDRESS = 0x0C
TINY_CODE_READER_DELAY = 0.1
TINY_CODE_READER_LENGTH_OFFSET = 0
TINY_CODE_READER_LENGTH_FORMAT = "H"
TINY_CODE_READER_MESSAGE_OFFSET = TINY_CODE_READER_LENGTH_OFFSET + struct.calcsize(TINY_CODE_READER_LENGTH_FORMAT)
TINY_CODE_READER_MESSAGE_SIZE = 254
TINY_CODE_READER_MESSAGE_FORMAT = "B" * TINY_CODE_READER_MESSAGE_SIZE
TINY_CODE_READER_I2C_FORMAT = TINY_CODE_READER_LENGTH_FORMAT + TINY_CODE_READER_MESSAGE_FORMAT
TINY_CODE_READER_I2C_BYTE_COUNT = struct.calcsize(TINY_CODE_READER_I2C_FORMAT)


# Function used to attempt a QR code scan. 
def scan():
    try:
        for x in range(10): # A scan will be attempted 10 times.
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
        # If a scan is unsuccessful, pretend that A was scanned since it is the closest depot.
        return 'A'


# Hard-coded paths between each of the locations for the robot.
# Each path assumes: for start and depots: the robot starts facing away (out) from the location
#                    for houses: the robot starts facing in to the house (after just delivering a package)
# Each instruction in a path is run whenver a junction is detected (line found by either of the far left or right sensors)
# Instruction abbreviations: 
#     'S': proceed straight through the junction
#     'L': turn left at the junction (fast, on-the-spot turn)
#     'R': turn right at the junction (fast, on-the-spot turn)
#     'WL': turn left at the junction (wide, higher radius turn to avoid wall immediately in front)
#     'WR': turn right at the junction (wide, higher radius turn to avoid wall immediately in front)
#     'LBO': back out of a space and end up facing left 
#     'RBO': back out of a space and end up facing right
#     'LOAD': align robot with QR code, scan code, lift up the package, and U-turn out of the depot
#     'UNLOAD': drop the package off at the house
#     'STOP': stop once back at the start point

paths = { 
    ('st', 'da'): ['S', 'R', 'WR', 'LOAD'], 
    ('da', 'ha'): ['L', 'S', 'R', 'UNLOAD'], 
    ('ha', 'da'): ['RBO','S', 'WR', 'LOAD'],
    ('da', 'hb'): ['S', 'L', 'L', 'UNLOAD'], 
    ('hb', 'da'): ['LBO', 'WR', 'S', 'LOAD'], 
    ('da', 'hc'): ['S', 'L', 'S', 'R','L','UNLOAD'], 
    ('hc', 'da'): ['LBO','WL', 'S', 'WR','S', 'LOAD'], 
    ('da', 'hd'): ['S', 'S', 'WL', 'L', 'UNLOAD'], 
    ('hd', 'da'): ['LBO', 'WR', 'S', 'S', 'LOAD'], 
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

# Initialise pin numbers for the various components
led_pin = Pin(22,Pin.OUT)
button_pin = Pin(14, Pin.IN, Pin.PULL_DOWN)
motor_left = Motor(4,5)
motor_right = Motor(7,6)
linear_actuator = Motor(0,1)
flls = Pin(17, Pin.IN, Pin.PULL_DOWN) # far left line sensor
lls = Pin(12, Pin.IN, Pin.PULL_DOWN)  # far right line sensor 
rls = Pin(26, Pin.IN, Pin.PULL_DOWN)  # middle right line sensor
frls = Pin(16, Pin.IN, Pin.PULL_DOWN) # middle left line sensor
i2c = machine.I2C(1,
                  scl=machine.Pin(19), # yellow
                  sda=machine.Pin(18), # blue
                  freq=400000)


# Give the current status with respect to line following based on sensor values
#    'ONLINE': robot is currently straight on the line
#    'OFFRIGHT': robot is veering too far right and adjustment is necessary
#    'OFFLEFT': robot is veering too far left and adjustment is necessary
#    'TJUNCTION': robot has reached an intersection 
def find_type_of_line(): 
    print(flls.value(),lls.value(),rls.value(),frls.value())
    if flls.value() == 0 and lls.value() == 0 and rls.value() == 0 and frls.value() == 0:
        return 'ONLINE'
    elif flls.value() == 0 and lls.value() == 1 and rls.value() == 0 and frls.value() == 0:
        return 'OFFRIGHT'
    elif flls.value() == 0 and lls.value() == 0 and rls.value() == 1 and frls.value() == 0:
        return 'OFFLEFT'
    else:
        return 'TJUNCTION'

# Command to activate both motors forward for a variable amount of time at full power. 
# default time = 0.03 seconds
def move_forward(time=0.03):
    motor_left.forward(100)
    motor_right.forward(100)
    sleep(time)
    motor_left.off()
    motor_right.off()

# Function to execute turns. The durations each motor is activated has been experimentally tested
# and fine-tuned to account for potential differences in left and right motor power.
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
        sleep(1.7) 
        motor_left.off()
        motor_right.off()
    elif direction == 'RBO': 
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
    elif direction == 'LBO': 
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

# Function used to make adjustments when line following. 
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

# Function used to make both motors reverse for a variable amount of time.
def move_reverse(time=0.05):
    motor_left.reverse(100)
    motor_right.reverse(100)
    sleep(time)
    motor_left.off()
    motor_right.off()

# Converts qr string to an actionable location (e.g. 'ha', 'hb')
def find_next_location(longtext): 
    return 'h' + longtext[0].lower()

# Activate the linear actuator to lift the block.
def lift():
    linear_actuator.reverse(100)
    sleep(3)
    linear_actuator.off()

# Activate the linear actuator to drop the block.
# Drop time is longer than lift time to ensure linear actuator reaches the bottom of its
# range every time.
def drop():
    linear_actuator.forward(100)
    sleep(3.5)
    linear_actuator.off()
    pass

# Function to perform alignment with QR code, scan, lift the block, turn out of the depot,
# and update the path to the next destination. 
# current_location: which depot we are currently at
# spot_number: which loading space to advance to (based on how many blocks already picked up)
# spot_number 1 is the first block, etc.
def load(current_location = 'da', spot_number = 1):
    print('loading')

    # First, the robot reverses to the nearest junction (i.e. backs up so distance
    # from QR code is sufficiently far.)
    state = find_type_of_line()
    while state != 'TJUNCTION':
        state = find_type_of_line()
        move_reverse()

    # Then, the robot moves forward while line following and trying to scan the QR code.
    QR = scan()
    tries = 0
    while QR == '':
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
        if tries > 300: 
            # If it fails too many times, we just go to depot A.
            print('stop trying')
            QR = 'A'

    location = find_next_location(QR)
    move_forward(0.5)

    # Once we have a scan, we count the number of loading spaces to advance depending
    # on how many blocks have already been picked up. 
    count = 0
    while True: 
            state = find_type_of_line()
            if state == 'ONLINE':
                move_forward()
            elif state == 'TJUNCTION': 
                count += 1 # Increment the count of how many spaces we have passed already.

                # Separate logic for the last block since the robot never advances enough
                # to see the final junction. We instead just advance forward 0.5 seconds
                # and then pick up the block.
                if count == 3 and spot_number == 4:
                    move_forward(0.5)
                    # direction to U-turn depends on which depot we're at.
                    if current_location == 'db':
                        # start lifting the forklift while the turn is still in progress
                        # to save time.
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
                
                # Otherwise, for the first 3 spaces, we just pick up the block when the junction
                # is reached.
                if count == spot_number:
                    linear_actuator.reverse(100)
                    move_reverse(0.05)
                    # Direction to u-turn depends on which depot we are at. 
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
    return location

# Function to drop the block off. 
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

# Function to come to a stop at the start point. 
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


# make sure LED starts off.
led_pin.value(0)

# Main function - this essentially only contains a loop to follow the paths prescribed above
# one at a time. Once a path is complete it starts following the next path based on whether
# it is returning to depot, going to a house, etc. 
def main():
    path = ('st','da') # First path is to go to depot A
    led_pin.value(0)
    current_block_number = 1
    while True: 
        current_location = path[1] # Update current_location to where we will end up after completing the path
        for instruction in paths[path]:
            fulfilled = False # fulfilled variable is used for when we need to wait to the next junction 
                              # to execute the path instruction. Set to True when a junction is reached
                              # and the turn is completed. 
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


drop() # Make sure forklift is fully dropped when robot is booted.
while True:
    led_pin.value(0)
    if button_pin.value() != 1: # Wait for button to be pressed before starting the main loop. 
        sleep(0.05)
    else:
        main()

