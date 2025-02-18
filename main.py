from time import sleep

from machine import Pin, PWM, ADC

from time import sleep

import struct

import machine

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

TINY_CODE_READER_I2C_ADDRESS = 0x0C
# How long to pause between sensor polls.
TINY_CODE_READER_DELAY = 0.05
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
 
print(i2c.scan())
 
# Keep looping and reading the sensor results until we get a QR code
def scan():
    for x in range(100): #these can be changed
        sleep(TINY_CODE_READER_DELAY)
        read_data = i2c.readfrom(TINY_CODE_READER_I2C_ADDRESS,
                                 TINY_CODE_READER_I2C_BYTE_COUNT)
        #print('raw data',read_data)
        message_length,  = struct.unpack_from(TINY_CODE_READER_LENGTH_FORMAT, read_data,
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
 

class Ultrasound():

    def __init__(self):

        self.sensor_pin = ADC(28)

 

    def value(self):

        self.measurements = []

        self.trials = 0

        while(len(self.measurements) <= 20): # maximum time should be 0.03 seconds

            self.pin_value = self.sensor_pin.read_u16()

            self.one_measurement = (self.pin_value*500)/65535

            if(self.one_measurement < 30):

                self.measurements.append(self.one_measurement)

            sleep(0.001)

            self.trials += 1

            if self.trials > 30:

                return 30 # if we are having to take too many readings then we have an issue and should just return the base value

        self.dist = 0

        self.measurements.sort()

        self.measurements = self.measurements[4:15]

        for x in self.measurements:

            self.dist += x

        self.dist = self.dist/len(self.measurements)

        print(self.dist)

        return self.dist

 

led_pin = Pin(22,Pin.OUT)

paths = {

    ('st','da'):['LOAD'],

    ('st','db'):['L','S','R','L','S','R','L','S','R','L','S','R','L','L','L','L','L','RR','R','RL','LOAD'],

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

 

flls = Pin(17, Pin.IN, Pin.PULL_DOWN)#far left line sensor

lls = Pin(12, Pin.IN, Pin.PULL_DOWN)

rls = Pin(11, Pin.IN, Pin.PULL_DOWN)

frls = Pin(16, Pin.IN, Pin.PULL_DOWN)

ultrasound = Ultrasound()



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

        return 'TJUNCTION'

    elif flls.value() == 0 and lls.value() == 0 and rls.value() == 1 and frls.value() == 1:

        return 'TJUNCTION'

    elif flls.value() == 1 and lls.value() == 1 and rls.value() == 1 and frls.value() == 1:

        return 'TJUNCTION'

    else:

        return 'TJUNCTION'

 

 

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

    elif direction == 'L':

        motor_left.forward(100)

        motor_right.forward(100)

        sleep(0.5)

        motor_right.forward(100)

        motor_left.reverse(100)

        sleep(0.70)

        motor_left.off()

        motor_right.off()

    else:

        motor_right.forward(100)

        motor_left.reverse(100)

        sleep(1.40) #need to check this

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

 

def move_reverse(time=0.05):

    motor_left.reverse(100)

    motor_right.reverse(100)

    sleep(time)

    motor_left.off()

    motor_right.off()

 

def find_next_location(longtext): #converts qr string to an actionable location

    return longtext[0].lower()

 

def lift():

    pass

 

def drop():

    pass

 

def load(current_location = 'd1'):

    print('loading')

    # position sensor:

    while ultrasound.value() >= 19: #approach till close

        move_forward()
        print('f')

    while ultrasound.value() <= 19: #approach till far back enough

        move_reverse()
        print('r')
    QR = scan()

    tries = 0

    while QR == '':

        #can add error repitition later

        #move_forward(0.1)

        qr_scanner.scan()

        tries += 1

        if tries > 100: #we are not finding a code so go to other depot

            if current_location == 'd2':

                return 'st'

            return 'd2'

    location = find_next_location(QR)
    print(location)

    while ultrasound.value() >= 1: #change distance later

            state = find_type_of_line()

            if state == 'ONLINE':

                move_forward()

            elif state == 'TJUNCTION': #if we get to the junction that the thing is placed on we can ignore the ultrasound

                lift()

                return location

            elif state == 'OFFRIGHT':

                adjust('R')

            elif state == 'OFFLEFT':

                adjust('L')

    lift()

    turn('U')

    return location

 

def unload():

    state = find_type_of_line()

    while state != 'TJUNCTION':

        if state == 'ONLINE':

            move_forward()

        elif state == 'OFFRIGHT':

            adjust('R')

        elif state == 'OFFLEFT':

            adjust('L')

    drop()

    move_reverse(1) #we reverse so we no longer are touching the block

    turn('U')

 

#main loop:

 

path = ('st','da')

while True: #needs to be simplified with new junction detection, needs to count packages delivered

    current_location = path[1] #where we will end up after completing the path

   

    for instruction in paths[path]:

       

        #This logic is messy but not wrong

        fulfilled = False

        print(instruction)

        if instruction == 'LOAD':

            fulfilled = True
        elif instruction == 'UNLOAD':
            fulfilled = True
       
        while fulfilled == False:
            print('moving')
            led_pin.value(1)

            state = find_type_of_line()

            if state == 'ONLINE':

                move_forward()

            elif state == 'OFFRIGHT':

                adjust('R')

            elif state == 'OFFLEFT':

                adjust('L')

            elif state == 'LEFTTURN':

                if instruction == 'R':

                    turn('R')

                    fulfilled = True

                elif instruction == 'RR':

                    move_reverse()

                    fulfilled = True

                if instruction == 'L':

                    turn('L')

                    fulfilled = True

                elif instruction == 'RL':

                    move_reverse()

                    fulfilled = True

                elif instruction == 'S':

                    move_forward(0.5)

                    fulfilled = True

            elif state == 'RIGHTTURN':

                if instruction == 'R':

                    turn('R')

                    fulfilled = True

                elif instruction == 'RR':

                    move_reverse()

                    fulfilled = True

                if instruction == 'L':

                    turn('L')

                    fulfilled = True

                elif instruction == 'RL':

                    move_reverse()

                    fulfilled = True

                elif instruction == 'S':

                    move_forward(0.5)

                    fulfilled = True

            elif state == 'TJUNCTION':

                if instruction == 'R':

                    turn('R')

                    fulfilled = True

                elif instruction == 'RR':

                    move_reverse()

                    fulfilled = True

                if instruction == 'L':

                    turn('L')

                    fulfilled = True

                elif instruction == 'RL':

                    move_reverse()

                    fulfilled = True

                elif instruction == 'S':

                    move_forward(0.5)

                    fulfilled = True

        if instruction == 'LOAD':

            print('load')

            next_location = load(current_location) #hope this works

            path = (current_location,next_location)

        elif instruction == 'UNLOAD':

            unload()

            next_location = 'd1'

            path = (current_location,next_location)
            print path
