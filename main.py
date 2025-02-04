#main script of the robot
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

class QRScanner:
    def __init__(self, i2c_address=0x0C, sda_pin=18, scl_pin=19, freq=400000):
        # Initialize the I2C interface
        self.i2c_address = i2c_address
        self.TINY_CODE_READER_I2C_ADDRESS = self.i2c_address
        self.TINY_CODE_READER_DELAY = 0.05
        self.TINY_CODE_READER_LENGTH_OFFSET = 0
        self.TINY_CODE_READER_LENGTH_FORMAT = "H"
        self.TINY_CODE_READER_MESSAGE_OFFSET = self.TINY_CODE_READER_LENGTH_OFFSET + struct.calcsize(self.TINY_CODE_READER_LENGTH_FORMAT)
        self.TINY_CODE_READER_MESSAGE_SIZE = 254
        self.TINY_CODE_READER_MESSAGE_FORMAT = "B" * self.TINY_CODE_READER_MESSAGE_SIZE
        self.TINY_CODE_READER_I2C_FORMAT = self.TINY_CODE_READER_LENGTH_FORMAT + self.TINY_CODE_READER_MESSAGE_FORMAT
        self.TINY_CODE_READER_I2C_BYTE_COUNT = struct.calcsize(self.TINY_CODE_READER_I2C_FORMAT)
        
        # Set up the I2C interface with the provided pins and frequency
        self.i2c = machine.I2C(1,
                               scl=machine.Pin(scl_pin),  # yellow
                               sda=machine.Pin(sda_pin),  # blue
                               freq=freq)
        print(self.i2c.scan())

    def scan(self, num_scans=5):
        """Scan for QR codes a specific number of times."""
        qr_codes = []

        for _ in range(num_scans):
            sleep(self.TINY_CODE_READER_DELAY)
            read_data = self.i2c.readfrom(self.TINY_CODE_READER_I2C_ADDRESS, self.TINY_CODE_READER_I2C_BYTE_COUNT)
            message_length, = struct.unpack_from(self.TINY_CODE_READER_LENGTH_FORMAT, read_data, self.TINY_CODE_READER_LENGTH_OFFSET)
            message_bytes = struct.unpack_from(self.TINY_CODE_READER_MESSAGE_FORMAT, read_data, self.TINY_CODE_READER_MESSAGE_OFFSET)

            if message_length == 0:
                continue

            try:
                message_string = bytearray(message_bytes[0:message_length]).decode("utf-8")
                qr_codes.append(message_string)  # Store found QR code
            except:
                print("Couldn't decode as UTF-8")
                pass
        if len(qr_codes) == 0:
            return ''
        return qr_codes[0]  # Return a list of decoded QR codes


class ultrasound():
    def __init__(self):
        self.sensor_pin = ADC(28)

    def value(self):
        self.pin_value = self.sensor_pin.read_u16()
        self.dist = (self.pin_value*500)/65535 
        return self.dist
    
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

flls = Pin(17, Pin.IN, Pin.PULL_DOWN)#far left line sensor
lls = Pin(12, Pin.IN, Pin.PULL_DOWN)
rls = Pin(11, Pin.IN, Pin.PULL_DOWN)
frls = Pin(16, Pin.IN, Pin.PULL_DOWN)
ultrasound = Pin(12, Pin.IN, Pin.PULL_DOWN) #random pin
qr_scanner = QRScanner()
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

def move_reverse(direction):
    pass

def find_next_location(longtext): #converts qr string to an actionable location
    return 'hb'

def lift():
    pass

def load():
    # position sensor:
    while ultrasound.value() >= 19: #approach till close
        move_forward()
    while ultrasound.value() <= 19: #approach till far back enough
        move_reverse()
    QR = qr_scanner.scan()
    while QR == '':
        #can add error repitition later
        move_forward(0.1)
        qr_scanner.scan()
    location = find_next_location(QR)
    while ultrasound.value() >= 1: #change distance later
            state = find_type_of_line()
            if state == 'ONLINE':
                move_forward()
            elif state == 'OFFTHEGRID':
                break
            elif state == 'OFFRIGHT':
                adjust('R')
            elif state == 'OFFLEFT':
                adjust('L')
    lift()
    return location
    
    

#main loop:

path = ('st','da')
while True: #needs to be simplified with new junction detection
    current_location = path[1] #where we will end up after completing the path
    for instruction in paths[path]:
        #where robot is
        fulfilled = True
        if instruction != 'LOAD':
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
            next_location = load()
            path = (current_location,next_location)



