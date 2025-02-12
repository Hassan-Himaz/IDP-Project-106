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

from micropython import const
from machine import I2C, Pin
import struct
import time

class DFRobotVL53L0X:
    _IO_TIMEOUT = const(1000)
    _SYSRANGE_START = const(0x00)
    _RESULT_RANGE_STATUS = const(0x14)

    def __init__(self, i2c, address=0x29):  # Default VL53L0X address is 0x29
        self.i2c = i2c
        self._IO_TIMEOUT = const(1000)
        self._SYSRANGE_START = const(0x00)
        self._RESULT_RANGE_STATUS = const(0x14)
        self.address = address
        self.init_sensor()
        self.started = False

    def init_sensor(self):
        """Check sensor connection and initialize VL53L0X."""
        if self.address not in self.i2c.scan():
            pass
            #raise RuntimeError("VL53L0X not found on I2C bus")
       
        # Sensor Initialization Sequence
        self._write_byte(0x88, 0x00)
        self._write_byte(0x80, 0x01)
        self._write_byte(0xFF, 0x01)
        self._write_byte(0x00, 0x00)
        self._write_byte(0x91, self._read_byte(0x91))
        self._write_byte(0x00, 0x01)
        self._write_byte(0xFF, 0x00)
        self._write_byte(0x80, 0x00)

    def start(self):
        """Start continuous measurement mode."""
        self._write_byte(self._SYSRANGE_START, 0x01)
        self.started = True

    def stop(self):
        """Stop continuous measurement."""
        self._write_byte(self._SYSRANGE_START, 0x00)
        self.started = False

    def read_distance(self):
        """Read distance measurement in mm."""
        if not self.started:
            self.start()

        # Wait for measurement to complete
        for _ in range(10):  # Retry up to 10 times
            status = self._read_byte(self._RESULT_RANGE_STATUS)
            if status & 0x01:  # Data ready bit
                break
            time.sleep_ms(10)
        else:
            return -1  # Timeout or no valid data

        # Read distance data
        '''
        data = self.i2c.readfrom_mem(self.address, 0x14, 12)
        distance = struct.unpack(">H", data[10:12])[0]
        return distance'''
        try:
            data = self.i2c.readfrom_mem(self.address, 0x14, 12)
            if len(data) < 12:
                return -1  # Not enough data received

            distance = struct.unpack(">H", data[10:12])[0]

            # Handle out-of-range measurements
            if distance == 0 or distance > 2000:  # VL53L0X max range ~2000mm
                return -1  # Mark invalid measurement
            return distance
        except OSError as e:
            print("I2C Read Error:", e)
            return -1

    def _write_byte(self, register, data):
        """Write one byte to a register."""
        try:
            self.i2c.writeto_mem(self.address, register, bytes([data]))
        except:
            return -1

    def _read_byte(self, register):
        """Read one byte from a register."""
        try:
            return self.i2c.readfrom_mem(self.address, register, 1)[0]
        except:
            return -1

# -------- USAGE EXAMPLE --------
i2c = I2C(0, scl=Pin(21), sda=Pin(20))  # Adjust pins as needed
sensor = DFRobotVL53L0X(i2c)

def tof_scan():
    i2c = I2C(0, scl=Pin(21), sda=Pin(20))  # Adjust pins as needed
    sensor = DFRobotVL53L0X(i2c)
    distance = sensor.read_distance()
   
    if distance > 30:  # Only accept valid distances
        return distance
    else:
        return 0
   
    time.sleep(0.5)
# Adjust measurement frequency


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

    def scan(self, num_scans=10):
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

TINY_CODE_READER_I2C_ADDRESS = 0x0C

# How long to pause between sensor polls.

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

 

print(i2c.scan())

 

# Keep looping and reading the sensor results until we get a QR code

def scan():

    for x in range(100): #these can be changed
        print('trying')

        sleep(TINY_CODE_READER_DELAY)

        read_data = i2c.readfrom(TINY_CODE_READER_I2C_ADDRESS,

                                 TINY_CODE_READER_I2C_BYTE_COUNT)

        print('raw data',read_data)

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
        return tof_scan()
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
button_pin = Pin(14, Pin.IN, Pin.PULL_DOWN)
paths = {
    ('st', 'da'): ['S', 'R', 'WR', 'LOAD'],
    ('da', 'ha'): ['L', 'S', 'R', 'UNLOAD'],
    ('ha', 'da'): ['L', 'S', 'L', 'LOAD'],
    ('da', 'hb'): ['S', 'L', 'L', 'UNLOAD'],
    ('hb', 'da'): ['S', 'L', 'S', 'L', 'S', 'R', 'S'],
    #('hb', 'da'): ['R', 'R', 'S', 'LOAD'],
    ('da', 'hc'): ['S', 'S', 'L', 'L', 'UNLOAD'],
    ('hc', 'da'): ['R', 'L', 'S', 'L', 'S', 'LOAD'],
    ('da', 'hd'): ['S', 'S', 'L', 'L', 'UNLOAD'],
    ('hd', 'da'): ['R', 'R', 'S', 'S', 'LOAD'],
    ('db','ha'):[],
    ('db','hb'):[],
    ('db','hc'):[],
    ('db','hd'):[],
    ('ha','db'):[],
    ('hb','db'):[],
    ('hc','db'):[],
    ('hd','db'):[],

}

motor_left = Motor(4,5)
motor_right = Motor(7,6)
linear_actuator = Motor(0,1)


flls = Pin(17, Pin.IN, Pin.PULL_DOWN)#far left line sensor
lls = Pin(12, Pin.IN, Pin.PULL_DOWN)
rls = Pin(11, Pin.IN, Pin.PULL_DOWN)
frls = Pin(16, Pin.IN, Pin.PULL_DOWN)
ultrasound = Ultrasound()
#qr_scanner = QRScanner()

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
        motor_right.forward(20)
        sleep(1.85)
        motor_right.off()
        motor_left.off()
    elif direction == 'WL':
        motor_left.forward(20)
        motor_right.forward(100)
        sleep(1.85)
        motor_left.off()
        motor_right.off()
    elif direction == 'UR':
        motor_left.forward(100)
        motor_right.reverse(100)
        sleep(1.6) #need to check this
        motor_left.off()
        motor_right.off()
    elif direction == 'BO': # "back out"
        state = find_type_of_line()
        while state != 'OFFLEFT':
            motor_left.reverse(20)
            motor_right.reverse(100)
            sleep(0.1)
            motor_left.off()
            motor_right.off()
        #motor_left.forward(100)
        #motor_right.forward(100)
        #sleep(0.3)
        motor_left.off()
        motor_right.off()
    elif direction == 'UL':
        motor_right.forward(100)
        motor_left.reverse(100)
        sleep(1.50) #need to check this
        motor_left.off()
        motor_right.off()

def adjust(direction, intensity=1):
    if direction == 'L':
        motor_left.forward(100*intensity)
        motor_right.forward(60*intensity)
        sleep(0.05)
        motor_left.off()
        motor_right.off()
    elif direction == 'R':
        motor_left.forward(60*intensity)
        motor_right.forward(100*intensity)
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
    #can add in process to convert long text into actionable qr code
    #return 'hb'
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

def load(current_location = 'd1'):
   
    print('loading')
    # position sensor:
    drop() # make sure linear actuator is bottomed out
    while ultrasound.value() >= 250: #approach till close
        print(ultrasound.value())
        state = find_type_of_line()
        if state == 'ONLINE':
            move_forward()
        elif state == 'TJUNCTION': #if we get to the junction that the thing is placed on we can ignore the ultrasound
            move_forward()
        elif state == 'OFFRIGHT':
            adjust('R')
        elif state == 'OFFLEFT':
            adjust('L')
    while ultrasound.value() <= 300: #approach till far back enough
        move_reverse()
        print(ulasound.value())
    while ultrasound.value() >= 250: #approach till close
        print(ultrasound.value())
        state = find_type_of_line()
        if state == 'ONLINE':
            move_forward()
        elif state == 'TJUNCTION': #if we get to the junction that the thing is placed on we can ignore the ultrasound
            move_forward()
        elif state == 'OFFRIGHT':
            adjust('R')
        elif state == 'OFFLEFT':
            adjust('L')
        #move_forward()
    while ultrasound.value() <= 300: #approach till far back enough
        move_reverse()

    QR = scan()
    tries = 0
    while QR == '':
        #can add error repitition later
        state = find_type_of_line()
        if state == 'ONLINE':
            move_forward(0.05)
        elif state == 'TJUNCTION': #if we get to the junction that the thing is placed on we can ignore the ultrasound
            move_forward(0.05)
        elif state == 'OFFRIGHT':
            adjust('R',intensity=0.5)
        elif state == 'OFFLEFT':
            adjust('L',intensity=0.5)
        QR = scan()
        tries += 1
        if tries > 30: #we are not finding a code so go to other depot
            #added line as qr code no work
            print('stop trying')
            QR = 'B'
            if current_location == 'd2':
                pass
                #return 'st'
            pass
            #return 'd2'
   
    location = find_next_location(QR)
    while ultrasound.value() >= 1: #change distance later
            state = find_type_of_line()
            if state == 'ONLINE':
                move_forward()
            elif state == 'TJUNCTION': #if we get to the junction that the thing is placed on we can ignore the ultrasound
                lift()
                turn('UR')
                return location
            elif state == 'OFFRIGHT':
                adjust('R')
            elif state == 'OFFLEFT':
                adjust('L')
    lift()
    turn('UR')
    return location

def unload():
    state = find_type_of_line()
    while state != 'TJUNCTION':
        state = find_type_of_line()
        if state == 'ONLINE':
            move_forward()
        elif state == 'OFFRIGHT':
            adjust('R')
        elif state == 'OFFLEFT':
            adjust('L')
    drop()
    move_reverse(0.3) #we reverse so we no longer are touching the block
    #turn('BO')

#main loop:

path = ('st','da')

led_pin.value(0)

while True:
    if button_pin.value() != 1:
        sleep(0.05)
    else:
        break
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
                led_pin.value(1)
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
                elif instruction == 'WR':
                    turn('WR')
                    fulfilled = True
                elif instruction == 'WL':
                    turn('WL')
                    fulfilled = True
        if instruction == 'LOAD':
            print(load)
            next_location = load(current_location) #hope this works
            path = (current_location,next_location)
            fulfilled = True
           
        elif instruction == 'UNLOAD':
            unload()
            next_location = 'd1'
            path = (current_location,next_location)
            fulfilled = True





