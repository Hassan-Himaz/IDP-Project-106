
from micropython import const 

from machine import I2C, Pin

import struct

import time

 

class DFRobotVL53L0X:

    _IO_TIMEOUT = const(1000)

   
    _RESULT_RANGE_STATUS = const(0x14)

 

    def __init__(self, i2c, address=0x29):  # Default VL53L0X address is 0x29

        self.i2c = i2c
        self._SYSRANGE_START = const(0x00)
        self._IO_TIMEOUT = const(1000)

   
        self._RESULT_RANGE_STATUS = const(0x14)



        self.address = address

        self.init_sensor()

        self.started = False

 

    def init_sensor(self):

        """Check sensor connection and initialize VL53L0X."""

        if self.address not in self.i2c.scan():

            raise RuntimeError("VL53L0X not found on I2C bus")

       

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

        data = self.i2c.readfrom_mem(self.address, 0x14, 12)

        distance = struct.unpack(">H", data[10:12])[0]

        return distance

 

    def _write_byte(self, register, data):

        """Write one byte to a register."""

        self.i2c.writeto_mem(self.address, register, bytes([data]))

 

    def _read_byte(self, register):

        """Read one byte from a register."""

        return self.i2c.readfrom_mem(self.address, register, 1)[0]

 

# -------- USAGE EXAMPLE --------

i2c = I2C(0, scl=Pin(21), sda=Pin(20), freq = 50000)  # Adjust pins as needed

sensor = DFRobotVL53L0X(i2c)

 

while True:

    distance = sensor.read_distance()

    if distance >= 0:

        print("Distance:", distance, "mm")

    else:

        print("Error: No valid measurement")

    time.sleep(0.5)  # Adjust measurement frequency








     

