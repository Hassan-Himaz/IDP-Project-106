import struct
from time import sleep
import machine
import time

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
        
        return qr_codes  # Return a list of decoded QR codes


# Example usage:
if __name__ == "__main__":
    qr_scanner = QRScanner()
    print("Scanning for QR codes...")
    found_codes = qr_scanner.scan(num_scans=5)  # Scan for a number of times (e.g., 5 scans)
    print("Found QR codes:", found_codes)
