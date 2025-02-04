from machine import Pin, ADC
from time import sleep 

class distance:
    def __init__(self):
        self.sensor_pin = ADC(28)

    def get_distance(self):
        self.pin_value = self.sensor_pin.read_u16()
        self.dist = (self.pin_value*500)/65535 
        return self.dist



while(True):
    d = distance()
    sleep(0.1)
    print(d.get_distance())
    

     
