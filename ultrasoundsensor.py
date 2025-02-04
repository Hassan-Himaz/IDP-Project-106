from machine import Pin, ADC
from time import sleep 

class distance:
    def __init__(self):
        self.sensor_pin = ADC(28)

    def get_distance(self):
        self.measurements = []
        while(len(self.measurements) <= 20):
            self.pin_value = self.sensor_pin.read_u16()
            self.one_measurement = (self.pin_value*500)/65535 
            if(self.one_measurement < 30):
                self.measurements.append(self.one_measurement)
            sleep(0.001)
        self.dist = 0
        
        self.measurements.sort()
        self.measurements = self.measurements[5:15]
        for x in self.measurements:
            self.dist += x
        self.dist = self.dist/len(self.measurements)
        return self.dist
    
    

while(True):
    d = distance()
    sleep(0.1)
    print(d.get_distance())
    

     

