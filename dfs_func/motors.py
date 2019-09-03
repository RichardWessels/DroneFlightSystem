import time
from adafruit_servokit import ServoKit # Not sure if this is needed

import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

class Rotor:
    def __init__(self, channel, training_wheels=True):
        self.motor = hat.channels[channel]
        self.training_wheels = training_wheels

    def thrust(self, percent):
        print(f"Thrust: {percent}%")
        if self.training_wheels == True:
            self.motor.duty_cycle = int((1000+percent/100*200)/20_000*65535)
        else:
            self.motor.duty_cycle = int((1000+percent/100*1000)/20_000*65535)

if __name__ == "__main__":

    hat.frequency = 50
    
    rotor1 = Rotor(0)

    rotor1.thrust(0)
    time.sleep(3)

    rotor1.thrust(50)
    time.sleep(3)

    rotor1.thrust(0)