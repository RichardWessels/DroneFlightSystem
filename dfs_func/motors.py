import time
from adafruit_servokit import ServoKit # Not sure if this is needed

import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

class Rotor:
    def __init__(self, channel, training_wheels=20):
        self.channel = channel
        self.motor = hat.channels[channel]
        self.training_wheels = training_wheels
        self.thrust_percent = 0
        self.thrust(0)
        time.sleep(1)

    def get_thrust(self):
        return self.thrust_percent

    def thrust(self, percent): # Consider changing name to throttle
        self.thrust_percent = percent
        if self.thrust_percent > 100:
            self.thrust_percent = 100
        elif self.thrust_percent < 0:
            self.thrust_percent = 0
        print(f"Thrust {self.channel}: {self.thrust_percent}%")
        if self.training_wheels == False:
            self.motor.duty_cycle = int((1000+self.thrust_percent/100*1000)/20_000*65535)
        else:
            limit = self.training_wheels*10
            self.motor.duty_cycle = int((1000+self.thrust_percent/100*limit)/20_000*65535)

class MotorMovement:

    def __init__(self, rotor0, rotor1, rotor2, rotor3):
        self.rotor0 = rotor0
        self.rotor1 = rotor1
        self.rotor2 = rotor2
        self.rotor3 = rotor3

    def throttle(self, percent):
        self.rotor0.thrust(percent)
        self.rotor1.thrust(percent)
        self.rotor2.thrust(percent)
        self.rotor3.thrust(percent)

    def pitch(self, percent):
        self.rotor0.thrust(self.rotor0.get_thrust()+percent)
        self.rotor1.thrust(self.rotor1.get_thrust()+percent)
        self.rotor2.thrust(self.rotor2.get_thrust()-percent)
        self.rotor3.thrust(self.rotor3.get_thrust()-percent)

    def roll(self, percent):
        self.rotor0.thrust(self.rotor0.get_thrust()+percent)
        self.rotor1.thrust(self.rotor1.get_thrust()-percent)
        self.rotor2.thrust(self.rotor2.get_thrust()+percent)
        self.rotor3.thrust(self.rotor3.get_thrust()-percent)

    def yaw(self, percent):
        self.rotor0.thrust(self.rotor0.get_thrust()+percent)
        self.rotor1.thrust(self.rotor1.get_thrust()-percent)
        self.rotor2.thrust(self.rotor2.get_thrust()-percent)
        self.rotor3.thrust(self.rotor3.get_thrust()+percent)


if __name__ == "__main__":

    hat.frequency = 50
    
    rotor1 = Rotor(0, 30)
    rotor2 = Rotor(1)

    rotor1.thrust(0)
    rotor2.thrust(0)
    time.sleep(3)

    rotor1.thrust(100)
    rotor2.thrust(100)
    time.sleep(3)

    rotor1.thrust(0)
    rotor2.thrust(0)
