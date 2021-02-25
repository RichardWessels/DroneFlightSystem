import sys

sys.path.append('..')

from dfs_func import motors

import time
import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

hat.frequency = 50

runLoop = True

while runLoop:
    print("TESTING ALL FOUR MOTORS WITH EQUAL THRUST")

    motorThrottle = int(input("Enter throttle percentage: "))
    duration = int(input("Enter duration (in seconds): "))
    break

throttleLimit = 100

rotor0 = motors.Rotor(0, throttleLimit)
rotor1 = motors.Rotor(1, throttleLimit)
rotor2 = motors.Rotor(2, throttleLimit)
rotor3 = motors.Rotor(3, throttleLimit)
time.sleep(3)

print("Complete initialization...")

rotor0.thrust(motorThrottle)
rotor1.thrust(motorThrottle)
rotor2.thrust(motorThrottle)
rotor3.thrust(motorThrottle)

time.sleep(duration)

rotor0.thrust(0)
rotor1.thrust(0)
rotor2.thrust(0)
rotor3.thrust(0)