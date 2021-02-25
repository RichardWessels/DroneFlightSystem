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

throttleLimit = 100

rotor0 = motors.Rotor(0, throttleLimit)
rotor1 = motors.Rotor(1, throttleLimit)
rotor2 = motors.Rotor(2, throttleLimit)
rotor3 = motors.Rotor(3, throttleLimit)
time.sleep(3)

print("Complete initialization...")

rotor0.thrust(60)
rotor1.thrust(60)
rotor2.thrust(60)
rotor3.thrust(60)

time.sleep(1)

rotor0.thrust(40)
rotor1.thrust(40)
rotor2.thrust(40)
rotor3.thrust(40)

time.sleep(0.5)

rotor0.thrust(70)
rotor1.thrust(70)
rotor2.thrust(70)
rotor3.thrust(70)

time.sleep(1)

rotor0.thrust(0)
rotor1.thrust(0)
rotor2.thrust(0)
rotor3.thrust(0)

time.sleep(0.75)

rotor0.thrust(70)
rotor1.thrust(70)
rotor2.thrust(70)
rotor3.thrust(70)

time.sleep(1)

rotor0.thrust(0)
rotor1.thrust(0)
rotor2.thrust(0)
rotor3.thrust(0)