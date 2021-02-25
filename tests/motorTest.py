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
    print("MOTOR TEST")

    motorNum = int(input("Enter motor number: "))
    motorThrottle = int(input("Enter throttle percentage: "))
    throttleLimit = int(input("Enter throttle limit: "))
    duration = int(input("Enter duration (in seconds): "))

    if 100*(motorThrottle/100)*(throttleLimit/100) > 50: # Ensures absolute throttle is checked if higher than 50%
        if input("Press 'y' to confirm motor test: ").lower() == 'y':
            runLoop = False
    else:
        runLoop = False
        
testMotor = motors.Rotor(motorNum, throttleLimit)
time.sleep(3)

print("Complete initialization...")

testMotor.thrust(motorThrottle)
time.sleep(duration)

testMotor.thrust(0)