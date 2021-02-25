import sys

sys.path.append('..')

from dfs_func import motors
from dfs_func import transceiver

import time
import threading
import json
import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

hat.frequency = 50

# Retrive data from config file
with open('../config.json') as f:
    content = json.load(f)
    transceiver_vals = content['transmitter']

# Transceiver setup
transceiver.setup()

stop_thread = False

THROTTLE_PIN = transceiver.THROTTLE_PIN
PITCH_PIN = transceiver.PITCH_PIN
ROLL_PIN = transceiver.ROLL_PIN
YAW_PIN = transceiver.YAW_PIN

throttle = transceiver.Transmitter('THR', THROTTLE_PIN, transceiver_vals['THR']['pwm_low'], transceiver_vals['THR']['pwm_high'])
pitch = transceiver.Transmitter('PIT', PITCH_PIN, transceiver_vals['PIT']['pwm_low'], transceiver_vals['PIT']['pwm_high'])
roll = transceiver.Transmitter('ROL', ROLL_PIN, transceiver_vals['ROL']['pwm_low'], transceiver_vals['ROL']['pwm_high'])
yaw = transceiver.Transmitter('YAW', YAW_PIN, transceiver_vals['YAW']['pwm_low'], transceiver_vals['YAW']['pwm_high'])

update_thread = threading.Thread(target=transceiver.Transmitter.update_pwm, args=(lambda: stop_thread, throttle, pitch, roll, yaw))
update_thread.start()

# Wait for first PWM signals to be read to avoid first read being 1000
time.sleep(0.5)

runLoop = True

while True:
    try:
        throttleLimit = int(input("ENTER THROTTLE LIMIT: "))
        if throttleLimit > 100:
            raise AttributeError
        break
    except:
        print("Error, try again")

rotor0 = motors.Rotor(0, throttleLimit)
rotor1 = motors.Rotor(1, throttleLimit)
rotor2 = motors.Rotor(2, throttleLimit)
rotor3 = motors.Rotor(3, throttleLimit)
time.sleep(3)

print("Complete initialization...")

t1 = time.time()
while time.time()-t1 < 30:
    
    motorThrottle = throttle.get_percent()

    rotor0.thrust(motorThrottle)
    rotor1.thrust(motorThrottle)
    rotor2.thrust(motorThrottle)
    rotor3.thrust(motorThrottle)

# Cleanup
rotor0.thrust(0)
rotor1.thrust(0)
rotor2.thrust(0)
rotor3.thrust(0)
stop_thread = True
update_thread.join()
print("Successful transmitter completion")