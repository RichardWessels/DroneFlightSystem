import sys
import threading
import time
import RPi.GPIO as GPIO
import json

sys.path.append('..')

from dfs_func import transceiver

transceiver.setup()

stop_thread = False

THROTTLE_PIN = transceiver.THROTTLE_PIN
PITCH_PIN = transceiver.PITCH_PIN
ROLL_PIN = transceiver.ROLL_PIN
YAW_PIN = transceiver.YAW_PIN

throttle = transceiver.Transmitter('THR', THROTTLE_PIN)
pitch = transceiver.Transmitter('PIT', PITCH_PIN)
roll = transceiver.Transmitter('ROL', ROLL_PIN)
yaw = transceiver.Transmitter('YAW', YAW_PIN)

update_thread = threading.Thread(target=transceiver.Transmitter.update_pwm, args=(lambda: stop_thread, throttle, pitch, roll, yaw))
update_thread.start()

# thr_vals = {'low': 1000, 'high': 2000}
# pit_vals = {'low': 1000, 'mid':1500, 'high': 2000}
# rol_vals = {'low': 1000, 'mid':1500, 'high': 2000}
# yaw_vals = {'low': 1000, 'mid':1500, 'high': 2000}

# print("Calibrating...")

# print("Throttle Low...")
# time.sleep(2)
# thr_vals['low'] = throttle.get_pwm()
# print("Throttle High...")
# time.sleep(2)
# thr_vals['high'] = throttle.get_pwm()

# print("Pitch Low...")
# time.sleep(2)
# pit_vals['low'] = pitch.get_pwm()
# print("Pitch Mid...")
# time.sleep(2)
# pit_vals['mid'] = pitch.get_pwm()
# print("Pitch High...")
# time.sleep(2)
# pit_vals['high'] = pitch.get_pwm()

# print("Roll Low...")
# time.sleep(2)
# rol_vals['low'] = roll.get_pwm()
# print("Roll Mid...")
# time.sleep(2)
# rol_vals['mid'] = roll.get_pwm()
# print("Roll High...")
# time.sleep(2)
# rol_vals['high'] = roll.get_pwm()

# print("Yaw Low...")
# time.sleep(2)
# yaw_vals['low'] = yaw.get_pwm()
# print("Yaw Mid...")
# time.sleep(2)
# yaw_vals['mid'] = yaw.get_pwm()
# print("Yaw High...")
# time.sleep(2)
# yaw_vals['high'] = yaw.get_pwm()


throttle.calibrate('../config.json')
pitch.calibrate('../config.json')
roll.calibrate('../config.json')
yaw.calibrate('../config.json')


stop_thread = True
update_thread.join()
print("Successful transmitter completion")
GPIO.cleanup()

