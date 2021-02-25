import dfs_func.motors as motors
import dfs_func.transceiver as transceiver
import dfs_func.imu2 as imu
import dfs_func.pid5 as pid

import time
import json
import RPi.GPIO as GPIO 
import threading
from adafruit_servokit import ServoKit # Not sure if this is needed
import board
import busio
import adafruit_pca9685


# Retrive data from config file
with open('config.json') as f:
    content = json.load(f)
    gyro_bias = content['imu']['gyro_bias']
    transceiver_vals = content['transmitter']

# Motors Setup
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)
hat.frequency = 50

rotor0 = motors.Rotor(0, 50)
rotor1 = motors.Rotor(1, 50)
rotor2 = motors.Rotor(2, 50)
rotor3 = motors.Rotor(3, 50)

movement = motors.MotorMovement(rotor0, rotor1, rotor2, rotor3)

# IMU Setup
mpu = imu.mpu6050(0x68)
# gyro_bias = errors_raw['imu']['gyro_bias']

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

# PID Setup
kp = 2
ki = 3
kd = 1

# Integral and Derivative Terms Init
i = 0
d = 0
time_end = time.time()
time_begin = time.time()

angles = {'x': 0, 'y': 0}
MAX_ANGLE = 45 # Limits desired angle to 45 degrees
d_ang_1 = 0
d_ang_2 = 0
d_ang_3 = 0
d_ang_4 = 0
transceiver_threshold = 25

comp_t_begin = time.time()
t1 = time.time()

while time.time()-t1 < 15:
    comp_t_end = time.time()
    imu.complementary_filter(angles, mpu.get_accel_data(), mpu.get_gyro_data(), gyro_bias, comp_t_end-comp_t_begin)
    comp_t_begin = time.time()

    # Change percent roll to value from -45 to 45 degrees
    if roll.get_percent() > 75:
        desired_angle = 10
    elif roll.get_percent() < 25:
        desired_angle = -10
    else:
        desired_angle = 0
    # desired_angle = ((roll.get_percent()-50)/50)*45
    # desired_angle_total = (desired_angle+d_ang_1+d_ang_2+d_ang_3+d_ang_4)/5
    d_ang_4 = d_ang_3
    d_ang_3 = d_ang_2
    d_ang_2 = d_ang_1
    d_ang_1 = desired_angle

    pid_val = pid.applyPID(desired_angle, angles['y'], mpu.get_gyro_data()['y']-gyro_bias['y'], kp, ki, kd, i, d, time_end, time_begin)

    rotor0.thrust(throttle.get_percent()-pid_val)
    rotor1.thrust(throttle.get_percent()+pid_val)

    # movement.pitch(pid_val)

    print(f"X angle is: {angles['x']}, Y angle is: {angles['y']}")
    print(f"THR: {throttle.get_percent} ::: PIT: {pitch.get_percent()} ::: ROL: {roll.get_percent()} ::: YAW: {yaw.get_percent()}")
    print(f"Desired X-angle: {desired_angle}")


# Cleanup
rotor0.thrust(0)
rotor1.thrust(0)
rotor2.thrust(0)
rotor3.thrust(0)
stop_thread = True
update_thread.join()
print("Successful transmitter completion")
GPIO.cleanup()