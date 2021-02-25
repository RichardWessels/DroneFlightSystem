# Where the craziness lies

import sensors
import motors
import time
import json
import matplotlib.pyplot as plt

import RPi.GPIO as GPIO
import board
import busio
import adafruit_pca9685
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)


# Not sure if I want to use a class or function. The issue is that the angle updates outside of this and this function must have up to date info

def constrain(value, minimum, maximum):
    return max(min(maximum, value), minimum)

def proportional(p_const, error):
    return p_const*error

def integral(error, delta_time): # Rewrite as one PID function
    # return error*delta_time       # Apparently don't need delta time
    return error

def derivative(d_const, error, prev_error, delta_time, previous_delta):
    # print(f"DC: {d_const}, err_diff: {error-prev_error}")
    # print((error-prev_error)/delta_time)
    # if abs((error-prev_error)/delta_time) > 80:
    #     return d_const*(error-prev_error)/delta_time  # This works as the error-change is negative when the error is reducing
    # return 0

    new_delta = (error-prev_error) / delta_time
    freq_cut = 10
    rc = 1 / (2*3.14*freq_cut)
    print((previous_delta + delta_time / (rc + delta_time) * (new_delta-previous_delta)))
    return constrain(d_const*(previous_delta + delta_time / (rc + delta_time) * (new_delta-previous_delta)), -50, 50)


def applyPID(pid_sum):
    pass


if __name__ == "__main__":

    mpu = sensors.mpu6050(0x68)
    hat.frequency = 50

    angles = {'x': 0, 'y': 0}
    errors = {'count': 1030, 'x':1.7238, 'y':0.4511, 'z':2.1298}    

    rotor0 = motors.Rotor(0, 50)
    rotor1 = motors.Rotor(1, 50)
    # rotor2 = motors.Rotor(2)
    # rotor3 = motors.Rotor(3)

    with open('../config.json') as f:
        errors_raw = json.load(f)

    errors = errors_raw['imu']['gyro_bias']

    general_throttle = 50

    x_arr = []
    p_arr = []
    i_arr = []
    d_arr = []
    error_arr = []

    kp = 3
    ki = 0
    kd = 1


    rotor0.thrust(general_throttle)
    rotor1.thrust(general_throttle)

    i = 0 # Initialize integral term
    d = 0 # Initialize derivative term
    d1 = 0
    d2 = 0

    t5 = time.time()

    t9 = time.time()
    while time.time()-t9 < 20:

        try:
            sensors.complementary_filter(angles, mpu.get_accel_data(), mpu.get_gyro_data(), errors) # Read value might occur due to high accel value...
        except:
            print("Angle retrieve error...")

        # gyro_y = sensors.getGyro(mpu)[1] # Try use PID using rates instead of desired angle

        try:
            prev_error_y = error_y
        except:
            print("Prev error assign error...")
            prev_error_y = angles['y'] # Temporary change

        error_y = angles['y'] # No subtraction as desired angle is 0, recently added negative 10 due to position error of IMU

        print(f"Error: {error_y}, PrevError: {prev_error_y}")
        # print(f"DEGREES: {gyro_y}*/second")

        p = proportional(kp, error_y-12) # Change gain constants to Kp Ki etc.

        d2 = d1
        d1 = d

        t6 = time.time()
        if -3 < error_y < 3: # Try remove time element from integral and reduce the constant
            i += integral(error_y, t6-t5)*ki  # Added if to ensure integral doesn't act unless close.
        d = derivative(kd, error_y-12, prev_error_y-12, t6-t5, d) # To tune the derivative bit, run with low with one arm down, see the response and increase/decrease accordingly.
        t5 = time.time()

        d = (d + d1 + d2)/3

        print(f"P: {p} + I: {i} + D: {d}")

        x_arr.append(time.time())
        p_arr.append(p)
        i_arr.append(i)
        d_arr.append(d)
        error_arr.append(error_y)

        pid = (p-d) 

        print(f"= {pid}")

        print(f"PID: {pid}")

        rotor0.thrust(general_throttle + int(pid))
        rotor1.thrust(general_throttle + int(-pid))

    rotor0.thrust(0)
    rotor1.thrust(0)

    plt.plot(x_arr, p_arr, label="P")
    # plt.plot(x_arr, i_arr, label="I")
    plt.plot(x_arr, d_arr, label="D")
    plt.plot(x_arr, error_arr, label="Error")
    plt.legend()

    plt.show()

    GPIO.cleanup()