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

def derivative(d_const, gyro_d, delta_time, previous_g):
    # print(f"DC: {d_const}, err_diff: {error-prev_error}")
    # print((error-prev_error)/delta_time)
    # if abs((error-prev_error)/delta_time) > 80:
    #     return d_const*(error-prev_error)/delta_time  # This works as the error-change is negative when the error is reducing
    # return 0

    new_delta = (gyro_d)
    freq_cut = 2
    rc = 1 / (2*3.14*freq_cut)
    print((previous_g + delta_time / (rc + delta_time) * (new_delta-previous_g)))
    return constrain(previous_g + delta_time / (rc + delta_time) * (new_delta-previous_g), -50, 50)


def applyPID(pid_sum):
    pass


if __name__ == "__main__":

    mpu = sensors.mpu6050(0x68)
    mpu.set_accel_range(mpu.ACCEL_RANGE_16G)
    mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)
    hat.frequency = 50

    angles = {'x': 0, 'y': 0}
    errors = {'count': 1030, 'x':1.7238, 'y':0.4511, 'z':2.1298}    

    rotor0 = motors.Rotor(0, 50)
    rotor1 = motors.Rotor(1, 50)
    # rotor2 = motors.Rotor(2)
    # rotor3 = motors.Rotor(3)

    # with open('../config.json') as f:
    #     errors_raw = json.load(f)

    # errors = errors_raw['imu']['gyro_bias']

    general_throttle = 50
    epsilon = 10

    x_arr = []
    p_arr = []
    i_arr = []
    d_arr = []
    error_arr = []

    kp = 2
    ki = 0
    kd = 1

    rotor0.thrust(general_throttle)
    rotor1.thrust(general_throttle)

    i = 0 # Initialize integral term
    d = 0 # Initialize derivative term
    d1 = 0
    d2 = 0

    p_usage = 0
    d_usage = 0

    t5 = time.time()

    t9 = time.time()
    while time.time()-t9 < 20:

        try: # Could use d_term as gyro data bit
            sensors.complementary_filter(angles, mpu.get_accel_data(), mpu.get_gyro_data(), errors) # Read value might occur due to high accel value...
        except:
            print("Angle retrieve error...")

        gyro_read = mpu.get_gyro_data()['x']-errors['x']

        try:
            prev_error_y = error_y
        except:
            print("Prev error assign error...")
            prev_error_y = angles['y'] # Temporary change

        error_y = angles['y'] # No subtraction as desired angle is 0, recently added negative 10 due to position error of IMU

        p = proportional(kp, error_y) # Change gain constants to Kp Ki etc.
        # d2 = d1
        # d1 = d
        # d = gyro_read*kd
        # d_term = d


        t6 = time.time()
        d = derivative(kd, gyro_read, t6-t5, d) # To tune the derivative bit, run with low with one arm down, see the response and increase/decrease accordingly.
        t5 = time.time()



        x_arr.append(time.time())
        p_arr.append(p-18)
        i_arr.append(i)
        d_arr.append(d)
        error_arr.append(error_y-9)

        if abs(error_y-9) > epsilon:
            pid = (p)-(kp*9)
            p_usage += 1
            print("P Term")
        else:
            pid = -(d)*4 + (p)-(kp*9)
            d_usage += 1
            print("D Term")

        print(f"PID: {pid}")

        rotor0.thrust(general_throttle + int(pid))
        rotor1.thrust(general_throttle + int(-pid))

    rotor0.thrust(0)
    rotor1.thrust(0)

    print(f"P Usage: {round((p_usage/(p_usage+d_usage))*100, 2)}% --- D Usage: {round((d_usage/(p_usage+d_usage))*100, 2)}%")

    plt.plot(x_arr, p_arr, label="P")
    # plt.plot(x_arr, i_arr, label="I")
    plt.plot(x_arr, d_arr, label="D")
    plt.plot(x_arr, error_arr, label="Error")
    plt.legend()

    plt.show()

    GPIO.cleanup()