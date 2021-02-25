# Where the craziness lies

import time

# Not sure if I want to use a class or function. The issue is that the angle updates outside of this and this function must have up to date info

def constrain(value, minimum, maximum):
    return max(min(maximum, value), minimum)

def proportional(p_const, error):
    return p_const*error

def integral(i_const, error, delta_time): # Rewrite as one PID function
    # return error*delta_time       # Apparently don't need delta time
    return error*delta_time*i_const

def derivative(d_const, gyro_d, delta_time, previous_g):
    new_delta = (gyro_d)
    freq_cut = 64
    rc = 1 / (2*3.14*freq_cut)
    print((previous_g + delta_time / (rc + delta_time) * (new_delta-previous_g)))
    return constrain(previous_g + delta_time / (rc + delta_time) * (new_delta-previous_g), -50, 50)


def applyPID(desired_angle, actual_angle, gyro, kp, ki, kd, i, d, time_end, time_begin): # TEN PARAMETERS, pretty sure this is a bad practice :(

    gyro_read = gyro

    try:
        prev_error = error
    except:
        print("Prev error assign error...")
        prev_error = actual_angle-desired_angle

    error = actual_angle-desired_angle

    p = proportional(kp, error)

    time_end = time.time()
    d = derivative(kd, gyro_read, time_end-time_begin, d)
    i += integral(ki, error, time_end-time_begin)
    i = constrain(i, -10, 10)
    time_begin = time.time()

    d_term = d*(5/(abs(error)+5))

    if abs(error) < 10:
        pid = p+i+d_term
    else:
        i = 0
        pid = p+d_term

    return pid


if __name__ == "__main__":

    import imu2
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

    mpu = imu2.mpu6050(0x68)
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
    desired_angle = -10

    x_arr = []
    p_arr = []
    i_arr = []
    d_arr = []
    error_arr = []
    pid_arr = []

    kp = 2
    ki = 3
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

    t11 = time.time()

    t9 = time.time()
    while time.time()-t9 < 20:

        try: # Could use d_term as gyro data bit
            t12 = time.time()
            imu2.complementary_filter(angles, mpu.get_accel_data(), mpu.get_gyro_data(), errors, t12-t11) # Read value might occur due to high accel value...
            t11 = time.time()
            print(f"ANGLE X: {angles['x']}")
        except:
            print("Angle retrieve error...")

        gyro_read = mpu.get_gyro_data()['x']-errors['x']

        try:
            prev_error_y = error_y
        except:
            print("Prev error assign error...")
            prev_error_y = angles['x']-desired_angle

        error_y = angles['x']-desired_angle

        p = proportional(kp, error_y)

        t6 = time.time()
        d = derivative(kd, gyro_read, t6-t5, d)
        i += integral(ki, error_y, t6-t5)
        i = constrain(i, -10, 10)
        t5 = time.time()

        d_term = d*(5/(abs(error_y)+5))

        if abs(error_y) < 10:
            pid = p+i+d_term
        else:
            i = 0
            pid = p+d_term

        print(f"PID: {pid}")

        rotor0.thrust(general_throttle + int(-pid))
        rotor1.thrust(general_throttle + int(+pid))

        x_arr.append(time.time())
        p_arr.append(p)
        i_arr.append(i)
        d_arr.append(d_term)
        error_arr.append(error_y)
        pid_arr.append(pid)

    rotor0.thrust(0)
    rotor1.thrust(0)

    plt.plot(x_arr, p_arr, label="P")
    plt.plot(x_arr, i_arr, label="I")
    plt.plot(x_arr, d_arr, label="D")
    plt.plot(x_arr, error_arr, label="Error")
    plt.plot(x_arr, pid_arr, label="PID")
    plt.legend()

    plt.show()

    GPIO.cleanup()