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

    import imu2 as imu
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

    mpu = imu.mpu6050(0x68)
    mpu.set_accel_range(mpu.ACCEL_RANGE_16G)
    mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)
    hat.frequency = 50

    angles = {'x': 0, 'y': 0}
    errors = {'count': 1030, 'x':1.7238, 'y':0.4511, 'z':2.1298}    

    rotor0 = motors.Rotor(0, 50)
    rotor1 = motors.Rotor(1, 50)
    rotor2 = motors.Rotor(2, 50)
    rotor3 = motors.Rotor(3, 50)

    movement = motors.MotorMovement(rotor0, rotor1, rotor2, rotor3)


    kp = 2
    ki = 3
    kd = 1

    # Integral and Derivative Terms Init
    i = 0
    d = 0
    time_end = time.time()
    time_begin = time.time()

    angles = {'x': 0, 'y': 0}
    d_ang_1 = 0
    d_ang_2 = 0
    d_ang_3 = 0
    d_ang_4 = 0
    desired_angle = 0
    general_throttle = 50

    comp_t_begin = time.time()
    t1 = time.time()

    while time.time()-t1 < 15:
        comp_t_end = time.time()
        imu.complementary_filter(angles, mpu.get_accel_data(), mpu.get_gyro_data(), errors, comp_t_end-comp_t_begin)
        comp_t_begin = time.time()

        d_ang_4 = d_ang_3
        d_ang_3 = d_ang_2
        d_ang_2 = d_ang_1
        d_ang_1 = desired_angle

        pid_val = applyPID(desired_angle, angles['y'], mpu.get_gyro_data()['y']-errors['y'], kp, ki, kd, i, d, time_end, time_begin)

        rotor0.thrust(general_throttle+pid_val)
        rotor1.thrust(general_throttle-pid_val)


    rotor0.thrust(0)
    rotor1.thrust(0)


    GPIO.cleanup()