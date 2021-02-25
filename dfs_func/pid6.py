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
    return constrain(previous_g + delta_time / (rc + delta_time) * (new_delta-previous_g), -50, 50)


def applyPID(desired_angle, actual_angle, gyro, kp, ki, kd, i, d, time_end, time_begin): # TEN PARAMETERS, pretty sure this is a bad practice :(

    gyro_read = gyro

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

    throttle_limit = 75

    rotor0 = motors.Rotor(0, throttle_limit)
    rotor1 = motors.Rotor(1, throttle_limit)
    rotor2 = motors.Rotor(2, throttle_limit)
    rotor3 = motors.Rotor(3, throttle_limit)

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
    general_throttle = 85

    comp_t_begin = time.time()
    t1 = time.time()

    movement_dict = {'pitch': [[-5, -3], [0, 0], [8, -15]], 'roll': [[-5, -15], [0, 0], [8, -2]]}

    while time.time()-t1 < 15:
        try:
            comp_t_end = time.time()
            imu.complementary_filter(angles, mpu.get_accel_data(), mpu.get_gyro_data(), errors, comp_t_end-comp_t_begin)
            comp_t_begin = time.time()

            roll = 1
            pitch = 1

            desired_x = movement_dict['pitch'][pitch][0]+movement_dict['roll'][roll][0]
            desired_y = movement_dict['pitch'][pitch][1]+movement_dict['roll'][roll][1]

            pid_val_x = applyPID(desired_x, angles['x'], mpu.get_gyro_data()['x']-errors['x'], kp, ki, kd, i, d, time_end, time_begin)
            pid_val_y = applyPID(desired_y, angles['y']+8, mpu.get_gyro_data()['y']-errors['y'], kp, ki, kd, i, d, time_end, time_begin)

            print(f"PID X: {pid_val_x} ::: ERROR X: {desired_x-angles['x']}")
            print(f"PID Y: {pid_val_y} ::: ERROR Y: {desired_y-angles['y']}")
            print(f"ANGLES :: X: {angles['x']} || Y: {angles['y']}")
            print(f"DESIRED :: X: {desired_x} || Y: {desired_y}")

            rotor0.thrust(general_throttle+pid_val_x)
            rotor1.thrust(general_throttle-pid_val_x)
            rotor2.thrust(general_throttle+pid_val_y)
            rotor3.thrust(general_throttle-pid_val_y)
        except Exception as e:
            print(e)
            print("Error encountered, shutting off motors.")
            rotor0.thrust(0)
            rotor1.thrust(0)
            rotor2.thrust(0)
            rotor3.thrust(0)
            break


    rotor0.thrust(0)
    rotor1.thrust(0)
    rotor2.thrust(0)
    rotor3.thrust(0)

    GPIO.cleanup()