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

if __name__ == "__main__":
    mpu = imu.mpu6050(0x68)
    hat.frequency = 50
    mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)

    angles = {'x': 0, 'y': 0}
    errors = {'count': 1030, 'x':1.72, 'y':0.55, 'z':1.34}    

    rotor0 = motors.Rotor(0, 50)
    rotor1 = motors.Rotor(1, 50)
    rotor2 = motors.Rotor(2, 50)
    rotor3 = motors.Rotor(3, 50)

    def constrain(value, minimum, maximum):
        return max(min(maximum, value), minimum)

    general_throttle = 50
    p = 10

    rotor0.thrust(general_throttle)
    rotor1.thrust(general_throttle)
    rotor2.thrust(general_throttle)
    rotor3.thrust(general_throttle)

    readTimeAvg = 0
    run_time = 20
    counter = 0
    moAvg1 = 0
    moAvg2 = 0
    gyro_read = {'x':0, 'y':0, 'z':0}
    t1 = time.time()
    x_arr = []
    axis_x_arr = []
    axis_y_arr = []
    prev = 1
    t6 = time.time()

    d_x = 0
    d_y = 0
    t5 = time.time()

    while time.time()-t5 < run_time:
        t3 = time.time()
        gyro_read['x'] = mpu.get_gyro_data()['x']-errors['x']
        gyro_read['y'] = mpu.get_gyro_data()['y']-errors['y']
        readTimeAvg += time.time()-t3

        rotor0.thrust(general_throttle+gyro_read['x'])
        rotor1.thrust(general_throttle-gyro_read['x'])
        rotor2.thrust(general_throttle+gyro_read['y'])
        rotor3.thrust(general_throttle-gyro_read['y'])
        counter += 1
        x_arr.append(counter)
        axis_x_arr.append(gyro_read['x'])
        axis_y_arr.append(gyro_read['y'])

    print("Loop Frequency:", counter/run_time)
    print("Read Time Average:", readTimeAvg/counter)
    rotor0.thrust(0)
    rotor1.thrust(0)
    rotor2.thrust(0)
    rotor3.thrust(0)
    plt.plot(x_arr, axis_x_arr)
    plt.plot(x_arr, axis_y_arr)
    plt.show()