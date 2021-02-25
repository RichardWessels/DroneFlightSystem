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

if __name__ == "__main__":
    mpu = sensors.mpu6050(0x68)
    hat.frequency = 50
    mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)

    angles = {'x': 0, 'y': 0}
    errors = {'count': 1030, 'x':1.72, 'y':0.55, 'z':1.34}    

    rotor0 = motors.Rotor(0, 50)
    rotor1 = motors.Rotor(1, 50)
    # rotor2 = motors.Rotor(2)
    # rotor3 = motors.Rotor(3)

    # with open('../config.json') as f:
    #     errors_raw = json.load(f)

    # errors = errors_raw['imu']['gyro_bias']

    def constrain(value, minimum, maximum):
        return max(min(maximum, value), minimum)

    def derivative(d_const, gyro_d, delta_time, previous_g):
        new_delta = (gyro_d)
        freq_cut = 16
        rc = 1 / (2*3.14*freq_cut)
        print((previous_g + delta_time / (rc + delta_time) * (new_delta-previous_g)))
        return constrain(previous_g + delta_time / (rc + delta_time) * (new_delta-previous_g), -50, 50)

    general_throttle = 50
    p = 10

    rotor0.thrust(general_throttle)
    rotor1.thrust(general_throttle)

    readTimeAvg = 0
    run_time = 20
    counter = 0
    moAvg1 = 0
    moAvg2 = 0
    gyro_read = {'x':0, 'y':0, 'z':0}
    t1 = time.time()
    x_arr = []
    y_arr = []
    prev = 1
    t6 = time.time()

    d = 0
    t5 = time.time()

    while time.time()-t1 < run_time:
        t3 = time.time()
        gyro_read = mpu.get_gyro_data()['x']-errors['x']
        readTimeAvg += time.time()-t3
        # p_term = int(gyro_read['y']-1)
        # p_term = gyro_read['x']

        t6 = time.time()
        d = derivative(1, gyro_read, t6-t5, d) # To tune the derivative bit, run with low with one arm down, see the response and increase/decrease accordingly.
        t5 = time.time()


        # total = (p_term)
        rotor0.thrust(general_throttle-(d*1.5))
        rotor1.thrust(general_throttle+(d*1.5))
        counter += 1
        # print(total)
        # moAvg2 = moAvg1
        # moAvg1 = p_term
        x_arr.append(counter)
        y_arr.append(d)





    print("Loop Frequency:", counter/run_time)
    print("Read Time Average:", readTimeAvg/counter)
    rotor0.thrust(0)
    rotor1.thrust(0)
    plt.plot(x_arr, y_arr)
    plt.show()


