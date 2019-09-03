import dfs_func.motors as motors
import dfs_func.sensors as sensors

import time

# Motors import
from adafruit_servokit import ServoKit # Not sure if this is needed
import board
import busio
import adafruit_pca9685

# Sensors import
import RPi.GPIO as GPIO 
import threading

# Motors init
i2c = busio.I2C(board.SCL, board.SDA)
hat = adafruit_pca9685.PCA9685(i2c)

if __name__ == '__main__':

    print("Running main function.")

    sensors.setup()

    stop_thread = False

    throttle = sensors.Transmitter('THR', 14)

    update_thread = threading.Thread(target=sensors.Transmitter.update_pwm, args=(lambda: stop_thread, throttle))
    update_thread.start()

    # Wait for first PWM signals to be read to avoid first read being 1000
    time.sleep(0.5)

    throttle.calibrate()

    hat.frequency = 50
    
    rotor1 = motors.Rotor(0)
    
    rotor1.thrust(0)
    time.sleep(1)

    t9 = time.time()
    while time.time()-t9 < 10:
        percent_thr = (throttle.get_pwm()-throttle.get_low_point())/(throttle.get_high_point()-throttle.get_low_point())*100

        rotor1.thrust(int(percent_thr))

    stop_thread = True
    update_thread.join()
    print("Successful transmitter completion")
    GPIO.cleanup()