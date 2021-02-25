import time
import RPi.GPIO as GPIO 
import threading
import smbus
import json

from transceiver import Transmitter

def setup():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(14, GPIO.IN)
    GPIO.setup(15, GPIO.IN)
    GPIO.setup(18, GPIO.IN)
    GPIO.setup(23, GPIO.IN)


if __name__ == "__main__":

    setup()
    stop_thread = False

    with open('../config.json') as f:
        config = json.load(f)
    
    thr_config = config['transmitter']['THR']

    throttle = Transmitter('THR', 14, thr_config['pwm_low'], thr_config['pwm_high'])
    pitch = Transmitter('ELE', 18)
    roll = Transmitter('AIL', 15)
    yaw = Transmitter('RUD', 23)

    update_thread = threading.Thread(target=Transmitter.update_pwm, args=(lambda: stop_thread, throttle, pitch, roll, yaw))
    update_thread.start()

    # Wait for first PWM signals to be read to avoid first read being 1000
    time.sleep(0.5)

    throttle.calibrate('../config.json')
    # pitch.calibrate()
    # roll.calibrate()
    # yaw.calibrate()

    x_arr = []
    y_arr = []

    for i in range(20):
        print("In the loop")
        # Can result in zero division error
        throt = (round(throttle.get_pwm(), 0) - throttle.get_low_point()) / (throttle.get_high_point()-throttle.get_low_point()) * 100

        print(f"THR: {throt}")
        print(f"ELE: {pitch.get_pwm()}")
        print(f"AIL: {roll.get_pwm()}")
        print(f"RUD: {yaw.get_pwm()}")

        x_arr.append(time.time())
        y_arr.append(round(throttle.get_pwm(), 0))

        time.sleep(1)
        
    stop_thread = True

    update_thread.join()
    print("Successful completion")

    # plt.plot(x_arr, y_arr)
    # plt.ylabel('Y-Axis')
    # plt.xlabel('X-Axis')
    # plt.show()

    GPIO.cleanup()