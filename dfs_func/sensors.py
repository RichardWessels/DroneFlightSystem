import time
import RPi.GPIO as GPIO 
import threading
import smbus
import matplotlib.pyplot as plt

class Transmitter:

    def __init__(self, control, gpio_pin, low_point=1000, high_point=2000):
        self.control_type = control
        self.gpio_pin = gpio_pin
        self.low_point = low_point
        self.mid_point = (high_point + low_point) // 2
        self.high_point = high_point
        self.recent_read = low_point
        print(f"Created instance of Transmitter. Control: {self.control_type}, GPIO Pin: {self.gpio_pin}, LOW: {self.low_point}, HIGH: {self.high_point}")

    @classmethod
    def update_pwm(cls, stop_thread, throttle=None, pitch=None, roll=None, yaw=None):
        # print(f"THR: {throttle}")
        # print(f"PIT: {pitch}")
        # print(f"ROL: {roll}")
        # print(f"YAW: {yaw}")

        throttle_pin = throttle.get_gpio_pin()
        pitch_pin = pitch.get_gpio_pin()
        roll_pin = roll.get_gpio_pin()
        yaw_pin = yaw.get_gpio_pin()

        GPIO.add_event_detect(throttle_pin, GPIO.BOTH)
        GPIO.add_event_detect(pitch_pin, GPIO.BOTH)
        GPIO.add_event_detect(roll_pin, GPIO.BOTH)
        GPIO.add_event_detect(yaw_pin, GPIO.BOTH)

        thr_change = 0
        thr_prev = 0
        pit_change = 0
        roll_change = 0
        yaw_change = 0

        def update_time(control, time_change, prev):
            # Figuring out the continuous values
            if GPIO.input(control.get_gpio_pin()) == prev:
                return (time_change, prev)
            if prev == 1:
                prev = 0
            else:
                prev = 1
            if GPIO.input(control.get_gpio_pin()) == 1:
                # print("1")
                return (time.time(), prev)
            elif GPIO.input(control.get_gpio_pin()) == 0: 
                pulse_length = (time.time()-time_change)*1_000_000
                if 1000 <= pulse_length <= 2000:
                    control.set_pwm((time.time()-time_change)*1_000_000)
                # print("0")
                return (0, prev)
            else:
                # print(f"Dec:{GPIO.input(control.get_gpio_pin())}")
                return (time_change, prev)
        # Just finding frequency, delete this later
        # signal_length = 0
        while True:
            if GPIO.event_detected(throttle_pin, debounce=0.001):
                (thr_change, thr_prev) = update_time(throttle, thr_change, thr_prev)
                # if thr_change != 0:
                #     print(thr_change-signal_length)
                    # signal_length = thr_change
            # if GPIO.event_detected(pitch_pin):
            #     pit_change = update_time(pitch, pit_change)
            # if GPIO.event_detected(roll_pin):
            #     roll_change = update_time(roll, roll_change)
            # if GPIO.event_detected(yaw_pin):
            #     yaw_change = update_time(yaw, yaw_change)

            if stop_thread() == True:
                return None

    def calibrate(self):
        print("Calibrating", self.get_control_type())
        print("Set to low")
        while True:
            low = self.get_pwm()
            print(low)
            confirm = input("Enter Y to confirm: ").lower()
            if confirm == 'y':
                self.low_point = low
                break
        print("Set to high")
        while True:
            high = self.get_pwm()
            print(high)
            confirm = input("Enter Y to confirm: ").lower()
            if confirm == 'y':
                self.high_point = high
                break
        self.mid_point = (self.high_point + self.low_point) // 2

    def set_pwm(self, pwm):
        self.recent_read = pwm

    def get_pwm(self):
        return self.recent_read
        
    def get_control_type(self):
        return self.control_type

    def get_gpio_pin(self):
        return self.gpio_pin

    def get_low_point(self):
        return self.low_point

    def get_high_point(self):
        return self.high_point

class IMU:

    def __init__(self):
        print("IMU object that contains accelerometer and gyroscope methods")


def setup():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(14, GPIO.IN)
    GPIO.setup(15, GPIO.IN)
    GPIO.setup(18, GPIO.IN)
    GPIO.setup(23, GPIO.IN)


if __name__ == "__main__":

    setup()
    stop_thread = False

    throttle = Transmitter('THR', 14)
    pitch = Transmitter('ELE', 18)
    roll = Transmitter('AILE', 15)
    yaw = Transmitter('RUD', 23)

    update_thread = threading.Thread(target=Transmitter.update_pwm, args=(lambda: stop_thread, throttle, pitch, roll, yaw))
    update_thread.start()

    # Wait for first PWM signals to be read to avoid first read being 1000
    time.sleep(0.5)

    throttle.calibrate()
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
        print(f"AILE: {roll.get_pwm()}")
        print(f"RUD: {yaw.get_pwm()}")

        x_arr.append(time.time())
        y_arr.append(round(throttle.get_pwm(), 0))

        time.sleep(1)
        
    stop_thread = True

    update_thread.join()
    print("Successful completion")

    plt.plot(x_arr, y_arr)
    plt.ylabel('Y-Axis')
    plt.xlabel('X-Axis')
    plt.show()

    GPIO.cleanup()