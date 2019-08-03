import time
import RPi.GPIO as GPIO 
import threading
import smbus

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
        print(f"THR: {throttle}")
        print(f"PIT: {pitch}")
        print(f"ROL: {roll}")
        print(f"YAW: {yaw}")

        thr = throttle.get_gpio_pin()
        pit = pitch.get_gpio_pin()

        GPIO.add_event_detect(thr, GPIO.BOTH)
        GPIO.add_event_detect(pit, GPIO.BOTH)

        thr_change = 0
        pit_change = 0

        def update_time(control, time_change):
            if GPIO.input(control.get_gpio_pin()) == 1:
                return time.time()
            else: 
                control.set_pwm((time.time()-time_change)*1_000_000)
                return 0

        while True:
            if GPIO.event_detected(thr):
                thr_change = update_time(throttle, thr_change)
            if GPIO.event_detected(pit):
                pit_change = update_time(pitch, pit_change)

            if stop_thread() == True:
                return None

    def calibrate(self):
        print("Set to low")
        while True:
            low = self.read_pwm()
            print(low)
            confirm = input("Enter Y to confirm: ").lower()
            if confirm == 'y':
                self.low_point = low
                break
        print("Set to high")
        while True:
            high = self.read_pwm()
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
    # GPIO.setup(15, GPIO.IN)
    GPIO.setup(18, GPIO.IN)
    # GPIO.setup(23, GPIO.IN)


if __name__ == "__main__":

    setup()
    stop_thread = False

    throttle = Transmitter('THR', 14)

    pitch = Transmitter('ELE', 18)

    # roll = Transmitter('AILE', 15)

    # yaw = Transmitter('RUD', 23)

    update_thread = threading.Thread(target=Transmitter.update_pwm, args=(lambda: stop_thread, throttle, pitch))

    update_thread.start()
   
    for i in range(10):
        print("In the loop")

        print(f"THR: {throttle.get_pwm()}")
        print(f"ELE: {pitch.get_pwm()}")
        # print(f"AILE: {roll.read_pwm()}")
        # print(f"RUD: {yaw.read_pwm()}")

        time.sleep(1)
        
    stop_thread = True

    update_thread.join()
    print("Successful completion")

    GPIO.cleanup()