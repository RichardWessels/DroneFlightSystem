import time
import RPi.GPIO as GPIO
import threading
import json

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
        pit_prev = 0

        roll_change = 0
        roll_prev = 0

        yaw_change = 0
        yaw_prev = 0

        def update_time(control, time_change, prev):
            # Figuring out the continuous values
            if GPIO.input(control.get_gpio_pin()) == prev:
                return (time_change, prev)
            if prev == 1:
                prev = 0
            else:
                prev = 1
            if GPIO.input(control.get_gpio_pin()) == 1:
                return (time.time(), prev)
            elif GPIO.input(control.get_gpio_pin()) == 0: 
                pulse_length = (time.time()-time_change)*1_000_000
                if 1000 <= pulse_length <= 2000:
                    control.set_pwm((time.time()-time_change)*1_000_000)
                return (0, prev)
            else:
                return (time_change, prev)
        # Just finding frequency, delete this later
        # signal_length = 0
        while True:
            if GPIO.event_detected(throttle_pin):
                (thr_change, thr_prev) = update_time(throttle, thr_change, thr_prev)
            if GPIO.event_detected(pitch_pin):
                (pit_change, pit_prev) = update_time(pitch, pit_change, pit_prev)
            if GPIO.event_detected(roll_pin):
                (roll_change, roll_prev) = update_time(roll, roll_change, roll_prev)
            if GPIO.event_detected(yaw_pin):
                (yaw_change, yaw_prev) = update_time(yaw, yaw_change, yaw_prev)

            if stop_thread() == True:
                return None

    def calibrate(self, config_file):
        print("Calibrating", self.get_control_type())
        print("Set to low")
        while True:
            low = round(self.get_pwm(), 2)
            print(low)
            confirm = input("Enter Y to confirm: ").lower()
            if confirm == 'y':
                self.low_point = low
                break
        print("Set to high")
        while True:
            high = round(self.get_pwm(), 2)
            print(high)
            confirm = input("Enter Y to confirm: ").lower()
            if confirm == 'y':
                self.high_point = high
                break
        self.mid_point = round((self.high_point + self.low_point)/2, 2)
        write_file = input("Do you want to write changes to config file (y/n): ").lower()
        if write_file == 'y':
            print(f"Config file is {config_file}")
            # Read current data and edit values
            with open(config_file, 'r') as f:
                config = json.load(f)
                config['transmitter'][self.get_control_type()]['pwm_high'] = self.high_point
                config['transmitter'][self.get_control_type()]['pwm_mid'] = self.mid_point
                config['transmitter'][self.get_control_type()]['pwm_low'] = self.low_point
                print(config)
            # Write to JSON file
            with open(config_file, 'w') as f:
                json.dump(config, f, indent=2)

    def set_pwm(self, pwm):
        self.recent_read = pwm

    def get_pwm(self):
        return self.recent_read

    def get_percent(self):
        percent = ((self.get_pwm()-self.get_low_point())/(self.get_high_point()-self.get_low_point()))*100
        if percent < 0:
            percent = 0
        if percent > 100:
            percent = 100
        return percent
        
    def get_control_type(self):
        return self.control_type

    def get_gpio_pin(self):
        return self.gpio_pin

    def get_low_point(self):
        return self.low_point

    def get_high_point(self):
        return self.high_point

THROTTLE_PIN = 4
PITCH_PIN = 18
ROLL_PIN = 17
YAW_PIN = 27

def setup():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(THROTTLE_PIN, GPIO.IN)
    GPIO.setup(PITCH_PIN, GPIO.IN)
    GPIO.setup(ROLL_PIN, GPIO.IN)
    GPIO.setup(YAW_PIN, GPIO.IN)


if __name__ == "__main__":

    setup()

    stop_thread = False

    throttle = Transmitter('THR', THROTTLE_PIN)
    pitch = Transmitter('PIT', PITCH_PIN)
    roll = Transmitter('ROL', ROLL_PIN)
    yaw = Transmitter('YAW', YAW_PIN)

    update_thread = threading.Thread(target=Transmitter.update_pwm, args=(lambda: stop_thread, throttle, pitch, roll, yaw))
    update_thread.start()

    t1 = time.time()
    while time.time()-t1 < 20:
        print(f"THR: {throttle.get_pwm()} :: {throttle.get_gpio_pin()}")
        print(f"PIT: {pitch.get_pwm()} :: {pitch.get_gpio_pin()}")
        print(f"ROL: {roll.get_pwm()} :: {roll.get_gpio_pin()}")
        print(f"YAW: {yaw.get_pwm()} :: {yaw.get_gpio_pin()}")

    stop_thread = True
    update_thread.join()
    print("Successful transmitter completion")
    GPIO.cleanup()