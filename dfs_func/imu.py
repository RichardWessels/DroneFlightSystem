# IMU Test 2
import time
import smbus
from numpy import arcsin, arccos, arctan, rad2deg # Since it's pretty easy conversion, maybe solve without external help.
import matplotlib.pyplot as plt


# Not my stuff
class mpu6050:

    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        time.sleep(1)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    # I2C communication methods

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.
        register -- the first register to read from.
        Returns the combined read results.
        """
        
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    # MPU-6050 Methods

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.
        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.
        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False):
        """Gets and returns the X, Y and Z values from the accelerometer.
        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unknown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.
        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to.
        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.
        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unkown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return [accel, gyro]


# My stuff

def getGyro(mpu):
    gyro_data = mpu.get_gyro_data()
    return [gyro_data['x'], gyro_data['y'], gyro_data['z']]

def calibrate_gyro():
    pass

def complementary_filter(angles, accel_raw, gyro_raw, gyro_bias):
    '''
    All keys are expected to be x, y, z
    '''
    gyro_vel = {'x':0, 'y':0, 'z':0}
    comp_dis = {'x': int, 'y': int}
    accel = {'x':0, 'y':0, 'z':0}

    gyro_vel['x'] = gyro_raw['x']
    gyro_vel['y'] = gyro_raw['y']
    gyro_vel['z'] = gyro_raw['z']

    accel['x'] = accel_raw['x']
    accel['y'] = accel_raw['y']
    accel['z'] = accel_raw['z']

    # X
    result = (accel['y']**2+accel['z']**2)**0.5
    result = accel['x']/result
    accel_angle_x = rad2deg(arctan(result))

    # Y
    result = (accel['x']**2+accel['z']**2)**0.5
    result = accel['y']/result
    accel_angle_y = rad2deg(arctan(result))

    angles['x'] = 0.02*(round(accel_angle_x, 2)) + 0.98*(angles['x']+round((gyro_vel['x']-gyro_bias['x'])*1/90, 2)) # Using a constant for frequency, for accuracy, can use time later
    angles['y'] = 0.02*(round(accel_angle_y, 2)) + 0.98*(angles['y']+(gyro_vel['y']-gyro_bias['y'])*1/90) # Removed gyro rounding 

    # return (comp_dis['x'], comp_dis['y'])

if __name__ == "__main__":

    averageMPUt = 0
    averageCompt = 0
    loopt = 0

    
    mpu = mpu6050(0x68)
    mpu.set_accel_range(mpu.ACCEL_RANGE_16G)
    mpu.set_gyro_range(mpu.GYRO_RANGE_2000DEG)

    errors = {'count': 1030, 'x':1.72, 'y':0.55, 'z':1.34}

    t9 = time.time()

    angles = {'x': 0, 'y': 0}

    gyroArr = []
    xArr = []

    timeStep = time.time()

    counter = 0

    while time.time() -t9 < 30:
        t5 = time.time()
        t1 = time.time()
        accel_data = mpu.get_accel_data()
        gyro_data = mpu.get_gyro_data()
        
        averageMPUt += time.time()-t1

        t2 = time.time()
        complementary_filter(angles, accel_data, gyro_data, errors)
        averageCompt += time.time()-t2
        

        if time.time()-timeStep > 0.01:
            print(f"X: {angles['x']}")
            print(f"Y: {angles['y']}")
            timeStep = time.time()
        
        counter += 1
        loopt = time.time()-t5
        gyroArr.append(gyro_data['y'])
        xArr.append(counter)

    print(f"Frequency is {counter/30}hz")
    print(f"Average MPU time is {averageMPUt/counter}")
    print(f"Average Comp time is {averageCompt/counter}")
    print(f"Average loop time is {loopt/counter}")

    plt.plot(xArr, gyroArr)
    plt.show()




























    # mpu = mpu6050(0x68)

    # print("Keep stationary <--- if that's how you spell it...")
    # t9 = time.time()
    # # In actual program, call errors, bias.
    # # errors = {'count': 1030, 'x':-3.012, 'y':0.387, 'z':1.213}
    # errors = {'count': 1030, 'x':1.7238, 'y':0.4511, 'z':2.1298}
    # error_find_x = []
    # error_find_y = []
    # error_find_z = []

    # gyro_vel = {'x':0, 'y':0, 'z':0}
    # gyro_dis = {'x':0, 'y':0, 'z':0}
    # comp_dis = {'x':0, 'y':0, 'z':0}

    # t9 = time.time()
    # while time.time()-t9 < 30:

        
    #     accel_data = mpu.get_accel_data()
    #     gyro_data = mpu.get_gyro_data()

    #     error_find_x.append(gyro_data['x'])
    #     error_find_y.append(gyro_data['y'])
    #     error_find_z.append(gyro_data['z'])

    #     gyro_vel['x'] = gyro_data['x']
    #     gyro_vel['y'] = gyro_data['y']
    #     gyro_vel['z'] = gyro_data['z']

    #     gyro_dis['x'] += round((gyro_vel['x']-errors['x'])*1/180, 2)
    #     gyro_dis['y'] += round((gyro_vel['y']-errors['y'])*1/180, 2)
    #     gyro_dis['z'] += round((gyro_vel['z']-errors['z'])*1/180, 2)

    #     x = accel_data['x']
    #     y = accel_data['y']
    #     z = accel_data['z']

    #     # X
    #     result = (y**2+z**2)**0.5
    #     result = x/result
    #     accel_angle_x = rad2deg(arctan(result))

    #     # Y
    #     result = (x**2+z**2)**0.5
    #     result = y/result
    #     accel_angle_y = rad2deg(arctan(result))

    #     # comp_dis['x'] += round((gyro_vel['x']-errors['x'])*1/180, 2)
    #     # comp_dis['y'] += round((gyro_vel['x']-errors['x'])*1/180, 2)
    #     # comp_dis['z'] += round((gyro_vel['z']-errors['z'])*1/180, 2)

    #     # Actual complementary filter (seems to be working...), for video, go through the detailed math.
    #     print(f"Accel angle is {round(accel_angle_y, 2)}")
    #     print(f"Actual Gyro is {gyro_dis['y']}")
    #     print(f"Gyro angle is {comp_dis['y']+round((gyro_vel['y']-errors['y'])*1/90, 2)}")
    #     # comp_dis['x'] = 0.04*(round(accel_angle_x, 2)) + 0.96*(comp_dis['x']+round((gyro_vel['x']-errors['x'])*1/180, 2))
    #     comp_dis['y'] = 0.04*(round(accel_angle_y, 2)) + 0.96*(comp_dis['y']+round((gyro_vel['y']-errors['y'])*1/180, 2))
    #     print(comp_dis['y'])

    # # comp_dis['x'] += 0.04*(round(accel_angle_x, 2)) + 0.96*(round((gyro_vel['x']-errors['x'])*1/180, 2))
    # # With change
    # # comp_dis['x'] = 0.04*(round(accel_angle_x, 2)) + 0.96*(comp_dis['x']+round((gyro_vel['x']-errors['x'])*1/180, 2))
    # comp_dis['y'] = 0.04*(round(accel_angle_y, 2)) + 0.96*(comp_dis['y']+round((gyro_vel['y']-errors['y'])*1/180, 2))
    # comp_dis['z'] += round((gyro_vel['z']-errors['z'])*1/180, 2)

    # print(comp_dis)
    # print(gyro_dis)

    # print("Average X error")
    # print(sum(error_find_x)/len(error_find_x))
    # print("Average Y error")
    # print(sum(error_find_y)/len(error_find_y))
    # print("Average Z error")
    # print(sum(error_find_z)/len(error_find_z))