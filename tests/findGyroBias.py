import sys

sys.path.append('..')

from dfs_func.imu import mpu6050

import time

testTime = int(input("How many seconds to test for: "))

mpu = mpu6050(0x68)

bias = {'x':[], 'y':[], 'z':[]}

print("Keep stationary <--still don't know how to spell that :|")

startTime = time.time()

while time.time()-startTime < testTime:
    gyroRead = mpu.get_gyro_data()
    bias['x'].append(gyroRead['x'])
    bias['y'].append(gyroRead['y'])
    bias['z'].append(gyroRead['z'])

bias['x'] = sum(bias['x'])/len(bias['x'])
bias['y'] = sum(bias['y'])/len(bias['y'])
bias['z'] = sum(bias['z'])/len(bias['z'])

print("Bias X:", bias['x'])
print("Bias Y:", bias['y'])
print("Bias Z:", bias['z'])

print("Test complete.")