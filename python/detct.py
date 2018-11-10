import math

import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as ss

import filter
from Quaternion import*
from DateProcess import*
from Analysis import*

ax, ay, az = [], [], []
gx, gy, gz = [], [], []
frq = 100

with open('data/040.txt', 'r') as f:
    list1 = f.readlines()
    print(len(list1))
    for i in range(len(list1)):
        list1[i] = list1[i].rstrip('\n')
        sp = list1[i].split()

        ax.append(float(sp[1])/8192*9.81)
        ay.append(float(sp[0])/8192*9.81)
        az.append(-float(sp[2])/8192*9.81)
        # 弧度制
        gx.append(float(sp[4])/(57.3*16.4))
        gy.append(float(sp[3])/(57.3*16.4))
        gz.append(-float(sp[5])/(57.3*16.4))

ax_Mean = filter.Mean_Filter(ax, 4)
ay_Mean = filter.Mean_Filter(ay, 4)
az_Mean = filter.Mean_Filter(az, 4)
gx_Mean = filter.Mean_Filter(gx, 4)
gy_Mean = filter.Mean_Filter(gy, 4)
gz_Mean = filter.Mean_Filter(gz, 4)

ax_KF = filter.Kalman_Filter(ax_Mean, 1e-6, 1e-5, -1, 1)
ay_KF = filter.Kalman_Filter(ay_Mean, 1e-6, 1e-5, -1, 1)
az_KF = filter.Kalman_Filter(az_Mean, 1e-6, 1e-5, -1, 1)
gx_KF = filter.Kalman_Filter(gx_Mean, 1e-6, 1e-5, -1, 1)
gy_KF = filter.Kalman_Filter(gy_Mean, 1e-6, 1e-5, -1, 1)
gz_KF = filter.Kalman_Filter(gz_Mean, 1e-6, 1e-5, -1, 1)

aa = Analysis(ax_KF,ay_KF,az_KF,gx_KF,gy_KF,gz_KF,frq)

route = aa.countstep()
supportrate = aa.supportrate()
steplength = aa.steplength()
rollanlge = aa.rollangle()
# """
# GUI
# """
plt.figure("相对加速度时间图像")
plt.plot(ax_KF, 'b', label='X_Axis')
plt.plot(ay_KF, 'orange', label='Y_Axis')
plt.plot(az_KF, 'r', label='Z_Axis')
plt.legend()

plt.figure("绝对加速度时间图像(Q4方法)")
plt.plot(aa.Ax, 'b', label='X_Axis')
plt.plot(aa.Ay, 'orange', label='Y_Axis')
plt.plot(aa.Az, 'r', label='Z_Axis')
# plt.scatter(points_zerorise_index,pointszero_rise)
plt.legend()

plt.figure("速度时间图像")
plt.plot(aa.Vx, 'b', label='X_Axis')
plt.plot(aa.Vy, 'orange', label='Y_Axis')
plt.plot(aa.Vz, 'r', label='Z_Axis')
plt.legend()

plt.figure("位移时间图像")
plt.plot(aa.Sx, 'b', label='X_Axis')
plt.plot(aa.Sy, 'orange', label='Y_Axis')
plt.plot(aa.Sz, 'r', label='Z_Axis')
plt.legend()

plt.figure("姿态时间图像")
plt.subplot(3, 1, 1)
plt.title('pitch')
plt.plot(aa.pitch, 'b', label='pitch')
# plt.scatter(pitch_valley_index,pitch_valley,c='red')
# plt.scatter(pitch_peak_index,pitch_peak,c='green')
# plt.scatter(pitch_tinypeak_index,pitch_tinypeak,c='pink')
# plt.scatter(pitch_nearzeros_index,pitch_nearzeros,c='orange')
# plt.scatter(pitch_risezero_index,pitch_risezero,c='black')
# plt.scatter(pitch_fallzero_index,pitch_fallzero,c='gray')
pts = [aa.pitch[i] for i in route]
plt.scatter(route,pts,c='purple')

plt.subplot(3, 1, 2)
plt.title('yaw')
plt.plot(aa.yaw, 'orange', label='yaw')
plt.subplot(3, 1, 3)
plt.title('roll')
plt.plot(aa.roll, 'green', label='roll')

plt.show()
