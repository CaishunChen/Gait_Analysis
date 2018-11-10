import numpy as np
import math
class Quaternion(object):
    def __init__(self, Kp, Ki, T):
        self.Kp = Kp
        self.Ki = Ki
        self.halfT = T/2
        self.exInt, self.eyInt, self.ezInt = 0, 0, 0
        self.q0, self.q1, self.q2, self.q3 = 1, 0, 0, 0
        self.beta = 0.1

    def getAngle(self, ax, ay, az, gx, gy, gz):
        vx, vy, vz = 0, 0, 0
        ex, ey, ez = 0, 0, 0
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        halfT = self.halfT
        exInt, eyInt, ezInt = self.exInt, self.eyInt, self.ezInt

        q0q0 = q0**2
        q0q1 = q0*q1
        q0q2 = q0*q2
        q1q1 = q1**2
        q1q3 = q1*q3
        q2q2 = q2**2
        q2q3 = q2*q3
        q3q3 = q3**2

        if ax*ay*az == 0:  # 加计处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
            return 0, 0, 0

        # 单位化加速度计
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        # 这样变更了量程也不需要修改KP参数，因为这里归一化了
        ax = ax / norm
        ay = ay / norm
        az = az / norm

        # 用当前姿态计算出重力在三个轴上的分量，
        # 参考坐标n系转化到载体坐标b系的用四元数表示的方向余弦矩阵第三列即是

        vx = 2*(q1q3 - q0q2)
        vy = 2*(q0q1 + q2q3)
        vz = q0q0 - q1q1 - q2q2 + q3q3

        # 计算测得的重力与计算得重力间的误差，向量外积可以表示这一误差
        ex = (ay*vz - az*vy)
        ey = (az*vx - ax*vz)
        ez = (ax*vy - ay*vx)

        # 对误差进行积分
        exInt = exInt + ex * self.Ki
        eyInt = eyInt + ey * self.Ki
        ezInt = ezInt + ez * self.Ki

        # adjusted gyroscope measurements
        gx = gx + self.Kp*ex + exInt  # 将误差PI后补偿到陀螺仪，即补偿零点漂移
        gy = gy + self.Kp*ey + eyInt
        gz = gz + self.Kp*ez + ezInt  # 这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

        # 下面进行姿态的更新，也就是四元数微分方程的求解
        q0temp = q0
        q1temp = q1
        q2temp = q2
        q3temp = q3

        # 采用一阶毕卡解法
        q0 = q0temp + (-q1temp*gx - q2temp*gy - q3temp*gz)*halfT
        q1 = q1temp + (q0temp*gx + q2temp*gz - q3temp*gy)*halfT
        q2 = q2temp + (q0temp*gy - q1temp*gz + q3temp*gx)*halfT
        q3 = q3temp + (q0temp*gz + q1temp*gy - q2temp*gx)*halfT

        # 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
        norm = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        q0 = q0 / norm
        q1 = q1 / norm
        q2 = q2 / norm
        q3 = q3 / norm

        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

        self.exInt, self.eyInt, self.ezInt = exInt, eyInt, ezInt
        Q_ANGLE_X = math.atan2(2 * q2*q3 + 2 * q0*q1, -
                               2 * q1*q1 - 2 * q2*q2 + 1)  # roll
                               
        Q_ANGLE_Y = math.asin(-2 * q1*q3 + 2 * q0*q2)  # pitch

        Q_ANGLE_Z = math.atan2(2*q1*q2 + 2*q0*q3, -2 *
                               q2*q2 - 2*q3*q3 + 1)  # yaw

        q = [q0, q1, q2, q3]
        return Q_ANGLE_X, Q_ANGLE_Y, Q_ANGLE_Z, q

    # def getAngleAHRS(self, ax, ay, az, gx, gy, gz):
    #     # recipNorm = 0
	#     # s0, s1, s2, s3 = 0,0,0,0
	#     # qDot1, qDot2, qDot3, qDot4 = 0,0,0,0
	#     # _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3
    #     beta = self.beta
    #     sampleFreq = 1/(self.halfT*2)
    #     q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
    #     # Rate of change of quaternion from gyroscope
    #     qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
    #     qDot2 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
    #     qDot3 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
    #     qDot4 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

    #     # Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    #     if ax*ay*az == 0: 
    #         return 0, 0, 0

    #     # Normalise accelerometer measurement
    #     recipNorm = math.sqrt(ax*ax + ay*ay + az*az)
    #     ax = ax/recipNorm
    #     ay = ax/recipNorm
    #     az = ax/recipNorm   

    #     # Auxiliary variables to avoid repeated arithmetic
    #     _2q0 = 2.0 * q0
    #     _2q1 = 2.0 * q1
    #     _2q2 = 2.0 * q2
    #     _2q3 = 2.0 * q3
    #     _4q0 = 4.0 * q0
    #     _4q1 = 4.0 * q1
    #     _4q2 = 4.0 * q2
    #     _8q1 = 8.0 * q1
    #     _8q2 = 8.0 * q2
    #     q0q0 = q0 * q0
    #     q1q1 = q1 * q1
    #     q2q2 = q2 * q2
    #     q3q3 = q3 * q3

    #     # Gradient decent algorithm corrective step
    #     s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
    #     s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
    #     s2 = 4.0 * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
    #     s3 = 4.0 * q1q1 * q3 - _2q1 * ax + 4.0 * q2q2 * q3 - _2q2 * ay
        
    #     recipNorm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3) # normalise step magnitude
    #     s0 = s0/recipNorm
    #     s1 = s1/recipNorm
    #     s2 = s2/recipNorm
    #     s3 = s3/recipNorm

    #     # Apply feedback step
    #     qDot1 -= beta * s0
    #     qDot2 -= beta * s1
    #     qDot3 -= beta * s2
    #     qDot4 -= beta * s3
    
    #     # Integrate rate of change of quaternion to yield quaternion
    #     q0 += qDot1 * (1.0 / sampleFreq)
    #     q1 += qDot2 * (1.0 / sampleFreq)
    #     q2 += qDot3 * (1.0 / sampleFreq)
    #     q3 += qDot4 * (1.0 / sampleFreq)

    #     # Normalise quaternion
    #     recipNorm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    #     q0 = q0/recipNorm
    #     q1 = q1/recipNorm
    #     q2 = q2/recipNorm
    #     q3 = q3/recipNorm

    #     Q_ANGLE_X = math.atan2(2 * q2*q3 + 2 * q0*q1, -
    #                            2 * q1*q1 - 2 * q2*q2 + 1)  # roll
                               
    #     Q_ANGLE_Y = math.asin(-2 * q1*q3 + 2 * q0*q2)  # pitch

    #     Q_ANGLE_Z = math.atan2(2*q1*q2 + 2*q0*q3, -2 *
    #                            q2*q2 - 2*q3*q3 + 1)  # yaw

    #     self.q0 = q0
    #     self.q1 = q1
    #     self.q2 = q2
    #     self.q3 = q3
    #     self.beta = beta

    #     q = [q0, q1, q2, q3]
    #     return Q_ANGLE_X, Q_ANGLE_Y, Q_ANGLE_Z, q

    def transform(self, q, x, y, z):
        q0, q1, q2, q3 = q[0], q[1], q[2], q[3]

        multi = np.matrix([[1-2*q2*q2-2*q3*q3, 2*q1*q2+2*q0*q3, 2*q1*q3-2*q0*q2],\
                           [2*q1*q2-2*q0*q3, 1-2*q1*q1-2*q3*q3, 2*q2*q3+2*q0*q1],\
                           [2*q1*q3+2*q0*q2, 2*q2*q3-2*q0*q1, 1-2*q1*q1-2*q2*q2]\
                           ])
        res = np.dot(multi,[[x],[y],[z]]).getA()
        return res 