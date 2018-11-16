#include "Quaternion.h"

#include <math.h>
#include <vector>
#include "config.h"
using namespace std;

F32 q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // 初始姿态四元数，由上篇博文提到的变换四元数公式得来
F32 exInt = 0, eyInt = 0, ezInt = 0; //当前加计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差的积分
F32 halfT;                   // 采样周期的一半，用于求解四元数微分方程时计算角增量
struct ANGLE IMUupdate(F32 ax, F32 ay, F32 az, F32 gx, F32 gy, F32 gz,U16 frq) //g表陀螺仪，a表加计
{
    F32 q0temp, q1temp, q2temp, q3temp; //四元数暂存变量，求解微分方程时要用
    F32 norm;                           //矢量的模或四元数的范数
    F32 vx, vy, vz;                     //当前姿态计算得来的重力在三轴上的分量
    F32 ex, ey, ez;                     //当前加计测得的重力加速度在三轴上的分量
    //与用当前姿态计算得来的重力在三轴上的分量的误差
    struct ANGLE Q_ANGLE = {0, 0, 0};
        
    F32 q0q0 = q0 * q0;
    F32 q0q1 = q0 * q1;
    F32 q0q2 = q0 * q2;
    F32 q1q1 = q1 * q1;
    F32 q1q3 = q1 * q3;
    F32 q2q2 = q2 * q2;
    F32 q2q3 = q2 * q3;
    F32 q3q3 = q3 * q3;

    halfT = 1.0/frq/2.0;
    
    if (ax * ay * az == 0) //加计处于自由落体状态时不进行姿态解算，因为会产生分母无穷大的情况
        return Q_ANGLE;
    // cout <<ax<<"  "<<ay<<"   "<<az<< endl;
    norm = sqrt(ax * ax + ay * ay + az * az); //单位化加速度计，
    ax = ax / norm;                           // 这样变更了量程也不需要修改KP参数，因为这里归一化了
    ay = ay / norm;
    az = az / norm;

    //用当前姿态计算出重力在三个轴上的分量，
    //参考坐标n系转化到载体坐标b系的用四元数表示的方向余弦矩阵第三列即是（博文一中有提到）
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // cout <<vx<<"  "<<vy<<"   "<<vz<< endl;
    //计算测得的重力与计算得重力间的误差，向量外积可以表示这一误差
    //原因我理解是因为两个向量是单位向量且sin0等于0
    //不过要是夹角是180度呢~这个还没理解
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    exInt = exInt + ex * Ki; //对误差进行积分
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
    // adjusted gyroscope measurements
    gx = gx + Kp * ex + exInt; //将误差PI后补偿到陀螺仪，即补偿零点漂移
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt; //这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减
    //下面进行姿态的更新，也就是四元数微分方程的求解
    q0temp = q0; //暂存当前值用于计算
    q1temp = q1; //网上传的这份算法大多没有注意这个问题，在此更正
    q2temp = q2;
    q3temp = q3;
    //采用一阶毕卡解法，相关知识可参见《惯性器件与惯性导航系统》P212
    q0 = q0temp + (-q1temp * gx - q2temp * gy - q3temp * gz) * halfT;
    q1 = q1temp + (q0temp * gx + q2temp * gz - q3temp * gy) * halfT;
    q2 = q2temp + (q0temp * gy - q1temp * gz + q3temp * gx) * halfT;
    q3 = q3temp + (q0temp * gz + q1temp * gy - q2temp * gx) * halfT;
    //单位化四元数在空间旋转时不会拉伸，仅有旋转角度，这类似线性代数里的正交变换
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
    //四元数到欧拉角的转换，公式推导见博文一
    //其中YAW航向角由于加速度计对其没有修正作用，因此此处直接用陀螺仪积分代替
    Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1); //roll
    Q_ANGLE.Y = asin(-2 * q1 * q3 + 2 * q0 * q2); //pitch
    Q_ANGLE.Z = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 *q2 * q2 -2 * q3 * q3 + 1); //yaw

    Q_ANGLE.quter.push_back(q0);
    Q_ANGLE.quter.push_back(q1);
    Q_ANGLE.quter.push_back(q2);
    Q_ANGLE.quter.push_back(q3);

    return Q_ANGLE;
}

struct COORD transform(vector<F32>q4,F32 x,F32 y,F32 z){
    F32 q0 = q4[0],q1 = q4[1],q2 = q4[2],q3 = q4[3];

    F32 A00 = 1-2*q2*q2-2*q3*q3;
    F32 A01 = 2*q1*q2+2*q0*q3;
    F32 A02 = 2*q1*q3-2*q0*q2;
    F32 A10 = 2*q1*q2-2*q0*q3;
    F32 A11 = 1-2*q1*q1-2*q3*q3;
    F32 A12 = 2*q2*q3+2*q0*q1;
    F32 A20 = 2*q1*q3+2*q0*q2;
    F32 A21 = 2*q2*q3-2*q0*q1;
    F32 A22 = 1-2*q1*q1-2*q2*q2;

    struct COORD cod;
    cod.x = A00*x+A01*y+A02*z;
    cod.y = A10*x+A11*y+A12*z;
    cod.z = A20*x+A21*y+A22*z;

    return cod;
}