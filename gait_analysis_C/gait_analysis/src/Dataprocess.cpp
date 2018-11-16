#include "Dataprocess.h"

#include <vector>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include "config.h"
#include "Signalprocess.h"
#include "filter.h"
#include "Quaternion.h"
#include "Implement.h"

using namespace std;

//姿态数据

float *pitch;
float *roll;
float *yaw;
//绝对坐标系上的加速度
float* Ax;

//积分后的速度与位移数据
float* Vx;
float* Sx;


U16 totaltime = 0;

struct Pitch_Period *route;

static U16 route_size = 0;

static F32 human_height;
static F32 human_weight;
static F32 human_feetlength;

static U16 countstep = 0;
static U16 stepvelocity = 0;
static U8 supportrate = 0;
static F32 rollangle = 0;
static U8 steplength = 0;
static F32 wavevelocity_max = 0;
static F32 stepvelocity_avr = 0;

static F32 totaldistance_Mix = 0;
static F32 totaldistance_Integral = 0;
static F32 totaldistance_Fomula = 0;

static U16 data_size = 0;
void RegisterInfo(F32 w, F32 h)
{
    human_height = h;
    human_weight = w;
    human_feetlength = human_height / 7.0;
}

void ProcessNewData(float* ax_array,float* ay_array,float* az_array,float* gx_array,float* gy_array,float* gz_array)
{

    data_size = MAX_ARRAY_SIZE;

    // float *ax_array = (F32 *)malloc(data_size * sizeof(F32));
    // float *ay_array = (F32 *)malloc(data_size * sizeof(F32));
    // float *az_array = (F32 *)malloc(data_size * sizeof(F32));

    // float *gx_array = (F32 *)malloc(data_size * sizeof(F32));
    // float *gy_array = (F32 *)malloc(data_size * sizeof(F32));
    // float *gz_array = (F32 *)malloc(data_size * sizeof(F32));



    pitch = (F32 *)malloc(data_size * sizeof(F32));
    roll = (F32 *)malloc(data_size * sizeof(F32));
    yaw = (F32 *)malloc(data_size * sizeof(F32));

    Ax = (F32 *)malloc(data_size * sizeof(F32));
    Vx = (F32 *)malloc(data_size * sizeof(F32));
    Sx = (F32 *)malloc(data_size * sizeof(F32));

    //定义存放四元数的向量quaternion
    F32 quaternion[data_size][4];

    route = (struct Pitch_Period *)malloc(MAX_ARRAY_SIZE * sizeof(struct Pitch_Period));
    U16 i = 0;

    // 传感器还原真实值

    Raw2Real_A(ax_array, data_size, ACC_RATE);
    Raw2Real_A(ay_array, data_size, ACC_RATE);
    Raw2Real_A(az_array, data_size, -ACC_RATE);

    Raw2Real_A(gx_array, data_size, GYRO_RATE);
    Raw2Real_A(gy_array, data_size, GYRO_RATE);
    Raw2Real_A(gz_array, data_size, -GYRO_RATE);

    //中值滤波
    //    for(i=0;i<20;i++)
    // {
    //     cout<<i<<" "<<ax_array[i]<<endl;
    // }

    Mean_Filter_A(ax_array, data_size, MEAN_WINDOWSIZE);
    Mean_Filter_A(ay_array, data_size, MEAN_WINDOWSIZE);
    Mean_Filter_A(az_array, data_size, MEAN_WINDOWSIZE);

    Mean_Filter_A(gx_array, data_size, MEAN_WINDOWSIZE);
    Mean_Filter_A(gy_array, data_size, MEAN_WINDOWSIZE);
    Mean_Filter_A(gz_array, data_size, MEAN_WINDOWSIZE);

    //卡尔曼滤波
    Kalman_Filter_A(ax_array, data_size);
    Kalman_Filter_A(ay_array, data_size);
    Kalman_Filter_A(az_array, data_size);
    Kalman_Filter_A(gx_array, data_size);
    Kalman_Filter_A(gy_array, data_size);
    Kalman_Filter_A(gz_array, data_size);

    //获取pitch、yaw、roll,以及四元数
    for (i = 0; i < data_size; i++)
    {
        ANGLE angle;
        angle = IMUupdate(ax_array[i], ay_array[i], az_array[i],
                 gx_array[i], gy_array[i], gz_array[i], FRQ);
        roll[i] = (angle.X);
        pitch[i] = (angle.Y);
        yaw[i] = (angle.Z);

        quaternion[i][0] = (angle.q0);
        quaternion[i][1] = (angle.q1);
        quaternion[i][2] = (angle.q2);
        quaternion[i][3] = (angle.q3);
    }
    //将相对坐标系上的加速度转换为绝对坐标系上的加速度
    for (i = 0; i < data_size; i++)
    {
        COORD cod = transform(quaternion[i][0], quaternion[i][1], quaternion[i][2], quaternion[i][3],
          ax_array[i], ay_array[i], az_array[i]);
        Ax[i] = cod.x;
        // Ay.push_back(cod.y);
        // Az.push_back(cod.z);
    }
    //对加速度速度积分，得到速度和位移
    F32 temp_x;
    // F32 temp_y, temp_z;
    //加速度积分
    Vx[0] = 0;
    // Vy.push_back(0);
    // Vz.push_back(0);
    for (i = 1; i < data_size; i++)
    {
        temp_x = Vx[i - 1] + (Ax[i] + Ax[i - 1]) / 2.0 / FRQ;
        // temp_y = Vy[i - 1] + (Ay[i] + Ay[i - 1]) / 2.0 / FRQ;
        // temp_z = Vz[i - 1] + (Az[i] + Az[i - 1]) / 2.0 / FRQ;
        if (temp_x < 0)
            temp_x = 0;
        Vx[i] = (temp_x);
        // Vy.push_back(temp_y);
        // Vz.push_back(temp_z);
    }
    //速度积分
    // Sx.push_back(0);
    Sx[0] = 0;
    // Sy.push_back(0);
    // Sz.push_back(0);
    for (i = 1; i < data_size; i++)
    {
        temp_x = Sx[i - 1] + (Vx[i] + Vx[i - 1]) / 2.0 / FRQ;
        // temp_y = Sy[i - 1] + (Vy[i] + Vy[i - 1]) / 2.0 / FRQ;
        // temp_z = Sz[i - 1] + (Vz[i] + Vz[i - 1]) / 2.0 / FRQ;
        Sx[i] = temp_x;
        // Sy.push_back(temp_y);
        // Sz.push_back(temp_z);
    }
    //寻找关键点

 

    int* peaks = (int *)malloc(MAX_ARRAY_SIZE * sizeof(int));
    int peaks_size = 0;

    int* tiny_peaks = (int *)malloc(MAX_ARRAY_SIZE * sizeof(int));
    int tiny_peaks_size = 0;

    int* vallies = (int *)malloc(MAX_ARRAY_SIZE * sizeof(int));
    int vallies_size = 0;

    int* nearzeros = (int *)malloc(MAX_ARRAY_SIZE * sizeof(int));
    int nearzeros_size = 0;

    findPeaks(pitch,data_size,peaks,&peaks_size,
                PITCH_PEAKS_DIS, PITCH_PEAKS_LOWBOUND, PITCH_PEAKS_UPBOUND);
    findPeaks(pitch,data_size,tiny_peaks,&tiny_peaks_size, PITCH_TINYPEAKS_DIS,
                 PITCH_TINYPEAKS_LOWBOUND, PITCH_TINYPEAKS_UPBOUND);
    findVallies(pitch,data_size,vallies,&vallies_size,PITCH_VALLEIES_DIS, 
                    PITCH_VALLEIES_LOWBOUND, PITCH_VALLEIES_UPBOUND);
    findNearzeros(pitch,data_size,nearzeros,&nearzeros_size,
            NEARZERO_CONTIUNES_MINNUM, NEARZERO_LOWBOUND, NEARZERO_UPBOUND);

    cout<<peaks_size<<" "<<tiny_peaks_size<<" "<<nearzeros_size<<endl;

    //为时间线标注状态
    int* timeline = (int *)malloc(data_size * sizeof(int));

    totaltime = data_size;
    for (i = 0; i < nearzeros_size; i++)
            timeline[nearzeros[i]] = NEAR_ZERO;    

    for (i = 0; i < vallies_size; i++)
        timeline[vallies[i]] = VALLEY;

    for (i = 0; i < tiny_peaks_size; i++)
        timeline[tiny_peaks[i]] = TINY_PEAK;

    for (i = 0; i < peaks_size; i++)
        timeline[peaks[i]] = PEAK;

    //为时间线分割成周期
    U16 count = 0;
    struct Pitch_Period count_buffer;
    count_buffer.clear();
    for (i = 0; i < data_size; i++)
    {
        if (timeline[i] == VALLEY)
        {
            if (count == 0)
            {
                count = 1;
                count_buffer.start_valley = i;
            }
            else if (count == 3)
            {
                count = 1;
                count_buffer.end_valley = i;
                route[route_size] = (count_buffer);
                route_size++;
                count_buffer.clear();
                count_buffer.start_valley = i;
            }
            else
            {
                count = 0;
                count_buffer.clear();
            }
        }
        else if (timeline[i] == PEAK)
        {
            if (count == 1)
            {
                count = 2;
                count_buffer.peak = i;
            }
            else
            {
                count = 0;
                count_buffer.clear();
            }
        }
        else if (timeline[i] == NEAR_ZERO && count == 2)
        {
            // count_buffer.nearzero.push_back(i);
            if(count_buffer.nearzero_start==0&&count_buffer.nearzero_end==0)
                count_buffer.nearzero_start = i;
            else if(count_buffer.nearzero_start!=0)
                count_buffer.nearzero_end = i;
                
        }
        else if (timeline[i] == TINY_PEAK)
        {
            if (count == 2)
            {
                count = 3;
                count_buffer.tiny_peak = i;
            }
            else
            {
                count = 0;
                count_buffer.clear();
            }
        }
    }
    //计算步数等等数据
    setCountStep();
    setSupportRate();
    setRollAngle();
    setStepLen();
    setStepVelocity_Avr();
    setWaveVelocity();
    setTotalDistance();
    cout<<"step num is "<<countstep<<endl;
}

void setCountStep()
{
    F32 t = 0.0;
    U16 i = 0;
    for (i = 0; i <route_size; i++)
        t += route[i].end_valley - route[i].start_valley;
    t = t / totaltime;
    countstep = round(route_size / t * CONFIDENCE_RATE);
    stepvelocity = round((F32)countstep / totaltime * FRQ * 60.0);
}

void setSupportRate()
{
    F32 sum = 0.0;
    U16 i = 0;
    for (i = 0; i < route_size; i++)
        sum += (F32)(route[i].tiny_peak - route[i].peak) / (route[i].end_valley - route[i].start_valley);
    supportrate = sum / route_size * 100.0;
}

void setRollAngle()
{
    F32 sum = 0.0;
    U16 n = 0;
    U16 i = 0;
    for (i = 0; i < route_size; i++)
    {
        sum += roll[route[i].nearzero_start];
        n++;
    }
    rollangle = sum / n / PI * 180;
}

void setStepLen()
{
    F32 sum = 0.0;
    U16 n = 0;
    U16 i = 0;
    for (i = 1; i < route_size; i++)
        if (route[i - 1].end_valley == route[i].start_valley)
        {
            sum += Sx[route[i].nearzero_start] - Sx[route[i - 1].nearzero_start];
            n++;
        }
    totaldistance_Integral = sum;
    steplength = U16(100 * sum / n);
}

void setStepVelocity_Avr()
{
    F32 sum = 0.0;
    U16 n = 0;
    U16 i = 0;
    for (i = 1; i < route_size; i++)
        if (route[i - 1].end_valley == route[i].start_valley)
        {
            sum += (Sx[route[i].nearzero_start] - Sx[route[i - 1].nearzero_start]) / (route[i].nearzero_start - route[i - 1].nearzero_start) / T;
            n++;
        }
    stepvelocity_avr = sum / n;
}

void setWaveVelocity()
{
    F32 max = 0.0;
    F32 sum = 0.0;
    U16 n = 0, i = 0, j = 0;

    for (i = 1; i < route_size; i++)
        if (route[i - 1].end_valley == route[i].start_valley)
        {
            for (j = route[i - 1].tiny_peak; j < route[i].peak; j++)
                if (Vx[j] > max)
                    max = Vx[j];
            sum += max;
            n++;
            max = 0.0;
        }
    wavevelocity_max = sum / n;
}
void setTotalDistance()
{
    totaldistance_Mix = 1.0 * steplength * getStepNum();
    F32 step_length_formular;
    if (human_height >= 166) //高个
        step_length_formular = human_height - 1 / 3 * human_feetlength;
    else if (human_height < 166 && human_height >= 148) //中等
        step_length_formular = human_height - 1 / 2 * human_feetlength;
    else //小个
        step_length_formular = human_height - 2 / 3 * human_feetlength;
    totaldistance_Fomula = 1.0 * step_length_formular * getStepNum();
}

U16 getStepNum()
{
    return countstep;
}

F32 getSpeedMean()
{
    return stepvelocity_avr;
}
F32 getStrickRollMean()
{
    return rollangle;
}

F32 getTotalDistance_Mix()
{
    return totaldistance_Mix;
}
F32 getTotalDistance_Formula()
{
    return totaldistance_Fomula;
}
F32 getTotalDistance_Integral()
{
    return totaldistance_Integral;
}

