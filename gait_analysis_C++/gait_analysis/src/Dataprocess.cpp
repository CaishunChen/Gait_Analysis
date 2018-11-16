#include "Dataprocess.h"

#include <vector>
#include <string>
#include <stdlib.h>
#include <math.h>

#include "config.h"
#include "Signalprocess.h"
#include "filter.h"
#include "Quaternion.h"

using namespace std;

//姿态数据
vector<vector<F32> > quaternion;
vector<F32> pitch;
vector<F32> roll;
vector<F32> yaw;
//绝对坐标系上的加速度
vector<F32> Ax;
// vector<F32> Ay;
// vector<F32> Az;
//积分后的速度与位移数据
vector<F32> Vx;
// vector<F32> Vy;
// vector<F32> Vz;

vector<F32> Sx;
// vector<F32> Sy;
// vector<F32> Sz;

U16 totaltime = 0;
vector<struct Pitch_Period> route;

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

void RegisterInfo(F32 w, F32 h)
{
    human_height = h;
    human_weight = w;
    human_feetlength = human_height / 7.0;
}

void SplitString(const string &s, vector<string> &v, const string &c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while (string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2 - pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if (pos1 != s.length())
        v.push_back(s.substr(pos1));
}

void ProcessNewData(vector<string> raw_data)
{
    vector<F32> ax;
    vector<F32> ay;
    vector<F32> az;
    vector<F32> gx;
    vector<F32> gy;
    vector<F32> gz;
    U16 i = 0;
    //数据预处理
    for (i = 0; i < raw_data.size(); i++)
    {
        vector<string> single_data;
        SplitString(raw_data[i], single_data, " ");
        //因为传感器的数据第一行没有空格，会导致读入时出错
        if (single_data.size() == 7)
        {
            ax.push_back((float)(atoi(single_data[2].c_str())));
            ay.push_back((float)(atoi(single_data[1].c_str())));
            az.push_back((float)(atoi(single_data[3].c_str())));

            gx.push_back((float)(atoi(single_data[5].c_str())));
            gy.push_back((float)(atoi(single_data[4].c_str())));
            gz.push_back((float)(atoi(single_data[6].c_str())));
        }
    }
    // 传感器还原真实值
    ax = Raw2Real(ax, ACC_RATE);
    ay = Raw2Real(ay, ACC_RATE);
    az = Raw2Real(az, -ACC_RATE);
    gx = Raw2Real(gx, GYRO_RATE);
    gy = Raw2Real(gy, GYRO_RATE);
    gz = Raw2Real(gz, -GYRO_RATE);
    //中值滤波
    ax = Mean_Filter(ax, MEAN_WINDOWSIZE);
    ay = Mean_Filter(ay, MEAN_WINDOWSIZE);
    az = Mean_Filter(az, MEAN_WINDOWSIZE);
    gx = Mean_Filter(gx, MEAN_WINDOWSIZE);
    gy = Mean_Filter(gy, MEAN_WINDOWSIZE);
    gz = Mean_Filter(gz, MEAN_WINDOWSIZE);
    //卡尔曼滤波
    ax = Kalman_Filter(ax);
    ay = Kalman_Filter(ay);
    az = Kalman_Filter(az);
    gx = Kalman_Filter(gx);
    gy = Kalman_Filter(gy);
    gz = Kalman_Filter(gz);
    //定义存放四元数的向量quaternion
    quaternion.resize(ax.size());
    for (i = 0; i < quaternion.size(); i++)
        quaternion[i].resize(4);
    //获取pitch、yaw、roll,以及四元数
    for (i = 0; i < ax.size(); i++)
    {
        ANGLE angle;
        angle = IMUupdate(ax[i], ay[i], az[i], gx[i], gy[i], gz[i], FRQ);
        roll.push_back(angle.X);
        pitch.push_back(angle.Y);
        yaw.push_back(angle.Z);
        for (U16 j = 0; j < 4; j++)
            quaternion[i][j] = (angle.quter[j]);
    }
    //将相对坐标系上的加速度转换为绝对坐标系上的加速度
    for (i = 0; i < quaternion.size(); i++)
    {
        COORD cod = transform(quaternion[i], ax[i], ay[i], az[i]);
        Ax.push_back(cod.x);
        // Ay.push_back(cod.y);
        // Az.push_back(cod.z);
    }
    //对加速度速度积分，得到速度和位移
    F32 temp_x;
    // F32 temp_y, temp_z;
    //加速度积分
    Vx.push_back(0);
    // Vy.push_back(0);
    // Vz.push_back(0);
    for (i = 1; i < Ax.size(); i++)
    {
        temp_x = Vx[i - 1] + (Ax[i] + Ax[i - 1]) / 2.0 / FRQ;
        // temp_y = Vy[i - 1] + (Ay[i] + Ay[i - 1]) / 2.0 / FRQ;
        // temp_z = Vz[i - 1] + (Az[i] + Az[i - 1]) / 2.0 / FRQ;
        if (temp_x < 0)
            temp_x = 0;
        Vx.push_back(temp_x);
        // Vy.push_back(temp_y);
        // Vz.push_back(temp_z);
    }
    //速度积分
    Sx.push_back(0);
    // Sy.push_back(0);
    // Sz.push_back(0);
    for (i = 1; i < Vx.size(); i++)
    {
        temp_x = Sx[i - 1] + (Vx[i] + Vx[i - 1]) / 2.0 / FRQ;
        // temp_y = Sy[i - 1] + (Vy[i] + Vy[i - 1]) / 2.0 / FRQ;
        // temp_z = Sz[i - 1] + (Vz[i] + Vz[i - 1]) / 2.0 / FRQ;
        Sx.push_back(temp_x);
        // Sy.push_back(temp_y);
        // Sz.push_back(temp_z);
    }
    //寻找关键点
    vector<U16> peaks = findPeaks(pitch, PITCH_PEAKS_DIS, PITCH_PEAKS_LOWBOUND, PITCH_PEAKS_UPBOUND);
    vector<U16> tiny_peaks = findPeaks(pitch, PITCH_TINYPEAKS_DIS, PITCH_TINYPEAKS_LOWBOUND, PITCH_TINYPEAKS_UPBOUND);
    vector<U16> vallies = findVallies(pitch, PITCH_VALLEIES_DIS, PITCH_VALLEIES_LOWBOUND, PITCH_VALLEIES_UPBOUND);
    vector<vector<U16> > nearzeros = findNearzeros(pitch, NEARZERO_CONTIUNES_MINNUM, NEARZERO_LOWBOUND, NEARZERO_UPBOUND);

    //为时间线标注状态
    vector<S16> timeline(pitch.size(), UNKNOWN);
    totaltime = pitch.size();
    for (i = 0; i < nearzeros.size(); i++)
        for (U16 j = 0; j < nearzeros[i].size(); j++)
            timeline[nearzeros[i][j]] = NEAR_ZERO;
    for (i = 0; i < vallies.size(); i++)
        timeline[vallies[i]] = VALLEY;
    for (i = 0; i < tiny_peaks.size(); i++)
        timeline[tiny_peaks[i]] = TINY_PEAK;
    for (i = 0; i < peaks.size(); i++)
        timeline[peaks[i]] = PEAK;

    //为时间线分割成周期
    U16 count = 0;
    struct Pitch_Period count_buffer;
    count_buffer.clear();
    for (i = 0; i < timeline.size(); i++)
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
                route.push_back(count_buffer);
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
            count_buffer.nearzero.push_back(i);
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
}

void setCountStep()
{
    F32 t = 0.0;
    U16 i = 0;
    for (i = 0; i < route.size(); i++)
        t += route[i].end_valley - route[i].start_valley;
    t = t / totaltime;
    countstep = round(route.size() / t * CONFIDENCE_RATE);
    stepvelocity = round((F32)countstep/totaltime*FRQ*60.0);
}

void setSupportRate()
{
    F32 sum = 0.0;
    U16 i = 0;
    for (i = 0; i < route.size(); i++)
        sum += (F32)(route[i].tiny_peak - route[i].peak) / (route[i].end_valley - route[i].start_valley);
    supportrate = sum / route.size() * 100.0;
}

void setRollAngle()
{
    F32 sum = 0.0;
    U16 n = 0;
    U16 i = 0;
    for (i = 0; i < route.size(); i++)
    {
        sum += roll[route[i].nearzero[0]];
        n++;
    }
    rollangle = sum / n / PI * 180;
}

void setStepLen()
{
    F32 sum = 0.0;
    U16 n = 0;
    U16 i = 0;
    for (i = 1; i < route.size(); i++)
        if (route[i - 1].end_valley == route[i].start_valley)
        {
            sum += Sx[route[i].nearzero[0]] - Sx[route[i - 1].nearzero[0]];
            n++;
        }
    totaldistance_Integral = sum;
    steplength = U16(100*sum / n);
}

void setStepVelocity_Avr()
{
    F32 sum = 0.0;
    U16 n = 0;
    U16 i = 0;
    for (i = 1; i < route.size(); i++)
        if (route[i - 1].end_valley == route[i].start_valley)
        {
            sum += (Sx[route[i].nearzero[0]] - Sx[route[i - 1].nearzero[0]]) / (route[i].nearzero[0] - route[i - 1].nearzero[0]) / T;
            n++;
        }
    stepvelocity_avr = sum / n;
}

void setWaveVelocity()
{
    F32 max = 0.0;
    F32 sum = 0.0;
    U16 n = 0,i = 0,j = 0;

    for (i = 1; i < route.size(); i++)
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
// F32 GaitanalyseGetStancePercent()
// {
//     return supportrate;
// }


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


U16 GaitanalyseGetStepFrequence()
{
    return stepvelocity;
}
U16 GaitanalyseGetStrideLength()
{
    return steplength;
}
U16 GaitanalyseGetStancePercent()
{
    return supportrate;
}
F32 GaitanalyseSwingSpeedMean()
{
    return wavevelocity_max;
}
