#include "Dataprocess.h"

#include <vector>
#include <string>
#include <iostream>

#include <math.h>
// #include <math>
// #include <fstream>

#include "config.h"
#include "Signalprocess.h"
#include "filter.h"
#include "Quaternion.h"

using namespace std;
//传感器原始数据
vector<double> ax;
vector<double> ay;
vector<double> az;
vector<double> gx;
vector<double> gy;
vector<double> gz;
//均值滤波后的数据
vector<double> ax_Mean;
vector<double> ay_Mean;
vector<double> az_Mean;
vector<double> gx_Mean;
vector<double> gy_Mean;
vector<double> gz_Mean;
//卡尔曼滤波后的数据
vector<double> ax_KF;
vector<double> ay_KF;
vector<double> az_KF;
vector<double> gx_KF;
vector<double> gy_KF;
vector<double> gz_KF;
//姿态数据
vector<vector<double>> quaternion;
vector<double> pitch;
vector<double> roll;
vector<double> yaw;
//绝对坐标系上的加速度
vector<double> Ax;
vector<double> Ay;
vector<double> Az;
//积分后的速度与位移数据
vector<double> Vx;
vector<double> Vy;
vector<double> Vz;

vector<double> Sx;
vector<double> Sy;
vector<double> Sz;

short frq = FRQ;
short totaltime = 0;
vector<struct Pitch_Period> route;

int countstep = 0;
double supportrate = 0;
double rollangle = 0;
double steplength = 0;
double wavevelocity_max = 0;
double stepvelocity_avr = 0;

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

void DataInput(vector<string> raw_data)
{
    vector<int> ax_raw;
    vector<int> ay_raw;
    vector<int> az_raw;
    vector<int> gx_raw;
    vector<int> gy_raw;
    vector<int> gz_raw;

    //数据预处理
    for (unsigned i = 0; i < raw_data.size(); i++)
    {
        vector<string> single_data;
        SplitString(raw_data[i], single_data, " ");
        //因为传感器的数据第一行没有空格，会导致读入时出错
        if (single_data.size() == 7)
        {
            ax_raw.push_back(stoi(single_data[2], nullptr, 10));
            ay_raw.push_back(stoi(single_data[1], nullptr, 10));
            az_raw.push_back(stoi(single_data[3], nullptr, 10));

            gx_raw.push_back(stoi(single_data[5], nullptr, 10));
            gy_raw.push_back(stoi(single_data[4], nullptr, 10));
            gz_raw.push_back(stoi(single_data[6], nullptr, 10));
        }
    }
    // 传感器还原真实值
    ax = Raw2Real(ax_raw, ACC_RATE);
    ay = Raw2Real(ay_raw, ACC_RATE);
    az = Raw2Real(az_raw, -ACC_RATE);
    gx = Raw2Real(gx_raw, GYRO_RATE);
    gy = Raw2Real(gy_raw, GYRO_RATE);
    gz = Raw2Real(gz_raw, -GYRO_RATE);
    //中值滤波
    ax_Mean = Mean_Filter(ax, MEAN_WINDOWSIZE);
    ay_Mean = Mean_Filter(ay, MEAN_WINDOWSIZE);
    az_Mean = Mean_Filter(az, MEAN_WINDOWSIZE);
    gx_Mean = Mean_Filter(gx, MEAN_WINDOWSIZE);
    gy_Mean = Mean_Filter(gy, MEAN_WINDOWSIZE);
    gz_Mean = Mean_Filter(gz, MEAN_WINDOWSIZE);
    //卡尔曼滤波
    ax_KF = Kalman_Filter(ax_Mean);
    ay_KF = Kalman_Filter(ay_Mean);
    az_KF = Kalman_Filter(az_Mean);
    gx_KF = Kalman_Filter(gx_Mean);
    gy_KF = Kalman_Filter(gy_Mean);
    gz_KF = Kalman_Filter(gz_Mean);
    //定义存放四元数的向量quaternion
    quaternion.resize(ax_KF.size());
    for (int i = 0; i < quaternion.size(); i++)
        quaternion[i].resize(4);
    //获取pitch、yaw、roll,以及四元数
    for (int i = 0; i < ax_KF.size(); i++)
    {
        ANGLE angle;
        angle = IMUupdate(ax_KF[i], ay_KF[i], az_KF[i], gx_KF[i], gy_KF[i], gz_KF[i], FRQ);
        roll.push_back(angle.X);
        pitch.push_back(angle.Y);
        yaw.push_back(angle.Z);
        for (int j = 0; j < 4; j++)
            quaternion[i][j] = (angle.quter[j]);
    }
    //将相对坐标系上的加速度转换为绝对坐标系上的加速度
    for (int i = 0; i < quaternion.size(); i++)
    {
        COORD cod = transform(quaternion[i], ax_KF[i], ay_KF[i], az_KF[i]);
        Ax.push_back(cod.x);
        Ay.push_back(cod.y);
        Az.push_back(cod.z);
    }
    //对加速度速度积分，得到速度和位移
    float temp_x, temp_y, temp_z;
    //加速度积分
    Vx.push_back(0);
    Vy.push_back(0);
    Vz.push_back(0);
    for (int i = 1; i < Ax.size(); i++)
    {
        temp_x = Vx[i - 1] + (Ax[i] + Ax[i - 1]) / 2.0 / frq;
        temp_y = Vy[i - 1] + (Ay[i] + Ay[i - 1]) / 2.0 / frq;
        temp_z = Vz[i - 1] + (Az[i] + Az[i - 1]) / 2.0 / frq;
        if (temp_x < 0)
            temp_x = 0;
        Vx.push_back(temp_x);
        Vy.push_back(temp_y);
        Vz.push_back(temp_z);
    }
    //速度积分
    Sx.push_back(0);
    Sy.push_back(0);
    Sz.push_back(0);
    for (int i = 1; i < Vx.size(); i++)
    {
        temp_x = Sx[i - 1] + (Vx[i] + Vx[i - 1]) / 2.0 / frq;
        temp_y = Sy[i - 1] + (Vy[i] + Vy[i - 1]) / 2.0 / frq;
        temp_z = Sz[i - 1] + (Vz[i] + Vz[i - 1]) / 2.0 / frq;
        Sx.push_back(temp_x);
        Sy.push_back(temp_y);
        Sz.push_back(temp_z);
    }
    //寻找关键点
    vector<int> peaks = findPeaks(pitch, PITCH_PEAKS_DIS, PITCH_PEAKS_LOWBOUND, PITCH_PEAKS_UPBOUND);
    vector<int> tiny_peaks = findPeaks(pitch, PITCH_TINYPEAKS_DIS, PITCH_TINYPEAKS_LOWBOUND, PITCH_TINYPEAKS_UPBOUND);
    vector<int> vallies = findVallies(pitch, PITCH_VALLEIES_DIS, PITCH_VALLEIES_LOWBOUND, PITCH_VALLEIES_UPBOUND);
    vector<vector<int>> nearzeros = findNearzeros(pitch, NEARZERO_CONTIUNES_MINNUM, NEARZERO_LOWBOUND, NEARZERO_UPBOUND);

    //为时间线标注状态
    vector<int> timeline(pitch.size(), UNKNOWN);
    totaltime = pitch.size();
    for (int i = 0; i < nearzeros.size(); i++)
        for (int j = 0; j < nearzeros[i].size(); j++)
            timeline[nearzeros[i][j]] = NEAR_ZERO;
    for (int i = 0; i < vallies.size(); i++)
        timeline[vallies[i]] = VALLEY;
    for (int i = 0; i < tiny_peaks.size(); i++)
        timeline[tiny_peaks[i]] = TINY_PEAK;
    for (int i = 0; i < peaks.size(); i++)
        timeline[peaks[i]] = PEAK;

    //为时间线分割成周期
    int count = 0;
    struct Pitch_Period count_buffer;
    count_buffer.clear();
    for (int i = 0; i < timeline.size(); i++)
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
    CountStep();
    SupportRate();
    RollAngle();
    StepLen();
    StepVelocity_Avr();
    WaveVelocity();
}

void CountStep()
{
    double t = 0.0;
    for (int i = 0; i < route.size(); i++)
        t += route[i].end_valley - route[i].start_valley;
    t = t / totaltime;
    countstep = round(route.size() / t * CONFIDENCE_RATE);
}

void SupportRate()
{
    double sum = 0.0;
    for (int i = 0; i < route.size(); i++)
        sum += (double)(route[i].tiny_peak - route[i].peak) / (route[i].end_valley - route[i].start_valley);
    supportrate = sum / route.size() * 100;
}

void RollAngle()
{
    double sum = 0.0;
    int n = 0;
    for (int i = 0; i < route.size(); i++)
    {
        sum += roll[route[i].nearzero[0]];
        n++;
    }
    rollangle = sum / n / PI * 180;
}

void StepLen()
{
    double sum = 0.0;
    int n = 0;
    for (int i = 1; i < route.size(); i++)
        if (route[i - 1].end_valley == route[i].start_valley)
        {
            sum += Sx[route[i].nearzero[0]] - Sx[route[i - 1].nearzero[0]];
            n++;
        }
    steplength = sum / n;
}

void StepVelocity_Avr(){
    double sum = 0.0;
    int n = 0;
    for (int i = 1; i < route.size(); i++)
        if (route[i - 1].end_valley == route[i].start_valley)
        {
            sum += (Sx[route[i].nearzero[0]] - Sx[route[i - 1].nearzero[0]])/(route[i].nearzero[0]-route[i - 1].nearzero[0])/T;
            n++;
        }
    stepvelocity_avr = sum / n;
}

void WaveVelocity(){
    double max = 0.0;
    double sum = 0.0;
    int n = 0;
    for (int i = 1; i < route.size(); i++)
        if (route[i - 1].end_valley == route[i].start_valley)
        {
            // sum += (Sx[route[i].nearzero[0]] - Sx[route[i - 1].nearzero[0]])/(route[i].nearzero[0]-route[i - 1].nearzero[0])/T;
            // n++;
            for(int j = route[i-1].tiny_peak;j<route[i].peak;j++)
                if(Vx[j]>max)
                    max = Vx[j];
            sum += max;
            n++;
            max = 0.0;
        }
    wavevelocity_max = sum/n;
}
void getResult()
{
    cout << "步数为 " << countstep << endl;
    cout << "支撑相占比为 " << supportrate << " %" << endl;
    cout << "沿x轴脚落地角度(内外翻) " << rollangle << "°" << endl;
    cout << "步距为 " << steplength << " m" << endl;
    cout << "平均步速为 " << stepvelocity_avr << " m/s" << endl;
    cout << "平均最大摆速为 " << wavevelocity_max << " m/s" << endl;
}