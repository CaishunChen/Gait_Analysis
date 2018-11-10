#ifndef __DATAPROCESS_H__
#define __DATAPROCESS_H__

#include <string>
#include <vector>
using namespace std;

#define UNKNOWN 0
#define NEAR_ZERO 1
#define VALLEY 2
#define TINY_PEAK 3
#define PEAK 4

extern void DataInput(vector<string> raw_data);
extern void CountStep();
extern void SupportRate();
extern void RollAngle();
extern void StepLen();
extern void WaveVelocity();
extern void StepVelocity_Avr();

extern void getResult();
struct Pitch_Period
{
    int start_valley;
    int zero_rise;
    int peak;
    int tiny_peak;
    int zero_fall;
    int end_valley;
    vector<int> nearzero;
    void clear()
    {
        this->start_valley = UNKNOWN;
        this->zero_rise = UNKNOWN;
        this->peak = UNKNOWN;
        this->tiny_peak = UNKNOWN;
        this->zero_fall = UNKNOWN;
        this->end_valley = UNKNOWN;
        this->nearzero.clear();
    };
};
#endif