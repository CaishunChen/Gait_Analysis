#ifndef __DATAPROCESS_H__
#define __DATAPROCESS_H__

#include <string>
#include <vector>
#include "config.h"
using namespace std;

#define UNKNOWN 0
#define NEAR_ZERO 1
#define VALLEY 2
#define TINY_PEAK 3
#define PEAK 4

// extern void ProcessNewData(vector<string> raw_data);
extern void ProcessNewData(float* ax_array,float* ay_array,float* az_array,float* gx_array,float* gy_array,float* gz_array);
extern void RegisterInfo(F32 w,F32 h);

extern void setCountStep();
extern void setSupportRate();
extern void setRollAngle();
extern void setStepLen();
extern void setWaveVelocity();
extern void setStepVelocity_Avr();
extern void setTotalDistance();

extern U16 getStepNum();

extern F32 getSpeedMean();
extern F32 getStrickRollMean();
extern F32 getTotalDistance_Mix();
extern F32 getTotalDistance_Formula();
extern F32 getTotalDistance_Integral();

struct Pitch_Period
{
    U16 start_valley;
    U16 zero_rise;
    U16 peak;
    U16 tiny_peak;
    U16 zero_fall;
    U16 end_valley;
    U16 nearzero_start;
    U16 nearzero_end;
    void clear()
    {
        this->start_valley = UNKNOWN;
        this->zero_rise = UNKNOWN;
        this->peak = UNKNOWN;
        this->tiny_peak = UNKNOWN;
        this->zero_fall = UNKNOWN;
        this->end_valley = UNKNOWN;
        this->nearzero_start = UNKNOWN;
        this->nearzero_end = UNKNOWN;
    };
};

#endif