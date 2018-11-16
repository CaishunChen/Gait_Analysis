#ifndef __DATAPROCESS_H__
#define __DATAPROCESS_H__

#include "config.h"


#define UNKNOWN 0
#define NEAR_ZERO 1
#define VALLEY 2
#define TINY_PEAK 3
#define PEAK 4

extern void ProcessNewData(F32* ax_array,F32* ay_array,F32* az_array,F32* gx_array,F32* gy_array,F32* gz_array);

extern void setCountStep();
extern void setSupportRate();
extern void setRollAngle();
extern void setStepLen();
extern void setWaveVelocity();
extern void setStepVelocity_Avr();
extern void setTotalDistance();

struct Pitch_Period
{
    S16 start_valley;
    S16 zero_rise;
    S16 peak;
    S16 tiny_peak;
    S16 zero_fall;
    S16 end_valley;
    S16 nearzero_start;
    S16 nearzero_end;
};

extern U8 human_height;
extern U8 human_weight;
extern F32 human_feetlength;

extern int countstep;
extern int countstep_now ;

extern F32 stepvelocity;
extern F32 supportrate;
extern F32 rollangle;
extern F32 steplength;
extern F32 wavevelocity_max;
extern F32 stepvelocity_avr;

extern F32 totaldistance_Mix;
extern F32 totaldistance_Integral;
extern F32 totaldistance_Fomula;

#endif