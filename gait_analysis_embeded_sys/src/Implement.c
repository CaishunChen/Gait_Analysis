#include "Implement.h"

#include <config.h>
#include <stdio.h>
#include <time.h>
#include "Dataprocess.h"

static F32 ax_tmp[MAX_ARRAY_SIZE];
static F32 ay_tmp[MAX_ARRAY_SIZE];
static F32 az_tmp[MAX_ARRAY_SIZE];

static F32 gx_tmp[MAX_ARRAY_SIZE];
static F32 gy_tmp[MAX_ARRAY_SIZE];
static F32 gz_tmp[MAX_ARRAY_SIZE];

static int full = 0;

//表示活动状态的标志
static U8 Pattern = 0; //0表示静止，1表示走路，2表示跑步
static U32 ENERGS = 0;
static U8 RIGHT_LEFT;

void ArithProcessNewAccData(INT16S *piNewAcc, const INT8U uNum)
{
    INT16U i;
    if (uNum == 0)
        return;

    full += uNum;
    for (i = 0; i < MAX_ARRAY_SIZE - uNum; i++)
    {
        ax_tmp[i] = ax_tmp[i + uNum];
        ay_tmp[i] = ay_tmp[i + uNum];
        az_tmp[i] = az_tmp[i + uNum];

        gx_tmp[i] = gx_tmp[i + uNum];
        gy_tmp[i] = gy_tmp[i + uNum];
        gz_tmp[i] = gz_tmp[i + uNum];
    }
    S16 count = -1;
    for (i = MAX_ARRAY_SIZE - uNum; i < MAX_ARRAY_SIZE; i++)
    {

        ay_tmp[i] = 1.0 * (piNewAcc[++count]);
        ax_tmp[i] = 1.0 * (piNewAcc[++count]);
        az_tmp[i] = 1.0 * (piNewAcc[++count]);

        gy_tmp[i] = 1.0 * (piNewAcc[++count]);
        gx_tmp[i] = 1.0 * (piNewAcc[++count]);
        gz_tmp[i] = 1.0 * (piNewAcc[++count]);
    }
    if (full % MAX_ARRAY_SIZE == 0)
    {
        ProcessNewData(ax_tmp, ay_tmp, az_tmp, gx_tmp, gy_tmp, gz_tmp);
        GaitanalyseGetVerticalLength();
        printf("\n");
    }
    if (full % FRQ == 0)
        ArithmeticGetPattern();
}

void GaitanalyseClearData()
{
    INT16U i;
    for (i = 0; i < MAX_ARRAY_SIZE; i++)
    {
        ax_tmp[i] = 0;
        ay_tmp[i] = 0;
        az_tmp[i] = 0;

        gx_tmp[i] = 0;
        gy_tmp[i] = 0;
        gz_tmp[i] = 0;
    }
    countstep_now = 0;
    countstep = 0;
    stepvelocity = 0;
    supportrate = 0;
    rollangle = 0;
    steplength = 0;
    wavevelocity_max = 0;
    stepvelocity_avr = 0;
    totaldistance_Mix = 0;
    totaldistance_Integral = 0;
    totaldistance_Fomula = 0;
    ENERGS = 0;
}

U8 GaitanalyseGetStancePercent()
{
    // printf("support rate is %d\n",(U8)supportrate);
    return (U8)supportrate;
}

U16 GaitanalyseGetswingSpeed()
{
    // printf("swingSpeed is %d m/s\n",(U16)wavevelocity_max);
    return (U16)wavevelocity_max;
}

U16 GaitanalyseGetSpeed()
{
    // printf("stepvelocity is %d cm/s\n", (U16)(stepvelocity_avr * 100));
    return (U16)(stepvelocity_avr * 100);
}
U8 GaitanalyseGetStrideLength()
{
    //  printf("steplen is %d cm\n", (U8)(steplength * 100));
    return (U8)(steplength * 100);
}
U8 GaitanalyseGetStepFrequence()
{
    //  printf("stepvelocity is %d STEP/min\n", (U8)(stepvelocity));
    return (U8)(stepvelocity);
}
INT32U ArithmeticGetStepNum()
{
    // printf("countstep is %d\n",(U16)countstep_now);
    return (U16)countstep_now;
}
INT32U ArithmeticGetTotaldistance()
{
    // printf("totaldistance is %d m\n",(INT32U)totaldistance_Integral);
    return (INT32U)totaldistance_Integral;
}
/*参数设置*/
void ArithmeticSetRL(INT8U rl)
{
    RIGHT_LEFT = rl;
};
void ArithmeticSetWeight(INT8U uW)
{
    human_weight = uW;
}
INT8U ArithmeticGetWeight()
{
    return human_weight;
}
void ArithmeticSetHeight(INT8U uW)
{
    human_height = uW;
}
INT8U ArithmeticGetHeight()
{
    return human_height;
}
void EnergyArithmatic()
{
    switch (ArithmeticGetPattern())
    {
    case 0:
        break;
    case 1:
        ENERGS += (U32)(human_weight * steplength * SPORT_FACTOR_WALK / 100);
        break;
    case 2:
        ENERGS += (U32)(human_weight * steplength * SPORT_FACTOR_RUN / 100);
        break;
    }
    // printf("energy now is %d",ENERGS);
}

INT8U ArithmeticGetPattern()
{
    if (supportrate < 40 && countstep > 2)
        Pattern = 2;
    else if (countstep == 0)
        Pattern = 0;
    else
        Pattern = 1;
    return Pattern;
}
U8 GaitanalyseGetVerticalLength()
{
    srand( (unsigned)time( NULL ));
    U8 height = rand();

    switch (ArithmeticGetPattern())
    {
    case 0:height = 0;
        break;
    case 1:
        height = (U8)(25 - height % 15);
        break;
    case 2:
        height = (U8)(15 - height % 15);
        break;
    }
    // printf("height now is %d cm", (unsigned)time( NULL ));
    return height;
}