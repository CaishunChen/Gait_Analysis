#ifndef __IMPLEMENT_H__
#define __IMPLEMENT_H__
#include "config.h"
// extern U16 countstep = 0;
// extern U16 stepvelocity = 0;
// extern U8 supportrate = 0;
// extern F32 rollangle = 0;
// extern U8 steplength = 0;
// extern F32 wavevelocity_max = 0;
// extern F32 stepvelocity_avr = 0;

// extern F32 totaldistance_Mix = 0;
// extern F32 totaldistance_Integral = 0;
// extern F32 totaldistance_Fomula = 0;

/*******获取左右脚***********/
extern void ArithmeticSetRL(INT8U rl);
/***************************************************************************************************
函数:   获取步频   步/min
输入:

返回:   步频
***************************************************************************************************/
extern U8 GaitanalyseGetStepFrequence(void);
/***************************************************************************************************
函数:   获取步幅  单位cm
输入:

返回:   步幅
***************************************************************************************************/
extern U8 GaitanalyseGetStrideLength(void);
/***************************************************************************************************
函数:   获取步速  cm/min
输入:

返回:   步速
***************************************************************************************************/
extern U16 GaitanalyseGetSpeed(void);
/***************************************************************************************************
函数:   获取摆速  m/s
输入:

返回:   摆速
***************************************************************************************************/
extern U16 GaitanalyseGetswingSpeed(void);
/***************************************************************************************************
函数:   获取支撑相占比
输入:

返回:   支撑相占比
***************************************************************************************************/
extern U8 GaitanalyseGetStancePercent(void);
/***************************************************************************************************
函数:   获取离地高度 cm
输入:

返回:   离地高度
***************************************************************************************************/
extern U8 GaitanalyseGetVerticalLength(void);

extern void ArithProcessNewAccData(INT16S * piNewAcc, INT8U uNum);
#endif