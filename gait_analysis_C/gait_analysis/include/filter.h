#ifndef __FILTER_H__
#define __FILTER_H__

#include "config.h"

using namespace std;

extern void Raw2Real_A(F32 *input,U16 input_length, F32 rate);
extern void Mean_Filter_A(F32 *input,U16 input_length, U16 windowsize);
extern void Kalman_Filter_A(F32 *input,U16 len);

#endif