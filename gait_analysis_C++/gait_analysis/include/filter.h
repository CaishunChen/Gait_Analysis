#ifndef __FILTER_H__
#define __FILTER_H__

#include "config.h"
#include <vector>

using namespace std;

extern vector<F32> Raw2Real(vector<F32> input,F32 rate);
extern vector<F32> Mean_Filter(vector<F32> input, U16 windowsize);
extern vector<F32> Kalman_Filter(vector<F32> input);


#endif