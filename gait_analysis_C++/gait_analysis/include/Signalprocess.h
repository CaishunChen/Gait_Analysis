#ifndef __SIGNALPROCESS_H__
#define __SIGNALPROCESS_H__

#include "config.h"
#include <vector>
using namespace std;

extern vector<U16> findPeaks(vector<F32> src, F32 distance,F32 lowbound,F32 upbound);
extern vector<U16> findVallies(vector<F32> src, F32 distance,F32 lowbound,F32 upbound);  
extern vector<vector<U16> > findNearzeros(vector<F32> src, U16 minnum, F32 lowbound, F32 upbound);

#endif