#ifndef __SIGNALPROCESS_H__
#define __SIGNALPROCESS_H__

#include "config.h"
using namespace std;
extern void findPeaks(float* src,int src_lenth,int* indMax,int* indMax_len,U16 distance, F32 lowbound, F32 upbound);
extern void findVallies(float* src,int src_lenth,int* indMin,int* indMin_len,U16 distance, F32 lowbound, F32 upbound);
extern void findNearzeros(float *src, int len, int *nz,int* resize,
                                     U16 minnum, F32 lowbound, F32 upbound);
#endif