#ifndef __FILTER_H__
#define __FILTER_H__
#include <vector>

using namespace std;

extern vector<double> Raw2Real(vector<int> input,double rate);
extern vector<double> Mean_Filter(vector<double> input, int windowsize);
extern vector<double> Kalman_Filter(vector<double> input);


#endif