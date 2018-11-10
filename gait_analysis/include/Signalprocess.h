#ifndef __SIGNALPROCESS_H__
#define __SIGNALPROCESS_H__
#include <vector>
using namespace std;

extern vector<int> findPeaks(vector<double> src, double distance,double lowbound,double upbound);
extern vector<int> findVallies(vector<double> src, double distance,double lowbound,double upbound);  
extern vector<vector<int>> findNearzeros(vector<double> src, int minnum, double lowbound, double upbound);


#endif