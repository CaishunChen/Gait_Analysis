#ifndef __QUATERNION_H__
#define __QUATERNION_H__
#include <vector>

using namespace std;
extern struct ANGLE IMUupdate(float ax, float ay, float az,float gx, float gy, float gz,short frq);
extern struct COORD transform(vector<double>q4,double x,double y,double z);
struct ANGLE
{
    float X, Y, Z;
    vector<double> quter;
};

struct COORD
{
    float x, y, z;
};
#endif