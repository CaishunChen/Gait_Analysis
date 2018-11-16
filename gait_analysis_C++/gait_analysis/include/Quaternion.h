#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "config.h"
#include <vector>

using namespace std;
extern struct ANGLE IMUupdate(F32 ax, F32 ay, F32 az,F32 gx, F32 gy, F32 gz,U16 frq);
extern struct COORD transform(vector<F32>q4,F32 x,F32 y,F32 z);
struct ANGLE
{
    F32 X, Y, Z;
    vector<F32> quter;
};

struct COORD
{
    F32 x, y, z;
};
#endif