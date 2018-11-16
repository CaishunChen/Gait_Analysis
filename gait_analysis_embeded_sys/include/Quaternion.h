#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include "config.h"


extern struct ANGLE IMUupdate(F32 ax, F32 ay, F32 az,F32 gx, F32 gy, F32 gz,U16 frq);
extern struct COORD transform(F32 q0,F32 q1,F32 q2,F32 q3,F32 x,F32 y,F32 z);
struct ANGLE
{
    F32 X, Y, Z;
    F32 q0,q1,q2,q3;
};

struct COORD
{
    F32 x, y, z;
};
#endif