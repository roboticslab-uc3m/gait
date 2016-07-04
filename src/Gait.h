#ifndef GAIT_H
#define GAIT_H

#include "tools.h"

using namespace teo;

class Gait
{
public:
    //Gait();
    virtual bool AddStepForward(int stepNumber)=0;
    virtual bool GetTrajectories(tra::SpaceTrajectory& RightFoot, tra::SpaceTrajectory& LeftFoot)=0;

};

#endif // GAIT_H
