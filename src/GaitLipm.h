#ifndef GAITLIPM_H
#define GAITLIPM_H


#include "Gait.h"
#include "tools.h"
#include <stdio.h>

namespace teo {

class GaitLipm : public Gait
{
public:
    GaitLipm(kin::Pose initialRightFoot, kin::Pose initialLeftFoot) :
        Gait(initialRightFoot,initialLeftFoot){}

private:
    //Pure virtual Definitions.
    bool HalfStepForwardRS();
    bool HalfStepForwardLS();

    long LipForceZMP(const double & xzmp, const double & yzmp, double & x);

    physics::StateVariable mx,my,mz; //inverted pendulum x,y,z mass position from base (foot) variables


};

} //namespace teo

#endif // GAITLIPM_H
