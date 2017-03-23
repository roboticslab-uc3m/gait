#ifndef GAITLIPM_H
#define GAITLIPM_H


#include "Gait.h"
#include "tools.h"
#include <stdio.h>

namespace teo {

class GaitLipm : public Gait
{
public:
    GaitLipm(kin::Pose initialRightFoot, kin::Pose initialLeftFoot, double newMass);

private:
    //Pure virtual Definitions.
    bool HalfStepForwardRS();
    bool HalfStepForwardLS();

    long LipForceZMP(const double & xzmp, const double & yzmp, double & x);
    long ChangeComPosition(double dt, double xzmp, double yzmp);

    physics::StateVariable mx,my,mz; //currrent inverted pendulum x,y,z mass position from base (foot) variables

    double lipMass;

};

} //namespace teo

#endif // GAITLIPM_H
