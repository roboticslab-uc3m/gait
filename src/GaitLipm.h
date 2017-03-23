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
    //Step Definitions.
    bool HalfStepForwardRS();
    bool HalfStepForwardLS();

    //long LipForceZMP(const double & xzmp, const double & yzmp, double & x);
    long ChangeComPosition(double dt, double xzmp, double yzmp);



    //Variables section
    physics::StateVariable mx,my,mz; //currrent inverted pendulum x,y,z mass position from base (foot) variables
    std::vector<double> trax,tray,traz,trat; //x,y,z, time trajectories
    double lipMass;
    double k1,k2,kp,kv;

};

} //namespace teo

#endif // GAITLIPM_H
