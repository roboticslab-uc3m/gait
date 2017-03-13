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

};

} //namespace teo

#endif // GAITLIPM_H
