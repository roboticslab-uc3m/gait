#include "Gait.h"

#include <iostream>
#include <vector>
#include <math.h>       /* atan, sqrt */

using namespace std;

//kdl
//#include <frames.hpp>

using namespace teo;





Gait::Gait(kin::Pose initialRightFoot, kin::Pose initialLeftFoot)
{
    trajRightFoot.SetInitialWaypoint(initialRightFoot);
    trajLeftFoot.SetInitialWaypoint(initialLeftFoot);

    SetSwingParameters(0, 0);
    SetHipParameters(0,0);
    legHeight = initialLeftFoot.GetZ();

    startOnRightFootSupport = true;

}


bool Gait::BeforeStep()
{
    //Reduce hip elevation by changing z coordinate on both feet.
    trajLeftFoot.move(0,0,hipLower);
    trajRightFoot.move(0,0,hipLower);

}

bool Gait::SaveSpaceTrajectories(ofstream &fileRightFoot, ofstream &fileLeftFoot)
{

    trajLeftFoot.SaveToFile(fileLeftFoot);
    trajRightFoot.SaveToFile(fileRightFoot);
    return true;
}

bool Gait::SetSwingParameters( double swingFootDistance, double swingFootElevation )
{
    swingDistance = swingFootDistance;
    swingElevation = swingFootElevation;

    return true;
}

bool Gait::SetHipParameters(double new_hipSideshift, double new_hipLower)
{
    hipSideshift = new_hipSideshift;
    hipLower = new_hipLower;

    return true;
}

bool Gait::GetTrajectories(tra::SpaceTrajectory& getRightFoot, tra::SpaceTrajectory& getLeftFoot)
{

    getRightFoot = trajRightFoot;
    getLeftFoot = trajLeftFoot;

}


bool Gait::AddStepForward(int stepNumber)
{

    for (int i=0; i<stepNumber; i++)
    {
        if (startOnRightFootSupport)
        {
            HalfStepForwardRS();
            HalfStepForwardLS();
        }
        else
        {
            HalfStepForwardLS();
            HalfStepForwardRS();

        }
    }


    //one step finished


    return true;
}
