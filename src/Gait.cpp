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

    std::cout << "setting default gait parameters: " << std::endl;
    SetKickParameters(0.05, 0.05); //(Kick distance, Kick height). revisar valores
    SetHipParameters(0.10, 0.10); //(hip sideshift, hip squat). revisar estos valores
    std::cout << "SetKickParameters( " << kickDistance << ", " << kickElevation << " )" << std::endl;
    std::cout << "SetHipParameters( "<< hipSideshift << ", " << hipSquat << " )" <<std::endl;
    legHeight = initialLeftFoot.GetZ();

    startOnRightFootSupport = true;

}


bool Gait::BeforeStep()
{
    //Reduce hip elevation by changing z coordinate on both feet.
    trajLeftFoot.move(0,0,hipSquat);
    trajRightFoot.move(0,0,hipSquat);

}

bool Gait::AfterStep()
{
    //Reduce hip elevation by changing z coordinate on both feet.
    trajLeftFoot.move(0,0,-hipSquat);
    trajRightFoot.move(0,0,-hipSquat);

}

bool Gait::SaveSpaceTrajectories(ofstream &fileRightFoot, ofstream &fileLeftFoot)
{

    trajLeftFoot.SaveToFile(fileLeftFoot);
    trajRightFoot.SaveToFile(fileRightFoot);
    return true;
}

bool Gait::SetKickParameters( double kickFloatingFootDistance, double kickFloatingFootElevation )
{
    kickDistance = kickFloatingFootDistance;
    kickElevation = kickFloatingFootElevation;

    return true;
}

bool Gait::SetHipParameters(double new_hipSideshift, double new_hipLower)
{
    hipSideshift = new_hipSideshift;
    hipSquat = new_hipLower;

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
