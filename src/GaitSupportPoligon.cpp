#include <iostream>
#include <vector>
#include <math.h>       /* atan, sqrt */

using namespace std;

#include "GaitSupportPoligon.h"

//kdl
//#include <frames.hpp>




bool GaitSupportPoligon::SaveSpaceTrajectories(ofstream &fileLeftFoot, ofstream &fileRightFoot)
{

    trajLeftFoot.SaveToFile(fileLeftFoot);
    trajRightFoot.SaveToFile(fileRightFoot);
    return true;
}



GaitSupportPoligon::GaitSupportPoligon(kin::Pose initialRightFoot, kin::Pose initialLeftFoot)
{
    trajRightFoot.AddTimedWaypoint(-1, initialRightFoot);
    trajLeftFoot.AddTimedWaypoint(-1, initialLeftFoot);

    SetSwingParameters(0, 0);
    SetSupportParameters(0);
    legHeight = initialLeftFoot.GetZ();

    startOnRightFootSupport = true;

}


bool GaitSupportPoligon::SetSwingParameters( double swingFootDistance, double swingFootElevation )
{
    swingDistance = swingFootDistance;
    swingElevation = swingFootElevation;

    return true;
}

bool GaitSupportPoligon::SetSupportParameters(double new_hipSideshift)
{
    hipSideshift = new_hipSideshift;

    return true;
}

bool GaitSupportPoligon::GetTrajectories(tra::SpaceTrajectory& getRightFoot, tra::SpaceTrajectory& getLeftFoot)
{

    getRightFoot = trajRightFoot;
    getLeftFoot = trajLeftFoot;

}


bool GaitSupportPoligon::AddStepForward(int stepNumber)
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
    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
   /* double dx,dy,dz;
    Pose actualRightFoot, actualLeftFoot;
    Pose desiredRightFoot,desiredLeftFoot;

    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);


    //strategy:
    //-1-move root over right foot (right foot under root (0,0,z), z is actual foot elevation)
    //calculate right foot

    //origin (x,y,z) destination (0,0,z)
    dx=0-actualRightFoot.GetX();
    dy=0-actualRightFoot.GetY();
    dz=0;
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

    //left foot moves parallel to right foot
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-2-balance over right foot
    //TODO


    //-3-left foot forward
    //forward up
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, swingElevation);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //forward down
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, -swingElevation);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-4-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-5-move root over left foot (left foot under root (0,0,z), z is actual foot elevation)
    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);

    //origin (x,y,z) destination (0,0,z)
    dx=0-actualLeftFoot.GetX();
    dy=0-actualLeftFoot.GetY();
    dz=0;
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    //right foot moves parallel to left foot
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-6-balance over left foot.
    //TODO


    //-7-right foot forward
    //trajLeftFoot.GetCurrentPose(desiredLeftFoot);
    desiredRightFoot.ChangePosition(swingDistance/2, 0, swingElevation);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //trajLeftFoot.GetCurrentPose(desiredLeftFoot);
    desiredRightFoot.ChangePosition(swingDistance/2, 0, -swingElevation);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-8-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (or feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);*/

    //one step finished


    return true;
}

bool GaitSupportPoligon::HalfStepForwardRS()
{

    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
    double dx,dy,dz;
    double ankleAngle;

    kin::Pose actualRightFoot, actualLeftFoot;
    kin::Pose desiredRightFoot,desiredLeftFoot;

    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);


    //strategy:
    //-1-move root hipSideshift meters over right foot ( (0,-hipSideshift,z), z is actual foot elevation)
    //calculate right foot

    //origin (x,y,z) destination (0,0-hipSideshift,z)
    dx=0-actualRightFoot.GetX();
    dy=+hipSideshift;
    dz=0;
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

    //left foot moves parallel to right foot
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-2-balance over right foot
    //get ankle angle that balances robot over right foot
    ankleAngle = atan( hipSideshift / 2*sqrt(pow(legHeight,2)-pow(hipSideshift,2)) );
    //std::cout << "VALUES: "<< hipSideshift<< ","<<legWeight << ","<< legHeight<< ","<<hipSideshift << ","<< ankleAngle<< ","<<std::endl;
    //check angle sign before apply!!
    //apply angle
    desiredRightFoot.SetRotation(1,0,0,-ankleAngle);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-3-left foot forward
    //forward up
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, swingElevation);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //forward down
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, -swingElevation);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-4-reset ankle position after landing
    //remove angle
    desiredRightFoot.SetRotation(1,0,0,0);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-5-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //half step finished


    return true;
}

bool GaitSupportPoligon::HalfStepForwardLS()
{

    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
    double dx,dy,dz;
    double ankleAngle;


    kin::Pose actualRightFoot, actualLeftFoot;
    kin::Pose desiredRightFoot,desiredLeftFoot;

    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);



    //-6-move root over left foot (left foot under root (0,0,z), z is actual foot elevation)
    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);

    //origin (x,y,z) destination (0,0,z)
    dx=0-actualLeftFoot.GetX();
    dy=-hipSideshift;
    dz=0;
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    //right foot moves parallel to left foot
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-7-balance over left foot.
    //get ankle angle that balances robot over left foot
    ankleAngle = atan( hipSideshift / 2*sqrt(pow(legHeight,2)-pow(hipSideshift,2)) );
    //check angle sign before apply!!
    //std::cout << "VALUES: "<< hipSideshift<< ","<< legHeight<< ","<<hipSideshift << ","<< ankleAngle<< ","<<std::endl;

    //apply angle
    desiredLeftFoot.SetRotation(1,0,0,+ankleAngle);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);


    //-8-right foot forward
    //trajLeftFoot.GetCurrentPose(desiredLeftFoot);
    desiredRightFoot.ChangePosition(swingDistance/2, 0, swingElevation);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //trajLeftFoot.GetCurrentPose(desiredLeftFoot);
    desiredRightFoot.ChangePosition(swingDistance/2, 0, -swingElevation);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-9-reset ankle position after landing
    //remove angle
    desiredLeftFoot.SetRotation(1,0,0,0);
    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //-10-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (or feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddWaypoint(desiredLeftFoot);

    //one step finished


    return true;
}
