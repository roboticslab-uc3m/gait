#include <iostream>
#include <vector>
#include <math.h>       /* atan, sqrt */

using namespace std;


#include "GaitSupportPoligon.h"

//kdl
//#include <frames.hpp>

using namespace teo;



bool GaitSupportPoligon::SaveSpaceTrajectories(ofstream &fileRightFoot, ofstream &fileLeftFoot)
{

    trajLeftFoot.SaveToFile(fileLeftFoot);
    trajRightFoot.SaveToFile(fileRightFoot);
    return true;
}



GaitSupportPoligon::GaitSupportPoligon(kin::Pose initialRightFoot, kin::Pose initialLeftFoot)
{
    trajRightFoot.SetInitialWaypoint(initialRightFoot);
    trajLeftFoot.SetInitialWaypoint(initialLeftFoot);

    SetSwingParameters(0, 0);
    SetHipParameters(0,0);
    legHeight = initialLeftFoot.GetZ();

    startOnRightFootSupport = true;

}

bool GaitSupportPoligon::BeforeStep()
{
    //Reduce hip elevation by changing z coordinate on both feet.
    trajLeftFoot.move(0,0,hipLower);
    trajRightFoot.move(0,0,hipLower);

}


bool GaitSupportPoligon::SetSwingParameters( double swingFootDistance, double swingFootElevation )
{
    swingDistance = swingFootDistance;
    swingElevation = swingFootElevation;

    return true;
}

bool GaitSupportPoligon::SetHipParameters(double new_hipSideshift, double new_hipLower)
{
    hipSideshift = new_hipSideshift;
    hipLower = new_hipLower;

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


    //one step finished


    return true;
}

bool GaitSupportPoligon::HalfStepForwardRS()
{

    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
    double dx,dy,dz;
    double ankleAngle;
    double dt;

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

    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);


    //-2-balance over right foot
    //get ankle angle that balances robot over right foot
    ankleAngle = 0.04;//atan( hipSideshift / 2*sqrt(pow(legHeight,2)-pow(hipSideshift,2)) );
    //std::cout << "VALUES: "<< hipSideshift<< ","<<legWeight << ","<< legHeight<< ","<<hipSideshift << ","<< ankleAngle<< ","<<std::endl;
    //check angle sign before apply!!
    //apply angle
    //desiredRightFoot.SetRotation(1,0,0,-ankleAngle);
    double cux,cuy,cuz,cangle;
    //desiredRightFoot.GetRotation(cux,cuy,cuz,cangle);
    //desiredRightFoot.SetRotation(-0.69,-0.2,0.69,2.91);
    //desiredRightFoot.ChangeRotation(0,0,1,-ankleAngle);
    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //-3-left foot forward
    //forward up
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, swingElevation);
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //forward down
    desiredLeftFoot.ChangePosition(swingDistance/2, 0, -swingElevation);
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //-4-reset ankle position after landing
    //remove angle
    //desiredRightFoot.SetRotation(cux,cuy,cuz,cangle);
//    desiredRightFoot.ChangeRotation(0,0,1,ankleAngle);
    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //-5-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);


    //half step finished


    return true;
}

bool GaitSupportPoligon::HalfStepForwardLS()
{

    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
    double dx,dy,dz;
    double ankleAngle;
    double dt;


    kin::Pose actualRightFoot, actualLeftFoot;
    kin::Pose desiredRightFoot,desiredLeftFoot;

    //get actual foot poses
    trajRightFoot.GetLastWaypoint(actualRightFoot);
    trajLeftFoot.GetLastWaypoint(actualLeftFoot);



    //-6-move root over left foot (left foot under root (0,0,z), z is actual foot elevation)
    //trajRightFoot.GetLastWaypoint(actualRightFoot);
    //trajLeftFoot.GetLastWaypoint(actualLeftFoot);

    //origin (x,y,z) destination (0,0,z)
    dx=0-actualLeftFoot.GetX();
    dy=-hipSideshift;
    dz=0;
    desiredLeftFoot=actualLeftFoot;
    desiredLeftFoot.ChangePosition(dx,dy,dz);

    //right foot moves parallel to left foot
    desiredRightFoot=actualRightFoot;
    desiredRightFoot.ChangePosition(dx,dy,dz);

    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);


    //-7-balance over left foot.
    //get ankle angle that balances robot over left foot
    ankleAngle = atan( hipSideshift / 2*sqrt(pow(legHeight,2)-pow(hipSideshift,2)) );
    //check angle sign before apply!!
    //std::cout << "VALUES: "<< hipSideshift<< ","<< legHeight<< ","<<hipSideshift << ","<< ankleAngle<< ","<<std::endl;

    //apply angle
    //desiredLeftFoot.SetRotation(1,0,0,+ankleAngle);
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);


    //-8-right foot forward
    //forward up
    desiredRightFoot.ChangePosition(swingDistance/2, 0, swingElevation);
    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //forward down
    desiredRightFoot.ChangePosition(swingDistance/2, 0, -swingElevation);
    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);


    //-9-reset ankle position after landing
    //remove angle
    //desiredLeftFoot.SetRotation(1,0,0,0);
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //-10-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (or feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-swingDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-swingDistance/2,0,0);

    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //one step finished


    return true;
}





