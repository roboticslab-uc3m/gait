#include <iostream>
#include <vector>
#include <math.h>       /* atan, sqrt */

using namespace std;


#include "GaitSupportPoligon.h"

//kdl
//#include <frames.hpp>
#define rlaaOffset +3*0.017
#define llaaOffset -3*0.017

using namespace teo;




bool GaitSupportPoligon::HalfStepForwardRS()
{

    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
    double dx,dy,dz;
    double ankleAngle,hipAngle;
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
    ankleAngle = atan( hipSideshift / 2*sqrt(pow(legHeight,2)-pow(hipSideshift,2)) );
    ankleAngle += rlaaOffset;
    hipAngle = 0.5*hipSquat;
    //std::cout << "VALUES: "<< hipSideshift<< ","<<legWeight << ","<< legHeight<< ","<<hipSideshift << ","<< ankleAngle<< ","<<std::endl;
    //check angle sign before apply!!
    //apply angle
    desiredRightFoot.ChangeRotation(1,0,0,-ankleAngle);
    //apply hip balance
    //desiredRightFoot.ChangePosition(0,+hipAngle,-hipAngle);//-cos(hipAngle)*hipSideshift,-sin(hipAngle)*hipSideshift);
    dt=2;
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //-3-left foot forward
    //forward up
    desiredLeftFoot.ChangePosition(kickDistance/2, 0, kickElevation);
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //forward down
    desiredLeftFoot.ChangePosition(kickDistance/2, 0, -kickElevation);
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);



    //-5-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-kickDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-kickDistance/2,0,0);

    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //-4-reset ankle position after landing
    //remove angle
    desiredRightFoot.ChangeRotation(1,0,0,+ankleAngle);
    //remove hip balance
    //desiredRightFoot.ChangePosition(0,-hipAngle,+hipAngle);//cos(hipAngle)*hipSideshift,sin(hipAngle)*hipSideshift);
    dt=2;
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //half step finished


    return true;
}

bool GaitSupportPoligon::HalfStepForwardLS()
{

    //double x,y,z; actualRightFoot.GetPosition(x,y,z);
    double dx,dy,dz;
    double ankleAngle,hipAngle;
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
    ankleAngle += llaaOffset;
    hipAngle = 0.5*hipSquat;

    //check angle sign before apply!!
    //std::cout << "VALUES: "<< hipSideshift<< ","<< legHeight<< ","<<hipSideshift << ","<< ankleAngle<< ","<<std::endl;

    //apply angle
    desiredLeftFoot.ChangeRotation(1,0,0,-ankleAngle);
    //apply hip balance
    //desiredLeftFoot.ChangePosition(0,-hipAngle,-hipAngle);//+cos(hipAngle)*hipSideshift,-sin(hipAngle)*hipSideshift);
    dt=2;
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);


    //-8-right foot forward
    //forward up
    desiredRightFoot.ChangePosition(kickDistance/2, 0, kickElevation);
    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //forward down
    desiredRightFoot.ChangePosition(kickDistance/2, 0, -kickElevation);
    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);




    //-10-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (or feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-kickDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-kickDistance/2,0,0);

    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //-9-reset ankle position after landing
    //remove angle
    desiredLeftFoot.ChangeRotation(1,0,0,+ankleAngle);
    //remove hip balance
    //desiredLeftFoot.ChangePosition(0,+hipAngle,+hipAngle);//-cos(hipAngle)*hipSideshift,sin(hipAngle)*hipSideshift);
    dt=2;
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //one step finished


    return true;
}





