#include "GaitSupportPoligon.h"

#include <iostream>
#include <math.h>       /* atan, sqrt */

using namespace std;


//kdl
//#include <frames.hpp>
#define rlaaOffset +3.0*0.017
#define llaaOffset +4.5*0.017
#define rlfyOffset -0.01
#define llfyOffset +0.01
#define rlfzOffset +0.005
#define llfzOffset +0.01

using namespace roboticslab;

double ftime=1;



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
   // desiredRightFoot.ChangePosition(0,-0.123*ankleAngle,0);    //convert ankle moving in just rotation
   // desiredRightFoot.ChangePosition(0, 0, -hipSquat/2);

    //apply hip balance
    //desiredRightFoot.ChangePosition(0,+hipAngle,-hipAngle);//-cos(hipAngle)*hipSideshift,-sin(hipAngle)*hipSideshift);
    //dt=2;
    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

//    trajLeftFoot.wait(0.5);
//    trajLeftFoot.wait(0.5);
//    trajLeftFoot.wait(1);
//    trajRightFoot.wait(1);

    //-3-left foot forward
    //forward up
    desiredLeftFoot.ChangePosition(kickDistance/2, llfyOffset, kickElevation);

    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);


    //forward down
    //desiredRightFoot.ChangePosition(0, -llfyOffset/2, 0);
    desiredLeftFoot.ChangePosition(kickDistance/2, llfyOffset, -kickElevation+llfzOffset);

    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //-4-reset ankle position after landing
    //remove angle
    desiredRightFoot.ChangeRotation(1,0,0,+ankleAngle);
 //   desiredRightFoot.ChangePosition(0,+0.123*ankleAngle,0);    //convert ankle moving in just rotation
//    desiredRightFoot.ChangePosition(0, 0, +hipSquat/2);


//    //remove hip balance
//    //desiredRightFoot.ChangePosition(0,-hipAngle,+hipAngle);//cos(hipAngle)*hipSideshift,sin(hipAngle)*hipSideshift);
//    //dt=4;
    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //-5-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-(dy+llfyOffset),-dz);
    desiredLeftFoot.ChangePosition(-dx,-(dy+llfyOffset),-dz-llfzOffset);
    //also, move root x axis half a swing positive (feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-kickDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-kickDistance/2,0,0);

    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);



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
    dy=-hipSideshift;//-llfyOffset;//offset hack for left step
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
    desiredLeftFoot.ChangeRotation(1,0,0,+ankleAngle);
  //  desiredLeftFoot.ChangePosition(0,+0.123*ankleAngle,0);    //convert ankle moving in just rotation
  //  desiredLeftFoot.ChangePosition(0, 0, -hipSquat/2);

    //apply hip balance
    //desiredLeftFoot.ChangePosition(0,-hipAngle,-hipAngle);//+cos(hipAngle)*hipSideshift,-sin(hipAngle)*hipSideshift);
    //dt=2;
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //desiredLeftFoot.ChangeRotation(1,0,0,+ankleAngle/10);
    //-8-right foot forward
    //forward up
    desiredRightFoot.ChangePosition(kickDistance/2, rlfyOffset, kickElevation);

    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);


    //desiredLeftFoot.ChangeRotation(1,0,0,-ankleAngle/10);
    //forward down
    //desiredLeftFoot.ChangePosition(0, -rlfyOffset, rlfzOffset);
    desiredRightFoot.ChangePosition(kickDistance/2, rlfyOffset, -kickElevation+rlfzOffset);

    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);


    //-9-reset ankle position after landing
    //remove angle
    desiredLeftFoot.ChangeRotation(1,0,0,-ankleAngle);
 //   desiredLeftFoot.ChangePosition(0,-0.123*ankleAngle,0);    //convert ankle moving in just rotation
//    desiredLeftFoot.ChangePosition(0, 0, +hipSquat/2);

    //remove hip balance
    //desiredLeftFoot.ChangePosition(0,+hipAngle,+hipAngle);//-cos(hipAngle)*hipSideshift,sin(hipAngle)*hipSideshift);
    //dt=4;
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //-10-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-(dy+rlfyOffset),-dz-rlfzOffset);
    desiredLeftFoot.ChangePosition(-dx,-(dy+rlfyOffset),-dz);
    //also, move root x axis half a swing positive (or feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-kickDistance/2,hipSideshift,0);
    desiredLeftFoot.ChangePosition(-kickDistance/2,hipSideshift,0);

    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);


    //one step finished


    return true;
}





