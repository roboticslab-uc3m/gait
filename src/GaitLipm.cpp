#include "GaitLipm.h"


#include <iostream>
#include <vector>
#include <math.h>       /* atan, sqrt */

using namespace std;

//kdl
//#include <frames.hpp>

using namespace teo;




GaitLipm::GaitLipm(kin::Pose initialRightFoot, kin::Pose initialLeftFoot, double newMass) :
    Gait(initialRightFoot,initialLeftFoot)
{

    double gravity = -9.81;
    double height = this->legHeight;
    double inertia = newMass*height*height;


    lipMass=newMass;
    k1 = inertia/(lipMass*9.81/*gravity*/);
    k2 = height/gravity;

    comDisplacement = (hipSideshift/2)*0.32;//32% is the weight percentage of legs
    comHeight = 0.3;

}

long GaitLipm::LipmInitialState(physics::StateVariable mx0, physics::StateVariable my0, physics::StateVariable mz0)
{
    mx=mx0;
    my=my0;
    mz=mz0;

    return 0;
}

long GaitLipm::LipmInitialState(std::vector<double> &xwp, std::vector<double> &ywp, std::vector<double> &zwp, double dt)
{

    mx = physics::StateVariable(xwp.back(),(xwp.back()-xwp[xwp.size()-2])/dt,0);
    my = physics::StateVariable(ywp.back(),(ywp.back()-ywp[xwp.size()-2])/dt,0);
    mz = physics::StateVariable(zwp.back(),(zwp.back()-zwp[xwp.size()-2])/dt,0);
    return 0;
}

long GaitLipm::LipmInitialState(const std::vector<double> &xyz0, const std::vector<double> &xyz1, const std::vector<double> &D)
{

}

double GaitLipm::GetSwingYInitialSpeed(double initialY, double swingTime)
{
    return initialY*( (k1/k2)-(1-swingTime) );
}


double GaitLipm::LipInitAndGetZmpTrajectory(std::vector<double> &xwp, std::vector<double> &ywp, std::vector<double> &zwp, double dt)
{
    double timeSpent = 0;
    if( (xwp.size()<2)|(ywp.size()<2)|(zwp.size()<2) )
    {
        std::cout << "Please initialize x,y,z vectors with two values." << std::endl;
        return -1;
    }

    //initalize the lipm
    LipmInitialState(xwp,ywp,zwp,dt);

//    std::cout << "order: " << mx.GetOrder() << ", x: " << mx.D(0) << ", Dx: "  << mx.D(1) << ", D2x: "  << mx.D(2)  << std::endl;
//    std::cout << "order: " << my.GetOrder() << ", y: " << my.D(0) << ", Dy: "  << my.D(1) << ", D2y: "  << my.D(2)  << std::endl;
//    std::cout << "order: " << mz.GetOrder() << ", z: " << mz.D(0) << ", Dz: "  << mz.D(1) << ", D2z: "  << mz.D(2)  << std::endl;

    //std::cout << "std::abs(ywp.back()): " << std::abs(ywp.back()) << ", std::abs(ywp[0]): " << std::abs(ywp[0]) << std::endl;

    //Compute trajectory for a lipm half cycle with zmp 0,0
    while( std::abs(ywp.back()) < std::abs(ywp[0]) )
    {
        ChangeMassPosition(dt,0,0);
        xwp.push_back( mx.D(0) );
        ywp.push_back( my.D(0) );
        zwp.push_back( mz.D(0) );
        timeSpent += dt;
        std::cout << "newx: " << mx.D(0)<< ", newy: " << my.D(0)<< ", newz: " << mz.D(0) << std::endl;
    }

    return timeSpent;
}


double GaitLipm::LipZmpTrajectory(std::vector<double> &xwp, std::vector<double> &ywp, std::vector<double> &zwp, double dt)
{
    double timeSpent = 0;

    xwp.clear();
    ywp.clear();
    zwp.clear();
//    std::cout << "order: " << mx.GetOrder() << ", x: " << mx.D(0) << ", Dx: "  << mx.D(1) << ", D2x: "  << mx.D(2)  << std::endl;
//    std::cout << "order: " << my.GetOrder() << ", y: " << my.D(0) << ", Dy: "  << my.D(1) << ", D2y: "  << my.D(2)  << std::endl;
//    std::cout << "order: " << mz.GetOrder() << ", z: " << mz.D(0) << ", Dz: "  << mz.D(1) << ", D2z: "  << mz.D(2)  << std::endl;

    //std::cout << "std::abs(ywp.back()): " << std::abs(ywp.back()) << ", std::abs(ywp[0]): " << std::abs(ywp[0]) << std::endl;

    //Compute trajectory for a lipm half cycle with zmp 0,0
    do
    {
        ChangeMassPosition(dt,0,0);
        xwp.push_back( mx.D(0) );
        ywp.push_back( my.D(0) );
        zwp.push_back( mz.D(0) );
        timeSpent += dt;
        //std::cout << "newx: " << mx.D(0)<< ", newy: " << my.D(0)<< ", newz: " << mz.D(0) << std::endl;
    }
    while( std::abs(ywp.back()) <= std::abs(ywp[0]) );

    return timeSpent;
}


long GaitLipm::LipmAngularResponse(std::vector<double> &tiltwp, double dt, double radius)
{
    physics::StateVariable tilt(tiltwp.back(),(tiltwp.back()-tiltwp[tiltwp.size()-2])/dt,0);

    double newTilt;

    while( std::abs(tiltwp.back()) < std::abs(tiltwp[0]) )
    {
        //from the solution of the second order pendulum equation
        newTilt = ( tilt.D(0)/(dt*dt) + tilt.D(1)/dt ) / ( -9.81/radius + 1/(dt*dt) );
        tilt.Update(newTilt,dt);
        tiltwp.push_back( tilt.D(0) );

        //std::cout << "tilt.D(0): " << tilt.D(0) << std::endl;
    }



}

bool GaitLipm::HalfStepForwardRS()
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


    //-2-ZMP balance over right foot


    //define the pose of com from root.
    kin::Pose comFromRoot(0,comDisplacement,comHeight);
    kin::Pose rootFromCom = comFromRoot.Inverse();
//    comFromRoot.Print("comFromRoot");
//    rootFromCom.Print("rootFromCom");


    //define the pose of com from the actual support foot.
    kin::Pose tFootCom(desiredRightFoot,comFromRoot);
    kin::Pose tComFoot = tFootCom.Inverse();
//    tFootCom.Print("tFootCom");
//    tComFoot.Print("tComFoot");

    //pendulum mass variables x,y,z from the foot in the pendulum frame (z:height, y:swing, x:0)
    mx = physics::StateVariable(comFromRoot.GetX() - desiredRightFoot.GetX(),0,0);
    my = physics::StateVariable(comFromRoot.GetY() - desiredRightFoot.GetY(),-dy/dt,0);
    mz = physics::StateVariable(comFromRoot.GetZ() - desiredRightFoot.GetZ(),0,0);
    //LipmInitialState(sx,sy,sz); (Already initialized in latter lines)


    dt=0.01;
    double totalSwingTime = LipZmpTrajectory(ptx,pty,ptz,dt);
    std::cout << "totalSwingTime: " << totalSwingTime << std::endl;

    long nKickUp = (long)( ptx.size()/2 );
    long nKickDown = ptx.size()-nKickUp;

    double dxKick = kickDistance/ptx.size();
    double dyKick = 0;
    double dzKick = kickElevation/nKickUp;

    //update next y values for com from the foot

    //kin::Pose footfinal;

    for (long i=0; i<nKickUp; i++)
    {
        //update pendulum mass position
        tComFoot.SetPosition(ptx[i],-pty[i],-ptz[i]);
        //computation of foot position based on com position
        desiredRightFoot = kin::Pose(rootFromCom,tComFoot);
        desiredLeftFoot.ChangePosition(dxKick, dyKick, dzKick);

//        desiredRightFoot.Print("desiredRightFoot");
//        comFromRoot.Inverse().Print("comFromRoot.Inverse()");
//        tFootCom.Inverse().Print("tFootCom.Inverse()");

        trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);
        trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    }

    dz = -kickElevation/nKickDown;

    for (long i=0; i<nKickDown; i++)
    {
        //update pendulum mass position
        tComFoot.SetPosition(ptx[i],-pty[i],-ptz[i]);
        //computation of foot position based on com position
        desiredRightFoot = kin::Pose(rootFromCom,tComFoot);
        desiredLeftFoot.ChangePosition(dxKick, dyKick, dzKick);

//        desiredRightFoot.Print("desiredRightFoot");
//        comFromRoot.Inverse().Print("comFromRoot.Inverse()");
//        tFootCom.Inverse().Print("tFootCom.Inverse()");

        trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);
        trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    }

//    //feed zmp trajectory in time
//    //retrieve list of points same size as zmp trajectory

//    desiredRightFoot.ChangeRotation(1,0,0,-ankleAngle);
//    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
//    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

//    //-3-left foot forward
//    //forward up
//    desiredLeftFoot.ChangePosition(kickDistance/2, 0, kickElevation);
//    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
//    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

//    //forward down
//    desiredLeftFoot.ChangePosition(kickDistance/2, 0, -kickElevation);
//    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
//    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

//    //-4-reset ankle position after landing
//    //remove angle
//    //desiredRightFoot.SetRotation(cux,cuy,cuz,cangle);
//    desiredRightFoot.ChangeRotation(1,0,0,ankleAngle);
//    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
//    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);

    //-5-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-kickDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-kickDistance/2,0,0);

    dt=trajRightFoot.AddWaypoint(desiredRightFoot);
    trajLeftFoot.AddTimedWaypoint(dt,desiredLeftFoot);


    //half step finished


    return true;
}

bool GaitLipm::HalfStepForwardLS()
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
    desiredLeftFoot.ChangeRotation(1,0,0,ankleAngle);
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
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


    //-9-reset ankle position after landing
    //remove angle
    desiredLeftFoot.ChangeRotation(1,0,0,-ankleAngle);
    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //-10-move root over center again (undo former feet movement)
    desiredRightFoot.ChangePosition(-dx,-dy,-dz);
    desiredLeftFoot.ChangePosition(-dx,-dy,-dz);
    //also, move root x axis half a swing positive (or feet x axis half a swing negative)
    desiredRightFoot.ChangePosition(-kickDistance/2,0,0);
    desiredLeftFoot.ChangePosition(-kickDistance/2,0,0);

    dt=trajLeftFoot.AddWaypoint(desiredLeftFoot);
    trajRightFoot.AddTimedWaypoint(dt,desiredRightFoot);

    //one step finished


    return true;
}


long GaitLipm::ChangeMassPosition(double dt, double xzmp, double yzmp)
{
    double newx,newy;
    double dt2=dt*dt;

    //kv=k1/dt;
    //kp=(k2/dt*dt)+k1/dt;

    //compute new x position based on xzmp and former x variable
    newx = ( xzmp - mx.D(0)*(k2/dt2+k1/dt) - mx.D(1)*(k2/dt) ) / ( 1-k2/dt2-k1/dt );


//    kv=-k1/dt;
//    kp=(k2/dt*dt)-k1/dt;

    //compute new y position based on yzmp and former y variable
    newy = ( yzmp - my.D(0)*(k2/dt2-k1/dt) - my.D(1)*(k2/dt) ) / ( 1-k2/dt2+k1/dt );

//    std::cout << "-kv*my.D(1): " << my.D(1) << ", -kp*my.D(0): " << my.D(0) <<std::endl;

//    std::cout << "nx: " << newx << ", ny: " << newy <<std::endl;

    mx.Update(newx,dt);
    my.Update(newy,dt);

    return 0;


}

