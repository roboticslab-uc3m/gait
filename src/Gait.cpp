#include "Gait.h"

#include <iostream>
#include <math.h>       /* atan, sqrt */

using namespace std;

//kdl
//#include <frames.hpp>

using namespace roboticslab;




/**
 * @brief Gait::Gait: Constructor based on itial robot feet poses. Give the positions and rotations, and the
 * gait object will initialize, so the methods will work as expected.
 * @param initialRightFoot: Pose object with right foot position and rotation from the robot coordinate system.
 * @param initialLeftFoot: Pose object with left foot position and rotation from the robot coordinate system.
 */
Gait::Gait(kin::Pose initialRightFoot, kin::Pose initialLeftFoot)
{
    initialPoseRightFoot = initialRightFoot;
    initialPoseLeftFoot = initialLeftFoot;
    trajRightFoot.SetInitialWaypoint(initialRightFoot);
    trajLeftFoot.SetInitialWaypoint(initialLeftFoot);

    std::cout << "setting default gait parameters: " << std::endl;
    SetKickParameters(0.2, 0.2); //(Kick distance, Kick height). revisar valores
    SetHipParameters(0.10, 0.10, 0.15); //(hip sideshift, hip squat, hip speed). revisar estos valores
    std::cout << "SetKickParameters( " << kickDistance << ", " << kickElevation << " )" << std::endl;
    std::cout << "SetHipParameters( "<< hipSideshift << ", " << hipSquat << " )" <<std::endl;
    legHeight = initialLeftFoot.GetZ();

    startOnRightFootSupport = true;

}

Gait::~Gait()
{}

/**
 * @brief Gait::BeforeStep: Things to do before step (like Squat).
 * @return 0 for success. negative for error.
 */
long Gait::BeforeStep()
{
    //Reduce hip elevation by changing z coordinate on both feet.
    trajLeftFoot.moveTimed(0,0,hipSquat,hipSquat*200);
    trajRightFoot.moveTimed(0,0,hipSquat,hipSquat*200);

    double lateralInitial=hipSideshift;
    if (startOnRightFootSupport)
    {
        lateralInitial = -lateralInitial;
    }

//    trajLeftFoot.move(0,lateralInitial,0);
//    trajRightFoot.move(0,lateralInitial,0);

//    trajLeftFoot.move(0,0,0);
//    trajRightFoot.move(0,0,0);

//    trajLeftFoot.move(0,-lateralInitial,0);
//    trajRightFoot.move(0,-lateralInitial,0);


    return 0;

}
long Gait::Squat(double squatHeight)
{
    //Reduce hip elevation by changing z coordinate on both feet.
    trajLeftFoot.moveTimed(0,0,squatHeight,squatHeight*200);
    trajRightFoot.moveTimed(0,0,squatHeight,squatHeight*200);

    double lateralInitial=squatHeight;
    if (startOnRightFootSupport)
    {
        lateralInitial = -lateralInitial;
    }

//    trajLeftFoot.move(0,lateralInitial,0);
//    trajRightFoot.move(0,lateralInitial,0);

//    trajLeftFoot.move(0,0,0);
//    trajRightFoot.move(0,0,0);

//    trajLeftFoot.move(0,-lateralInitial,0);
//    trajRightFoot.move(0,-lateralInitial,0);

    
    return 0;
}



/**
 * @brief Gait::AfterStep: Things to do after step (like unSquat).
 * @return 0 for success. negative for error.
 */
long Gait::AfterStep()
{
    //Recover hip elevation by changing z coordinate on both feet.
    trajLeftFoot.move(0,0,-hipSquat*0.95);
    trajRightFoot.move(0,0,-hipSquat*0.95);

    return 0;
}

/**
 * @brief Gait::SaveSpaceTrajectories: Write actual computed trajectories in two files, one for each foot.
 * Format is time, position, and rotation(axis-angle) in csv, in that order: time, x, y, z, ux, uy, uz, angle.
 * @param fileRightFoot: Right foot file.
 * @param fileLeftFoot: Left foot file.
 * @return 0 for success. negative for error.
 */
long Gait::SaveSpaceTrajectories(ofstream &fileRightFoot, ofstream &fileLeftFoot)
{

    trajLeftFoot.SaveToFile(fileLeftFoot);
    trajRightFoot.SaveToFile(fileRightFoot);
    return 0;
}

/**
 * @brief Gait::SetKickParameters: Sets the floating foot parameters (kick) for the step.
 * @param kickFloatingFootDistance: Advance distance for the floating foot.
 * @param kickFloatingFootElevation: Elevation of floating foot from floor.
 * @return 0 for success. negative for error.
 */
long Gait::SetKickParameters( double kickFloatingFootDistance, double kickFloatingFootElevation )
{
    kickDistance = kickFloatingFootDistance;
    kickElevation = kickFloatingFootElevation;

    return 0;
}

/**
 * @brief Gait::SetHipParameters: Sets the hip displacement parameters.
 * @param new_hipSideshift(meters): The distance the robot will displace hip before the swing.
 * @param new_hipLower: The distance the robot will lower the hip before the step.
 * @return 0 for success. negative for error.
 */
long Gait::SetHipParameters(double new_hipSideshift, double new_hipSquat, double new_hipSpeed)
{
    hipSideshift = new_hipSideshift;
    hipSquat = new_hipSquat;
    hipSpeed = new_hipSpeed;

    return 0;
}

long Gait::SetDefaultSpeeds(double velocity, double rotspeed)
{
    trajLeftFoot.SetDefaultSpeeds(velocity, rotspeed);
    trajRightFoot.SetDefaultSpeeds(velocity, rotspeed);
}

/**
 * @brief Gait::GetTrajectories: Getter for the current trajectory objects.
 * @param getRightFoot: right foot trajectory object.
 * @param getLeftFoot: left foot trajectory object.
 * @return 0 for success. negative for error.
 */
long Gait::GetTrajectories(tra::SpaceTrajectory& getRightFoot, tra::SpaceTrajectory& getLeftFoot)
{

    getRightFoot = trajRightFoot;
    getLeftFoot = trajLeftFoot;
    return 0;

}

long Gait::GetSmoothTrajectories(tra::SpaceTrajectory& getRightFoot, tra::SpaceTrajectory& getLeftFoot, double xyz_accel, double rot_accel)
{

    kin::Pose rfwp, lfwp;
    kin::Pose rfVel1, lfVel1, rfVel2, lfVel2;
    kin::Pose add_rfwp, add_lfwp;
    double t,at;
    double dvx,dvy,dvz,dvr;
    double dpx,dpy,dpz,dar;
    double accx,accy,accz,accr;

    //init empty trajs
    trajRightFoot.GetWaypoint(0,rfwp);
    tra::SpaceTrajectory smoothRigth(rfwp);
    //init empty trajs
    trajLeftFoot.GetWaypoint(0,lfwp);
    tra::SpaceTrajectory smoothLeft(lfwp);

    //double rVel,vel;
    //double rvx,rvy,rvz;

    double dts=0.1;
    double dxdt=0.1/dts;

    long steps=0;


    for (int i=1;i<-1+trajLeftFoot.Size();i++)
    {

        //actual wp
        trajLeftFoot.GetWaypoint(i, lfwp);
        //time to reach i wp from i-1
        t = trajLeftFoot.GetWaypointTd(i);

        //vel from last wp (vel(0) for pose0 to pose1)
        trajLeftFoot.GetVelocitiesRel(i-1, lfVel1);
        //vel to next wp
        trajLeftFoot.GetVelocitiesRel(i, lfVel2);

        dvx = lfVel2.GetX()-lfVel1.GetX();
        dvy = lfVel2.GetY()-lfVel1.GetY();
        dvz = lfVel2.GetZ()-lfVel1.GetZ();
        dvr = lfVel2.Angle()-lfVel1.Angle();

        //acceleration time
        at=sqrt(dvx*dvx + dvy*dvy + dvz*dvz)/xyz_accel;
        at=max(at,dvr/rot_accel);
        //steps=max(max(max(dvx,dvy),dvz),dvr)/dts;
        steps = (long) at/dts;

        accx = dvx/at;
        accy = dvy/at;
        accz = dvz/at;
        accr = dvr/at;

        //first wp is actual
        add_lfwp=lfwp;

        //then add wp timed
        for (double st=0; st<at; st+=dts)
        {
            dpx=st*st*accx/2;
            dpy=st*st*accy/2;
            dpz=st*st*accz/2;
            dar=st*st*accr/2;
            add_lfwp.ChangePosition(dpx,dpy,dpz);
            add_lfwp.ChangeRotationAngle(dar);


        }


        for (int i=steps; i>=0; i--)
        {

        }


    }

    return 0;

}

/**
 * @brief Gait::AddStepForward: Add steps to exixting trajectory, in forward direction.
 * @param stepNumber: Number of steps to add.
 * @return 0 for success. negative for error.
 */
long Gait::AddStepForward(int stepNumber)
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


    return 0;
}
