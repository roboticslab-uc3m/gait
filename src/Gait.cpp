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
    trajLeftFoot.move(0,0,hipSquat);
    trajRightFoot.move(0,0,hipSquat);

    double lateralInitial=hipSideshift;
    if (startOnRightFootSupport)
    {
        lateralInitial = -lateralInitial;
    }

    trajLeftFoot.move(0,lateralInitial,0);
    trajRightFoot.move(0,lateralInitial,0);

    trajLeftFoot.move(0,0,0);
    trajRightFoot.move(0,0,0);

    trajLeftFoot.move(0,-lateralInitial,0);
    trajRightFoot.move(0,-lateralInitial,0);


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
