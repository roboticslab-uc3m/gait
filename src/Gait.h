#ifndef GAIT_H
#define GAIT_H

#include <fstream>  // std::ofstream

#include "tools.h"

namespace roboticslab
{

class Gait
{
public:
    //Gait();
    //virtual bool AddStepForward(int stepNumber)=0;
    //virtual bool GetTrajectories(tra::SpaceTrajectory& RightFoot, tra::SpaceTrajectory& LeftFoot)=0;


    /**
     * @brief spGait::spGait = Constructor with the initial feet poses.
     * Frame of reference for feet poses must be the center of mass (com) at robot default configuration.
     * @param initialRightFoot = Initial right foot pose from com.
     * @param initialLeftFoot = Initial left foot pose from com.
     */
    Gait(kin::Pose initialRightFoot, kin::Pose initialLeftFoot);

    virtual ~Gait();

    long Squat(double squatHeight);

    long BeforeStep();

    /**
     * @brief This function will add "stepNumber" steps in forward direction, using
     * support poligon stability criteria. Results are stored in the Space trajectory variable.
     * @param stepNumber = Number of steps to add.
     * @return true at success.
     */
    long AddStepForward(int stepNumber);


    /**
     * @brief GaitSP::SaveSpaceTrajectories: Will save existing trajectories in the class (all the waypoints) in two csv files,
     * one for each foot.
     * @param fileLeftFoot: File (std::ofstream) for saving left foot trajectory.
     * @param fileRightFoot: File (std::ofstream) for saving right foot trajectory.
     * @return true on success.
     */
    long SaveSpaceTrajectories(std::ofstream &fileRightFoot, std::ofstream &fileLeftFoot);


    /**
     * @brief spGait::SetStepParameters = Set the step parameters for the gait functions.
     * @param swingFootDistance = The distance the floating foot will move forward on every step.
     * @param swingFootElevation = The distance the floating foot will raise from ground on every step.
     * @return
     */
    long SetKickParameters(double swingFootDistance, double swingFootElevation);
    long SetHipParameters(double new_hipSideshift, double new_hipSquat, double new_hipSpeed);

    long SetDefaultSpeeds(double velocity, double rotspeed);


    long GetTrajectories(tra::SpaceTrajectory& getRightFoot, tra::SpaceTrajectory& getLeftFoot);

    long AfterStep();
    long GetSmoothTrajectories(tra::SpaceTrajectory &getRightFoot, tra::SpaceTrajectory &getLeftFoot, double xyz_accel, double rot_accel);
protected:
    //parameters in meters, seconds

    //kick foot parameters
    double kickElevation; //meters
    double kickDistance; //meters
    bool startOnRightFootSupport;

    //support foot parameters
    double hipSideshift; //meters. Lateral hip movement for one leg support.
    double hipSquat; //meters. Hip lowering for step ik reachability.
    double hipSpeed; //velocity of hip movements.
    double legHeight; //meters. Heigth of one leg.
    //double hipDistance; //meters. distance from root to hip joint.

    //step parameters
    double t;
    int stepPhase;  //from 1 to .. step phases
    long stepTotalPhases; //total number of phases in a step

    //initial foot poses
    kin::Pose initialPoseRightFoot,initialPoseLeftFoot;

    //trajectories based on root
    tra::SpaceTrajectory trajRightFoot, trajLeftFoot;

    //private functions
    virtual bool HalfStepForwardRS()=0;
    virtual bool HalfStepForwardLS()=0;

};

}

#endif // GAIT_H
