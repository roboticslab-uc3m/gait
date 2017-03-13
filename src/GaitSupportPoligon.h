#include "Gait.h"
#include "tools.h"
#include <stdio.h>

namespace teo
{

/**
 * @brief The GaitSupportPoligon class encapsulates cartesian foot trajectory generation for biped gaits.
 *
 * How to use:
 * 1- Instance by calling the constructor wiht feet Poses as parameters.
 * 2- Set the gait parameters with SetStepParameters function.
 * 3- Call the Add functions to add steps to trajectory.
 *
 * The object main properties are:
 * 1- SpaceTrajectory: All the steps will be stored in a cartesian trajectory object. Retrieve data with get functions.
 *
 */

class GaitSupportPoligon : public Gait
{
public:
    GaitSupportPoligon(kin::Pose initialRightFoot, kin::Pose initialLeftFoot) :
        Gait(initialRightFoot,initialLeftFoot){}


private:

    //private functions
    bool HalfStepForwardRS();
    bool HalfStepForwardLS();

};


}
