#ifndef ZEROMOMENTPOINT_HPP
#define ZEROMOMENTPOINT_HPP


#include "tools.h"

class ZeroMomentPoint
{
public:
    long Initialize();
    ZeroMomentPoint();

    long SolveXm(const double &xZMP, const physics::TimedVariable &zx, physics::TimedVariable &xm);

private:


};

#endif // ZEROMOMENTPOINT_HPP
