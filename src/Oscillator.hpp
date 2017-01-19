#ifndef OSCILLATOR_HPP
#define OSCILLATOR_HPP

#include <vector>
#include <math.h>

#include "tools.h"

class Oscillator
{
public:
    Oscillator();
    Oscillator(double periodT, double Amplitude1, double A2);

    double GetVelocity(double actualPos);


private:
    double T; //period in seconds
    double Ts; //time between two positions
    unsigned int samples; //number of samples per period
    std::vector<double> positionProfile;
    std::vector<double> velocityProfile;

    int profileSegment;
    double lastPos, nextPos, posRatio;

    bool Initialization(double newT, double newTs);
};

#endif // OSCILLATOR_HPP
