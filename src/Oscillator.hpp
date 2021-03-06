#ifndef OSCILLATOR_HPP
#define OSCILLATOR_HPP

#include <vector>

class Oscillator
{
public:
    Oscillator();
    Oscillator(double t1, double t2, double A1, double A2);

    double GetVelocity(double actualTime);


private:
    double T; //period in seconds
    double Ts; //time between two positions
    unsigned int samples; //number of samples per period
    std::vector<double> positionProfile;
    std::vector<double> velocityProfile;
    std::vector<double> timeProfile;

    int profileSegment;
    double lastTime, nextTime, tRatio;

    bool Initialization(double newT, double newTs);
};

#endif // OSCILLATOR_HPP
