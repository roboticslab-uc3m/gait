#ifndef OSCILLATOR_HPP
#define OSCILLATOR_HPP

#include <vector>

class Oscillator
{
public:
    Oscillator();
    Oscillator(double periodT, double positiveAmplitude, double negativeAmplitude);


private:
    double T; //period in seconds
    double Ts; //time between two positions
    unsigned int samples; //number of samples per period
    std::vector<double> positionProfile;
    std::vector<double> velocityProfile;

    bool Initialization(double newT, double newTs);
};

#endif // OSCILLATOR_HPP
