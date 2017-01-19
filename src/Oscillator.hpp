#ifndef OSCILLATOR_HPP
#define OSCILLATOR_HPP

#include <vector>

class Oscillator
{
public:
    Oscillator();
    Oscillator(double periodT, double center, double positiveAmplitude, double negativeAmplitude);


private:
    double T; //period
    double Ts; //time between two positions
    std::vector<double> positionProfile;
    std::vector<double> velocityProfile;
};

#endif // OSCILLATOR_HPP
