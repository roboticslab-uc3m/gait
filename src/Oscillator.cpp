#include "Oscillator.hpp"

Oscillator::Oscillator()
{

    Initialization(1,1/10);
}

Oscillator::Oscillator(double periodT, double positiveAmplitude, double negativeAmplitude)
{

    Initialization(periodT, periodT/10);
    double amp;
    //double timeRate = positiveAmplitude/(positiveAmplitude - negativeAmplitude);
    //double pT= timeRate*T;
    //double pSamples = (int) (1/timeRate);
    //double aN= negativeAmplitude/(positiveAmplitude - negativeAmplitude);
    unsigned int pSamples = (int) samples * negativeAmplitude/(positiveAmplitude - negativeAmplitude);
    //zero position crossing at
    for (int i=0; i<pSamples; i++)
    {

    }


}

bool Oscillator::Initialization(double newT, double newTs)
{
    T=newT;
    Ts=newTs;
    samples = T/Ts;

}
