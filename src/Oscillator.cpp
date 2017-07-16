#include "Oscillator.hpp"

#include <math.h>
#include <iostream>

#include "tools.h"

Oscillator::Oscillator()
{

    Initialization(1,1/10);
}

Oscillator::Oscillator(double t1, double t2, double A1, double A2)
{

    Initialization(t1+t2, (t1+t2)/100);
    double t;
    //double timeRate = A1/(A1 - A2);
    //double T1= timeRate*T;
    //double T2= (1-timeRate)*T;
    int samples1 = (int) (t1/Ts);
    //double samples2 = (int) (samples-samples1);

    //double aN= negativeAmplitude/(positiveAmplitude - negativeAmplitude);
    //unsigned int pSamples = (int) samples * negativeAmplitude/(positiveAmplitude - negativeAmplitude);

    double w1 = M_PI/t1;
    double w2 = M_PI/t2;
    //zero position crossing at
    //compute position and velocity profiles
    for (int i=0; i<samples1; i++)
    {
        t= i*Ts;
        timeProfile.push_back(t);
        positionProfile.push_back(A1*sin(w1*t));
        velocityProfile.push_back(A1*w1*cos(w1*t));
        //std::cout << i << "time : " <<  t << ",pos: " << positionProfile[i] << ",vel: " << velocityProfile[i] << std::endl;
    }
    //negative part
    for (int i=0; i<(samples-samples1); i++)
    {
        t= i*Ts;
        timeProfile.push_back((i+samples1)*Ts);
        positionProfile.push_back(A2*sin(w2*t));
        velocityProfile.push_back(A2*w2*cos(w2*t));
        //std::cout << i+samples1 << "time : " <<  t << ",pos: " << positionProfile[i+samples1] << ",vel: " << velocityProfile[i+samples1] << std::endl;

    }


}

double Oscillator::GetVelocity(double actualTime)
{

    if( (actualTime>nextTime)|(actualTime<lastTime) )
    {
        profileSegment = UpdateVectorPointer(timeProfile,actualTime,nextTime,lastTime);
    }
    tRatio = (actualTime-lastTime)/(nextTime-lastTime);
   // std::cout << "time : " <<  actualTime << ",ratio: " << tRatio << ",vel: " << tRatio*velocityProfile[profileSegment];

    return velocityProfile[profileSegment]+
            tRatio*(velocityProfile[profileSegment+1] - velocityProfile[profileSegment]);

    //this code was deigned for position but not used at the end
    /*
    if( (actualPos>nextPos)|(actualPos<lastPos) )
    {
        profileSegment = UpdateVectorPointer(positionProfile,actualPos,nextPos,lastPos);
    }

    posRatio = (actualPos-lastPos)/(nextPos-lastPos);
    return posRatio*velocityProfile[profileSegment];
*/


}

bool Oscillator::Initialization(double newT, double newTs)
{
    T=newT;
    Ts=newTs;
    samples = T/Ts;
    profileSegment=0;


}
