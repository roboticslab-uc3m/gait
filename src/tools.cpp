#include "tools.h"

using namespace std;
using namespace teo::kin;
using namespace teo::tra;


//quaternions
Quaternion::Quaternion()
{

}

bool Quaternion::FromAxisAngle(double ux, double uy, double uz, double angle)
{

    //get the factors
    cosPart = cos(angle/2);
    sinPart = sqrt(1-cosPart*cosPart);
    //get the indices
    qw = cosPart;
    qi = ux*sinPart;
    qj = uy*sinPart;
    qk = uz*sinPart;
    return true;
}

bool Quaternion::ToAxisAngle(double & ux, double & uy, double & uz, double & angle)
{
    if (qw==1)
    {
        //Angle is 0, so i,j,k are 0
        ux=uy=uz=0;
        return true;
    }
    sinPart = sqrt(1-qw*qw);
    angle = 2 * acos(qw);
    ux = qi / sinPart;
    uy = qj / sinPart;
    uz = qk / sinPart;
    return true;

}

bool Quaternion::FromProduct(const Quaternion & q1, const Quaternion & q2)
{
    qi =  q1.qi * q2.qw + q1.qj * q2.qk - q1.qk * q2.qj + q1.qw * q2.qi;
    qj = -q1.qi * q2.qk + q1.qj * q2.qw + q1.qk * q2.qi + q1.qw * q2.qj;
    qk =  q1.qi * q2.qj - q1.qj * q2.qi + q1.qk * q2.qw + q1.qw * q2.qk;
    qw = -q1.qi * q2.qi - q1.qj * q2.qj - q1.qk * q2.qk + q1.qw * q2.qw;

}

//Pose definitions

Pose::Pose()
{

}

Pose::Pose(double x0, double y0, double z0)
{
    x=x0;
    y=y0;
    z=z0;
    ux=0;
    uy=0;
    uz=0;
    angle=0;
}

Pose::Pose(Pose initialPose, Pose finalPose)
{
    double x1,y1,z1;
    double x2,y2,z2;
    initialPose.GetPosition(x1,y1,z1);
    finalPose.GetPosition(x2,y2,z2);

    x=x2-x1;
    y=y2-y1;
    z=z2-z1;

    //Rotation from initial to final trough origin
    initialPose.GetRotation(ux,uy,uz,angle);
    //invert rotation so rotation is the same as origin
    angle = -angle;
    //compose with final rotation from origin
    double u2x,u2y,u2z,angle2;
    finalPose.GetRotation(u2x,u2y,u2z,angle2);

    ChangeRotation(u2x,u2y,u2z,angle2);

    //now rotation is based on initial pose and translation as well.


}

Pose::Pose(Pose initialPose, Pose finalPose, double factor)
{
    PoseInterpolation(initialPose, finalPose, factor);
}

bool Pose::PoseInterpolation(Pose initialPose, Pose finalPose, double factor)
{
    double x1,y1,z1;
    double x2,y2,z2;
    initialPose.GetPosition(x1,y1,z1);
    finalPose.GetPosition(x2,y2,z2);

    x=x1+((x2-x1)*factor);
    y=y1+((y2-y1)*factor);
    z=z1+((z2-z1)*factor);

}

bool Pose::PoseFraction(Pose & fraction, double factor)
{
    fraction.SetPosition(x*factor,y*factor,z*factor);
    fraction.SetRotation(ux,uy,uz,angle*factor);
}

bool Pose::GetPosition(double &pose_x, double &pose_y, double &pose_z)
{
    pose_x=x;
    pose_y=y;
    pose_z=z;
    return true;
}

double Pose::GetX()
{
    return x;
}

double Pose::GetY()
{
    return y;
}

double Pose::GetZ()
{
    return z;
}

double Pose::GetAngle()
{
    return angle;
}

bool Pose::GetRotation(double & axis_i, double & axis_j, double & axis_k, double & pose_angle)
{
    axis_i=ux;
    axis_j=uy;
    axis_k=uz;
    pose_angle=angle;
    return true;
}

bool Pose::SetRotation(double axis_i, double axis_j, double axis_k, double pose_angle)
{
    ux=axis_i;
    uy=axis_j;
    uz=axis_k;
    angle=pose_angle;


    return true;
}

bool Pose::ChangeRotation(double u2x, double u2y, double u2z, double angle2)
{
    //double ux1,uy1,uz1,angle1;
    double angle1=angle;
    double u1x=ux;
    double u1y=uy;
    double u1z=uz;
    double c1=cos(angle1/2);
    double s1=sin(angle1/2);
    double c2=cos(angle2/2);
    double s2=sin(angle2/2);

    //WARNING!!! this should be -s1*s2*( u1x*u2x + u1y*u2y + u1z*u2z )+c1*c2; not (+)
    //according to quaternion multiplication formulas
    //TODO: Check why this works and not in the right way!!!!!!!!
    //Maybe something to do with negative angles and cos(-a)=cos(a), sin(-a)=-sin(a)???
    //Done!!! s=sqrt(1-c*c) has + and - solutions, so must keep in mind the angle sign
    double c = -s1*s2*( u1x*u2x + u1y*u2y + u1z*u2z )+c1*c2;

    if (c==1)
    {
        //Angle is 0, so no rotation as result
        angle=ux=uy=uz=0;
        return true;
    }

    //this operation can be a problem (see above) keep in mind the signs.
    double s=sqrt(1-c*c);


    ux = (s1*s2) * ( +u1y*u2z - u1z*u2y ) + s1*u1x*c2 + s2*c1*u2x;
    ux=ux/s;
    uy = (s1*s2) * ( -u1x*u2z + u1z*u2x ) + s1*u1y*c2 + s2*c1*u2y;
    uy=uy/s;
    uz = (s1*s2) * ( +u1x*u2y - u1y*u2x  )+ s1*u1z*c2 + s2*c1*u2z;
    uz=uz/s;

    angle = 2 * acos(c);

    return true;


}

bool Pose::ChangePose(Pose variation)
{
    double x2,y2,z2;
    variation.GetPosition(x2,y2,z2);
    ChangePosition(x2,y2,z2);

    double u2x,u2y,u2z,angle2;
    variation.GetRotation(u2x,u2y,u2z,angle2);


    ChangeRotation(u2x,u2y,u2z,angle2);


}

/*bool Pose::PoseDifference( Pose otherPose, Pose & difference)
{
    double x2,y2,z2;
    otherPose.GetPosition(x2,y2,z2);
    difference.SetPosition(x2-x, y2-y, z2-z);

    return true;
}*/

bool Pose::SetPosition(double new_x, double new_y, double new_z)
{
    x=new_x;
    y=new_y;
    z=new_z;
    return true;
}

bool Pose::ChangePosition(double dx, double dy, double dz)
{
    x+=dx;
    y+=dy;
    z+=dz;
    return true;

}

/*
Pose Pose::TransformTo(Pose anotherPose)
{
    Pose transform;
    return transform;
}
*/


//link definitions



bool LinkRotZ::changePose(double dof)
{
    end.SetRotation(0,0,1,dof);

}

Link::Link()
{
    end = Pose(0,0,0);

}

Link::Link(const Pose &initialPose)
{
    end = initialPose;

}

Pose Link::getCOG() const
{
    return COG;
}

void Link::setCOG(const Pose &value)
{
    COG = value;
}


//SpaceTrajectory definitions

SpaceTrajectory::SpaceTrajectory()
{

    Reset();
    SetInitialWaypoint(Pose(0,0,0)); //Undefined trajectories start at origin


}

SpaceTrajectory::SpaceTrajectory(kin::Pose initialWaypoint)
{
    Reset();
    SetInitialWaypoint(initialWaypoint); //Undefined trajectories start at origin

}

bool SpaceTrajectory::SetInitialWaypoint(kin::Pose initialWaypoint)
{
    if (waypoints.size()==0)
    {

        waypoints.resize(1);
        time_deltas.resize(1);
        time_totals.resize(1);
    }


    waypoints[0]=initialWaypoint;
    time_deltas[0]=0;
    time_totals[0]=0;

    return true;
}

bool SpaceTrajectory::Reset()
{
    defaultVelocity = 0.2;
    next_wp = 0;
    last_wp = 0;
    next_wpTime = 0;
    last_wpTime = 0;

    return true;
}

bool SpaceTrajectory::AddTimedWaypoint(double dt, Pose waypoint)
{

    waypoints.push_back(waypoint);
    if (dt == 0)
    {
        std::cout << "Warning! Adding waypoint with 0 delta time." << std::endl;
    }
    time_deltas.push_back(dt);
    time_totals.push_back(time_totals.back()+dt);
    /*
    error = waypoints.insert(std::pair<double,Pose>(t,waypoint));
    if (error.second == false)
    {
        std::cout << "Trying to insert existing values" << std::endl;
        return -1;
    }*/
    return true;
}

double SpaceTrajectory::AddWaypoint(Pose waypoint)
{


    //get the time based on default velocity
    Pose lastwp;
    double dx,dy,dz, dt;

    GetLastWaypoint(lastwp);

    dx = waypoint.GetX()-lastwp.GetX();
    dy = waypoint.GetY()-lastwp.GetY();
    dz = waypoint.GetZ()-lastwp.GetZ();

    dt = sqrt( dx*dx + dy*dy + dz*dz ) / defaultVelocity;

    //TODO: Calculate rotation angle to limit rotation velocity.
    //TODO: apply dt as max between rotation time and translation time (1 second now).
    dt=max(dt,1.0);

    AddTimedWaypoint(dt, waypoint);

    return dt;
}

int SpaceTrajectory::Size()
{
    return waypoints.size();
}

double SpaceTrajectory::getDefaultVelocity() const
{
    return defaultVelocity;
}

void SpaceTrajectory::setDefaultVelocity(double value)
{
    defaultVelocity = value;
}

bool SpaceTrajectory::GetSample(double sampleTime, Pose & samplePose)
{

    //This "if loop" only happens sometimes. At waypoints, or when calling random.
    if( (sampleTime>next_wpTime)|(sampleTime<last_wpTime) )
    {

        time_actual = lower_bound (time_totals.begin(),time_totals.end(),sampleTime);
        if (time_actual == time_totals.end())
        {
            std::cout << "No Waypoints defined for that time" << std::endl;
            return -1;
        }

        next_wp = *time_actual;
        if (next_wp < 1)
        {
            std::cout << "Error: Check if time is positive and waypoints are defined" << std::endl;
            return -1;
        }

        //if no errors, store index values and times.
        last_wp = next_wp-1; // next_wp > 1 at this point
        next_wpTime = time_totals[next_wp];
        last_wpTime = time_totals[last_wp];

        //recalculate transform between last_wp and next_wp (as a pose) for interpolation
        //and store at segment variable
        segment = Pose(waypoints[last_wp],waypoints[next_wp]);

        std::cout << "New trajectory segment : " << last_wp << "->" << next_wp << std::endl;


    }

    //This will happen most times when called sequentially.
    double wpRatio = (sampleTime-last_wpTime)/(next_wpTime-last_wpTime);

    segment.PoseFraction(trajPointer, wpRatio);
    //wpRatio = NextWaypointRate(sampleTime);

    //The sample is the concatenation of last waypoint and segment poses.

    samplePose = waypoints[last_wp];
    samplePose.ChangePose(trajPointer);
    //samplePose.PoseInterpolation(waypoints[last_wp], waypoints[next_wp], wpRatio);


}

double SpaceTrajectory::NextWaypointRate(double atTime)
{



    return (atTime-last_wpTime)/(next_wpTime-last_wpTime);

}

bool SpaceTrajectory::GetWaypoint(int index, Pose& getWaypoint)
{
    getWaypoint=waypoints[index];
    return true;
}

bool SpaceTrajectory::GetWaypoint(int index, Pose &getWaypoint, double &time_total)
{
    getWaypoint=waypoints[index];
    time_total = time_totals[index];
    return true;
}

bool SpaceTrajectory::GetLastWaypoint(Pose &waypoint)
{
    waypoint = waypoints.back();
    return true;
}

bool SpaceTrajectory::SaveToFile(std::ofstream &csvFile)
{
    //Pose wpPose;
    double x,y,z;
    double i,j,k,angle;
    for (int n=0; n<waypoints.size(); n++)
    {
        //wpPose.GetPosition(x,y,z);
        waypoints[n].GetPosition(x,y,z);
        //wpPose.GetRotation(i,j,k,angle);
        waypoints[n].GetRotation(i,j,k,angle);
        csvFile << x << ",";
        csvFile << y << ",";
        csvFile << z << ",";
        csvFile << i << ",";
        csvFile << j << ",";
        csvFile << k << ",";
        csvFile << angle << std::endl;
    }
}




bool Robot::addLink(const Link& newLink)
{
    links.push_back(newLink);
    return true;
}

Pose Robot::getRobotBase() const
{
    return robotBase;
}

void Robot::setRobotBase(const Pose &value)
{
    robotBase = value;
}

