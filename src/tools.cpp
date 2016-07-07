#include "tools.h"

using namespace std;
using namespace teo::kin;
using namespace teo::tra;

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
    y=x1-y1;
    z=z2-z1;

}

Pose::Pose(Pose initialPose, Pose finalPose, double fraction)
{
    double x1,y1,z1;
    double x2,y2,z2;
    initialPose.GetPosition(x1,y1,z1);
    finalPose.GetPosition(x2,y2,z2);

    x=x2-x1;
    y=x1-y1;
    z=z2-z1;

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
    double sinAngle = sin(angle/2);

    q1 = cos(angle/2);
    q2 = ux*sinAngle;
    q3 = uy*sinAngle;
    q4 = uz*sinAngle;


    return true;
}

bool Pose::PoseDifference( Pose otherPose, Pose & difference)
{
    double x2,y2,z2;
    otherPose.GetPosition(x2,y2,z2);
    difference.SetPosition(x2-x, y2-y, z2-z);

    return true;
}

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
    time_deltas.push_back(dt);
    time_totals.push_back(time_totals.back()+dt);
    /*
    error = waypoints.insert(std::pair<double,Pose>(t,waypoint));
    if (error.second == false)
    {
        std::cout << "Trying to insert existing values" << std::endl;
        return -1;
    }*/
    return 0;
}

bool SpaceTrajectory::AddWaypoint(Pose waypoint)
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

    AddTimedWaypoint(dt, waypoint);

    return 0;
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
    Pose *nextPose,*lastPose;

    double wpRatio;
    double xlast,ylast,zlast;
    double xnext,ynext,znext;

    wpRatio = NextWaypointRate(sampleTime);

    samplePose = waypoints[last_wp];

    nextPose = &waypoints[next_wp];
    lastPose = &waypoints[last_wp];

    Pose halfWayPose(*lastPose, *nextPose, wpRatio);
    samplePose = halfWayPose;
    //lastPose->PoseDifference(nextPose, halfWayPose);

    //samplePose.ChangePosition(nextPose.GetX()-lastPose.GetX(),nextPose.GetY()-lastPose.GetY(),nextPose.GetZ()-lastPose.GetZ());

}

double SpaceTrajectory::NextWaypointRate(double atTime)
{

    if( (atTime>next_wpTime)|(atTime<last_wpTime) )
    {
    //this only happens sometimes. When track reaches a waypoint, or when calling random.

        time_actual = lower_bound (time_totals.begin(),time_totals.end(),atTime);
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

    }

    //just this will happen most times if called sequentially.
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
