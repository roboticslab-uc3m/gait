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
    defaultVelocity = 0.5;
    total_t.push_back(0);

}

bool SpaceTrajectory::AddTimedWaypoint(double dt, Pose waypoint)
{

    waypoints.push_back(waypoint);
    delta_t.push_back(dt);
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

bool SpaceTrajectory::GetWaypoint(int index, Pose& getWaypoint)
{
    getWaypoint=waypoints[index];
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
