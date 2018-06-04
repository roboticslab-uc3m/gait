#include "tools.h"



using namespace std;
using namespace roboticslab::kin;
using namespace roboticslab::tra;
using namespace physics;

//quaternions
Quaternion::Quaternion()
{

}

Quaternion::Quaternion(const Quaternion &new_q)
{
    *this=new_q;

}

Quaternion::Quaternion(double new_w, double new_i, double new_j, double new_k)
{
    qw = new_w;
    qi = new_i;
    qj = new_j;
    qk = new_k;

}

Quaternion::Quaternion(const Quaternion & q1, const Quaternion & q2)
{

/*
    qi = (+q1.Qi() * q2.Qw() + q1.Qj() * q2.Qk() - q1.Qk() * q2.Qj() + q1.Qw() * q2.Qi());
    std::cout << "qi assignement: " << qi << ", " << (+q1.Qi() * q2.Qw() + q1.Qj() * q2.Qk() - q1.Qk() * q2.Qj() + q1.Qw() * q2.Qi()) << std::endl;
    qj = -q1.Qi() * q2.Qk() + q1.Qj() * q2.Qw() + q1.Qk() * q2.Qi() + q1.Qw() * q2.Qj();
    std::cout << "qj assignement: " << qj << ", "
      << -q1.Qi() * q2.Qk() + q1.Qj() * q2.Qw() + q1.Qk() * q2.Qi() + q1.Qw() * q2.Qj() << std::endl;
    qk = +q1.Qi() * q2.Qj() - q1.Qj() * q2.Qi() + q1.Qk() * q2.Qw() + q1.Qw() * q2.Qk();
    std::cout << "qk assignement: " << qk << ", "
      << +q1.Qi() * q2.Qj() - q1.Qj() * q2.Qi() + q1.Qk() * q2.Qw() + q1.Qw() * q2.Qk() << std::endl;
    qw = -q1.Qi() * q2.Qi() - q1.Qj() * q2.Qj() - q1.Qk() * q2.Qk() + q1.Qw() * q2.Qw();
    std::cout << "qw assignement: " << qw << ", "
      << -q1.Qi() * q2.Qi() - q1.Qj() * q2.Qj() - q1.Qk() * q2.Qk() + q1.Qw() * q2.Qw() << std::endl;
*/

    qw = -q1.qi * q2.qi - q1.qj * q2.qj - q1.qk * q2.qk + q1.qw * q2.qw;
    qi =  q1.qi * q2.qw + q1.qj * q2.qk - q1.qk * q2.qj + q1.qw * q2.qi;
    qj = -q1.qi * q2.qk + q1.qj * q2.qw + q1.qk * q2.qi + q1.qw * q2.qj;
    qk =  q1.qi * q2.qj - q1.qj * q2.qi + q1.qk * q2.qw + q1.qw * q2.qk;

}

bool Quaternion::FromAxisAngle(const double ux, const double uy, const double uz, double angle)
{

    //get the factors
    cosPart = cos(angle/2);
    sinPart = sin(angle/2);//sqrt(1-cosPart*cosPart);
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



Quaternion Quaternion::operator*(const Quaternion & q2)
{
    Quaternion q1=*this;
    double pw,pi,pj,pk;
    pw = -q1.Qi() * q2.Qi() - q1.Qj() * q2.Qj() - q1.Qk() * q2.Qk() + q1.Qw() * q2.Qw();
    pi = +q1.Qi() * q2.Qw() + q1.Qj() * q2.Qk() - q1.Qk() * q2.Qj() + q1.Qw() * q2.Qi();
    pj = -q1.Qi() * q2.Qk() + q1.Qj() * q2.Qw() + q1.Qk() * q2.Qi() + q1.Qw() * q2.Qj();
    pk = +q1.Qi() * q2.Qj() - q1.Qj() * q2.Qi() + q1.Qk() * q2.Qw() + q1.Qw() * q2.Qk();

    return Quaternion(pw,pi,pj,pk);


}

Quaternion Quaternion::Conjugate()
{
    return Quaternion(qw,-qi,-qj,-qk);
}

double Quaternion::Qw() const
{
    return qw;
}

double Quaternion::Qi() const
{
    return qi;
}

double Quaternion::Qj() const
{
    return qj;
}

double Quaternion::Qk() const
{
    return qk;
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
    uz=1;
    angle=0;
}

Pose::Pose(Pose initialPose, Pose finalPose)
{
    double x1,y1,z1;
    double x2,y2,z2;
    initialPose.GetPosition(x1,y1,z1);
    finalPose.GetPosition(x2,y2,z2);


    ///TODO: fix to relative movement check ¿Fixed?
    x=x2-x1;
    y=y2-y1;
    z=z2-z1;
    //Rotation from initial to origin
    Rotation to0(-initialPose.Ux(),-initialPose.Uy(),-initialPose.Uz(),initialPose.Angle());
    to0.RotatePoint(x,y,z);
    //x=-x;y=-y;z=-z;

    //Compute rotation from initialPose to common origin.
    initialPose.GetRotation(ux,uy,uz,angle);
    angle = -angle;    //invert rotation.

    //compose with finalPose rotation from origin
    double u2x,u2y,u2z,angle2;
    finalPose.GetRotation(u2x,u2y,u2z,angle2);

    ChangeRotation(u2x,u2y,u2z,angle2);

    //now rotation is based on initial pose and translation as well.


}

Pose Pose::ExtrinsicMoveTo(Pose finalPose)
{
    Pose result(finalPose.GetX()-x, finalPose.GetY()-y, finalPose.GetZ()-z);


    //Rotation from  origin to initial
    Rotation from0(ux,uy,uz,angle);

    //Compute rotation from initialPose to common origin.
    //result will be the relative rotation from pose to finalPose
    result.SetRotation(-ux,-uy,-uz,angle);
    result.ChangeRotation(finalPose.Ux(),finalPose.Uy(),finalPose.Uz(),finalPose.Angle());

    double rux,ruy,ruz,rangle;
    result.GetRotation(rux,ruy,ruz,rangle);
    //Change origin for rotation.
    //It is achieved by rotating the relative vector of rotation
    from0.RotatePoint(rux,ruy,ruz);
    //now rotation is based on initial Origin and translation as well.
    result.SetRotation(rux,ruy,ruz,rangle);

    return result;

}

Pose::Pose(Pose initialPose, Pose finalPose, double factor)
{
    PoseInterpolation(initialPose, finalPose, factor);
}

long Pose::PoseInterpolation(Pose initialPose, Pose finalPose, double factor)
{
    double x1,y1,z1;
    double x2,y2,z2;
    initialPose.GetPosition(x1,y1,z1);
    finalPose.GetPosition(x2,y2,z2);

    x=x1+((x2-x1)*factor);
    y=y1+((y2-y1)*factor);
    z=z1+((z2-z1)*factor);

    return 0;

}

long Pose::PoseFraction(Pose & fraction, double factor)
{
    fraction.SetPosition(x*factor,y*factor,z*factor);
    fraction.SetRotation(ux,uy,uz,angle*factor);

    return 0;

}

long Pose::GetPosition(double &pose_x, double &pose_y, double &pose_z)
{
    pose_x=x;
    pose_y=y;
    pose_z=z;
    return 0;
}

double Pose::GetX() const
{
    return x;
}

double Pose::GetY() const
{
    return y;
}

double Pose::GetZ() const
{
    return z;
}

double Pose::Angle() const
{
    return angle;
}

long Pose::GetRotation(double & axis_i, double & axis_j, double & axis_k, double & pose_angle) const
{
    axis_i=ux;
    axis_j=uy;
    axis_k=uz;
    pose_angle=angle;
    return 0;
}

long Pose::GetRotation(std::vector<double> & rotation)
{
    rotation.clear();
    rotation.push_back(ux);
    rotation.push_back(uy);
    rotation.push_back(uz);
    rotation.push_back(angle);
    return 0;
}

long Pose::GetPoseMatrix(std::vector<double> & xyzRotationMatrix)
{
    if (xyzRotationMatrix.size()<12)
    {
        std::cerr << "Expected 12 value vector. Resizing" << std::endl;
        xyzRotationMatrix.resize(12);
    }
    xyzRotationMatrix[0]=x;
    xyzRotationMatrix[1]=y;
    xyzRotationMatrix[2]=z;

    std::vector<double> rot(9);
    GetRotationMatrix(rot);

    for (int i=0; i<rot.size(); i++)
    {
        xyzRotationMatrix[i+3]=rot[i];
    }

    return 0;
}

long Pose::GetRotationMatrix(std::vector<double> &rotation)
{
    double c=cos(angle);
    double s=sin(angle);
    double t=1-c;

    rotation.clear();
    rotation.resize(9);

    rotation[0] = t*ux*ux + c;
    rotation[1] = t*ux*uy - uz*s;
    rotation[2] = t*ux*uz + uy*s;
    rotation[3] = t*ux*uy + uz*s;
    rotation[4] = t*uy*uy + c;
    rotation[5] = t*uy*uz - ux*s;
    rotation[6] = t*ux*uz - uy*s;
    rotation[7] = t*uy*z + ux*s;
    rotation[8] = t*uz*uz + c;



}

long Pose::GetPose(std::vector<double> &pose)
{
    pose[0] = x;
    pose[1] = y;
    pose[2] = z;
    pose[3] = ux;
    pose[4] = uy;
    pose[5] = uz;
    pose[6] = angle;

    return 0;

}




long Pose::SetRotation(double axis_i, double axis_j, double axis_k, double pose_angle)
{
    ux=axis_i;
    uy=axis_j;
    uz=axis_k;
    angle=pose_angle;


    return 0;
}

long Pose::SetRotation(Quaternion newRotation)
{
    newRotation.ToAxisAngle(ux,uy,uz,angle);
    return 0;
}

bool Pose::ChangeRotationAngle(double angle2)
{
    angle=angle2;
}


bool Pose::ChangeRotation(double u2x, double u2y, double u2z, double angle2)
{
    //double ux1,uy1,uz1,angle1;
    /*  double normal = sqrt( u2xi*u2xi + u2yi*u2yi + u2zi*u2zi );

    double u2x=u2xi/normal;
    double u2y=u2yi/normal;
    double u2z=u2zi/normal;*/

    double angle1=angle;
    double u1x=ux;
    double u1y=uy;
    double u1z=uz;

    double c1=cos(angle1/2);
    double s1=sin(angle1/2);
    double c2=cos(angle2/2);
    double s2=sin(angle2/2);

    double c = -s1*s2*( u1x*u2x + u1y*u2y + u1z*u2z )+c1*c2;
    //std::cout << "c: " << c << std::endl;

    if (c>0.999999 | c<-0.999999)
    {
        //Angle is 0 -> no rotation
        angle=ux=uy=uz=0;
        //uz=1;
        return true;
    }


    //sin(theta/2) can always be positive?

    double s=sqrt(1-c*c);


    ux = (s1*s2) * ( +u1y*u2z - u1z*u2y ) + s1*u1x*c2 + s2*c1*u2x;
    uy = (s1*s2) * ( -u1x*u2z + u1z*u2x ) + s1*u1y*c2 + s2*c1*u2y;
    uz = (s1*s2) * ( +u1x*u2y - u1y*u2x ) + s1*u1z*c2 + s2*c1*u2z;

    angle = 2 * acos(c);

    ux=ux/s;
    uy=uy/s;
    uz=uz/s;

    return true;


}

long Pose::ChangePose(Pose variation)
{
    double x2,y2,z2;
    variation.GetPosition(x2,y2,z2);
    ChangePosition(x2,y2,z2);

    double u2x,u2y,u2z,angle2;
    variation.GetRotation(u2x,u2y,u2z,angle2);


    ChangeRotation(u2x,u2y,u2z,angle2);
    return 0;


}

double Pose::Ux() const
{
    return ux;
}

double Pose::Uy() const
{
    return uy;
}

double Pose::Uz() const
{
    return uz;
}

long Pose::Print(std::string varName)
{
    std::cout << varName << " position: (" << x << ", " << y << ", " << z <<  ")" << std::endl;
    std::cout << varName << " rotation: (" << Ux() << ", " << Uy() << ", " << Uz() << ", " << Angle()*180/M_PI <<  ")" << std::endl;

    return 0;

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

long Pose::CircularMotion(double xaxis, double yaxis, double zaxis, double ang)
{
    kin::Rotation cm(xaxis,yaxis, zaxis, ang);
    cm.RotatePoint(x,y,z);

    ChangeRotation(xaxis,yaxis, zaxis, ang);

    return 0;
}

/*
Pose Pose::TransformTo(Pose anotherPose)
{
    Pose transform;
    return transform;
}
*/

///TODO: Name is not good, can lead to mistake. Find a better name.
/// Or find a better implementation and name.
Pose Pose::WatchFromOriginOf(const kin::Pose & pose0)
{
    kin::Pose extrinsic;


    //Extrinsic rotation vector is computed as intrinsic vector rotated.
    kin::Quaternion q0;
    q0.FromAxisAngle(pose0.Ux(),pose0.Uy(),pose0.Uz(),pose0.Angle());

    //Rotation is computed as r'=q0*r*q0^-1
    kin::Quaternion intRot(0,ux,uy,uz);
    kin::Quaternion extRot;
    extRot = q0*intRot*q0.Conjugate();
    //extRot=q0*extRot;
    //remember that extRot is not a Quaternion, but the unit rotation axis.
    extrinsic.SetRotation(extRot.Qi(),extRot.Qj(),extRot.Qk(),angle);

    //Position is computed as p'=q0*p*q0^-1
    kin::Quaternion intPos(0,x,y,z),extPos;
    extPos= q0*intPos*q0.Conjugate();

    //remember that extPos is not a Quaternion, but the (x,y,z) vector.
    extrinsic.SetPosition(extPos.Qi(),extPos.Qj(),extPos.Qk());


    return extrinsic;
}


Pose Pose::Inverse()
{
    Pose inverse(*this);

    kin::Rotation invRotation(-ux,-uy,-uz,angle);

    inverse.SetRotation(-ux,-uy,-uz,angle);

    double ix,iy,iz;
    inverse.GetPosition(ix,iy,iz);

    invRotation.RotatePoint(ix,iy,iz);
    inverse.SetPosition(-ix,-iy,-iz);


    return inverse;
}

//link definitions



long LinkRotZ::changePose(double dof)
{
    end.SetRotation(0,0,1,dof);
    return 0;

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

    TrajectoryInit();
    SetInitialWaypoint(Pose(0,0,0)); //Undefined trajectories start at origin


}

SpaceTrajectory::SpaceTrajectory(kin::Pose initialWaypoint)
{
    TrajectoryInit();
    SetInitialWaypoint(initialWaypoint); //Defined trajectories start at initialWaypoint

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

bool SpaceTrajectory::TrajectoryInit()
{
    defaultVelocity = 0.1; //[m/s]
    defaultRotationSpeed = 0.1; //[rad/sec]
    next_wp = 0;
    last_wp = 0;
    next_wpTime = 0;
    last_wpTime = 0;

    return true;
}

int SpaceTrajectory::UpdatePointers(double atTime)
{
    /*time_actual = lower_bound (time_totals.begin(),time_totals.end(),forTime);
    //std::cout << "time_totals;" << time_totals[*time_actual-1] << "-" << time_totals[*time_actual] << ",atTime:" << forTime;

    if (time_actual == time_totals.end())
    {
        std::cout << "No Trajectory defined for that time" << std::endl;
        return -1;
    }*/


    last_wp = FindValueIndex(time_totals, atTime);
    //std::cout << "next_wp " << next_wp << ", ";

    if (last_wp < 0)
    {
        std::cout << "Error: Check if time is positive and waypoints are defined" << std::endl;
        return -1;
    }

    //if no errors, store index values and times.
    next_wp = last_wp+1; // next_wp >= 1 from here
    next_wpTime = time_totals[next_wp];
    last_wpTime = time_totals[last_wp];

    //recalculate transform between last_wp and next_wp (as a pose) for interpolation
    //and store at segment variable
    segment = waypoints[last_wp].ExtrinsicMoveTo(waypoints[next_wp]);//Pose(waypoints[last_wp],waypoints[next_wp]);
    segmentIndex = last_wp;

    std::cout << "New trajectory segment : " << last_wp << "->" << next_wp << ", Segment index: " << segmentIndex << std::endl;

    return segmentIndex;

}

long SpaceTrajectory::ShowData()
{

    std::cout << "vector time_totals";
    for (int i=0;i<time_totals.size();i++)
    {
        std::cout << time_totals[i] << std::endl;
        waypoints[i].Print("");
    }

    std::cout << std::endl;
    return 0;

}

bool SpaceTrajectory::AddTimedWaypoint(double &dt,const Pose& newWaypoint)
{

    //Compute segment and update segments vector
    Pose segment(waypoints.back(), newWaypoint);
    segments.push_back(segment);

    if (dt == 0)
    {


        double dx,dy,dz;
        double dtp,dtr;
        double angle = segment.Angle();

        segment.GetPosition(dx,dy,dz);

        dtp = sqrt( dx*dx + dy*dy + dz*dz ) / defaultVelocity;
        dtr = fabs(angle) / defaultRotationSpeed;   //rotation time variaton
        dt= max(dtp,dtr);
        std::cout << "dtp: " << dtp << " , dtr: " << dtr << std::endl;
        std::cout << "Warning! Adding waypoint [" << waypoints.size()-1 << "] with default velocities. dt = " << dt << std::endl;


    }
    time_deltas.push_back(dt);
    time_totals.push_back(time_totals.back()+dt);

    //Pose lastWaypoint=waypoints.back();
    //std::cout << "lastWaypoint: " << lastWaypoint.GetX() << "," << lastWaypoint.GetY() << "," << lastWaypoint.GetZ() << std::endl;
    //std::cout << "waypoint: " << newWaypoint.GetX() << "," << newWaypoint.GetY() << "," << newWaypoint.GetZ() << std::endl;
    //std::cout << "segment: " << segment.GetX() << "," << segment.GetY() << "," << segment.GetZ() << std::endl;



    //compute velocities and update velocities vector
    Pose velocity;
    segment.PoseFraction(velocity,1/dt);

    std::cout << "velocity: " << velocity.GetX() << "," << velocity.GetY() << "," << velocity.GetZ() << std::endl;
    std::cout << "angular v: " << velocity.Ux() << "," << velocity.Uy() << "," << velocity.Uz() << "," << velocity.Angle() << std::endl;
    std::cout << "segment: " << segment.Ux() << "," << segment.Uy() << "," << segment.Uz() << "," << segment.Angle() << std::endl;

    velocitiesRel.push_back(velocity);

    Pose segmentAbs = waypoints.back().ExtrinsicMoveTo(newWaypoint);
    Pose velocityAbs;
    segmentAbs.PoseFraction(velocityAbs,1/dt);
    velocitiesAbs.push_back(velocityAbs);

    waypoints.push_back(newWaypoint);

    return true;
}

double SpaceTrajectory::AddWaypoint(const Pose &waypoint)
{


    //get the time based on default velocity
    /*   Pose lastwp;

    double dx,dy,dz,dangle, dt;

    GetLastWaypoint(lastwp);

    Pose segment(lastwp, waypoint);


    dx = waypoint.GetX()-lastwp.GetX();
    dy = waypoint.GetY()-lastwp.GetY();
    dz = waypoint.GetZ()-lastwp.GetZ();
    dangle = segment.GetAngle();

    std::cout << "Warning! Adding waypoint with default velocity" << std::endl;

    double dx,dy,dz;
    double dtp,dtr;
    double angle = segment.GetAngle();

    segment.GetPosition(dx,dy,dz);

    dtp = sqrt( dx*dx + dy*dy + dz*dz ) / defaultVelocity;
    dtr = angle / defaultRotationSpeed;
    dt= max(dtp,dtr);*/
    double dt=0;
    AddTimedWaypoint(dt, waypoint);

    return dt;
}

double SpaceTrajectory::move(double dx, double dy, double dz)
{

    double dt;


    //kin::Pose actual;
    kin::Pose desired;

    //get actual pose
    GetLastWaypoint(desired);


    //desired=actual;
    desired.ChangePosition(dx,dy,dz);

    dt=AddWaypoint(desired);

    return dt;
}

double SpaceTrajectory::moveBeginSmooth(double dx, double dy, double dz, long sfactor)
{



    double dfactor=0;
    double dt;
    //long sfactor=10;

    kin::Pose actual;
    kin::Pose desired;

    //get actual pose
    GetLastWaypoint(actual);

//    dtp = sqrt( dx*dx + dy*dy + dz*dz ) / defaultVelocity;
//    dtr = fabs(angle) / defaultRotationSpeed;
//    dt= max(dtp,dtr);


    for (int i=0;i<sfactor;i++)
    {
        dfactor +=1/i;
    }
    for (int i=sfactor;i>=0;i--)
    {
        desired.ChangePosition(dx/(i*dfactor),dy/(i*dfactor),dz/(i*dfactor));
        dt+=AddWaypoint(desired);
    }

    //desired=actual;

    //dt=AddWaypoint(desired);

    return dt;
}

double SpaceTrajectory::getDefaultRotationSpeed() const
{
    return defaultRotationSpeed;
}


double SpaceTrajectory::moveTimed(double dx, double dy, double dz, double dt)
{


    //kin::Pose actual;
    kin::Pose desired;

    //get actual pose
    GetLastWaypoint(desired);


    //desired=actual;
    desired.ChangePosition(dx,dy,dz);

    AddTimedWaypoint(dt, desired);

    return dt;
}

double SpaceTrajectory::wait(double dt)
{


    //kin::Pose actual;
    kin::Pose desired;

    //get actual pose
    GetLastWaypoint(desired);


    //desired=actual;
    desired.ChangePosition(0,0,0);

    AddTimedWaypoint(dt, desired);

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

double SpaceTrajectory::GetTotalDuration()
{
    return time_totals.back();
}

void SpaceTrajectory::SetDefaultSpeeds(double vel, double rot)
{
    defaultVelocity = vel;
    defaultRotationSpeed = rot;
}

/**
 * @brief SpaceTrajectory::GetSample: Get the position and orientation for a specific time in a trajectory.
 * @param sampleTime(i): Time for the requested trajectory.
 * @param samplePose(o): Pose with position and orientation output.
 * @return
 */
long SpaceTrajectory::GetSample(double sampleTime, Pose & samplePose)
{

    //This (if statement) only happens sometimes. At waypoints, or when calling random.
    if( (sampleTime>next_wpTime)|(sampleTime<last_wpTime) )
    {
        errorCode = UpdatePointers(sampleTime);

        if (errorCode<0) return errorCode;
        /*        time_actual = lower_bound (time_totals.begin(),time_totals.end(),sampleTime);
        if (time_actual == time_totals.end())
        {
            std::cout << "No Trajectory defined for that time" << std::endl;
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
*/

    }

    //This will happen most times when called sequentially.
    double wpRatio = (sampleTime-last_wpTime)/(next_wpTime-last_wpTime);

    segment.PoseFraction(trajPointer, wpRatio);
    //wpRatio = NextWaypointRate(sampleTime);

    //The sample is the concatenation of last waypoint and segment poses.

    samplePose = waypoints[last_wp];
    samplePose.ChangePose(trajPointer);
    //samplePose.PoseInterpolation(waypoints[last_wp], waypoints[next_wp], wpRatio);

    return 0;

}


long SpaceTrajectory::GetSampleVelocity(double sampleTime, Pose & samplePoseVelocity)
{

    //This (if statement) only happens sometimes. At waypoints, or when calling random.
    //if sampleTime is outside actual segment
    if( (sampleTime>next_wpTime)|(sampleTime<last_wpTime) )
    {
        errorCode = UpdatePointers(sampleTime);
        if (errorCode<0) return errorCode;

        /* time_actual = lower_bound (time_totals.begin(),time_totals.end(),sampleTime);
        //std::cout << "time_totals;" << time_totals[*time_actual-1] << "-" << time_totals[*time_actual] << ",atTime:" << sampleTime;

        if (time_actual == time_totals.end())
        {
            std::cout << "No Trajectory defined for that time" << std::endl;
            return -1;
        }

        next_wp = *time_actual;
        //std::cout << "next_wp " << next_wp << ", ";

        if (next_wp < 1)
        {
            std::cout << "Error: Check if time is positive and waypoints are defined" << std::endl;
            return -1;
        }

        //if no errors, store index values and times.
        last_wp = next_wp-1; // next_wp >= 1 at this point
        next_wpTime = time_totals[next_wp];
        last_wpTime = time_totals[last_wp];

        //recalculate transform between last_wp and next_wp (as a pose) for interpolation
        //and store at segment variable
        segment = Pose(waypoints[last_wp],waypoints[next_wp]);
        segmentIndex = last_wp;

        std::cout << "New trajectory segment : " << last_wp << "->" << next_wp << ", Segment index: " << segmentIndex << std::endl;*/
    }


    //This will happen most times when called sequentially.

    samplePoseVelocity= velocitiesAbs[segmentIndex];

    return 0;

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

double SpaceTrajectory::GetWaypointTd(int index)
{
    return time_deltas[index];
}

long SpaceTrajectory::GetVelocitiesRel(int index, kin::Pose & velsfromLastWp) const
{
    velsfromLastWp = velocitiesRel[index];
    return 0;
}


bool SpaceTrajectory::GetLastWaypoint(Pose &waypoint)
{
    waypoint = waypoints.back();
    return true;
}

long SpaceTrajectory::SaveToFile(std::ofstream &csvFile)
{
    //Pose wpPose;
    double x,y,z;
    double i,j,k,angle;
    for (int n=0; n<waypoints.size(); n++)
    {
        csvFile << time_totals[i] << ",";
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
    return 0;


}




bool teoRightFootInDH(const roboticslab::kin::Pose & gaitRightFootPose, roboticslab::kin::Pose & teoRightFootPose)
{

    //    teoRightFootPose = gaitRightFootPose;

    return true;

}


bool teoLeftFootInDH(const roboticslab::kin::Pose & gaitLeftFootPose, roboticslab::kin::Pose & teoLeftFootPose)
{
    return true;


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


//get the first data greater than a value from a vector.
int FindValueIndex(std::vector<double> vector, double value)
{
    for (int i=0;i<vector.size();i++)
    {
        //std::cout << "value: " << value << "vector i :" << vector[i];
        if (value<vector[i])
        {
            return (i-1);
        }
    }
    return -1;
}

int UpdateVectorPointer(const std::vector<double> & vector, const double & actual, double & next, double & last )
{


    int vectorSegment;

    vectorSegment = FindValueIndex(vector, actual);
    //std::cout << "next_wp " << next_wp << ", ";

    if (vectorSegment < 0)
    {
        std::cout << "Error: Check if actual value exist and vector have data" << std::endl;
        return -1;
    }

    //if no errors, store index and values.
    last = vector[vectorSegment];
    next = vector[vectorSegment+1];


    //std::cout << "New segment : " << last << "->" << next << ", Segment index: " << vectorSegment << std::endl;

    return vectorSegment;

}


Rotation::Rotation()
{
    //q=kin::Quaternion(1,0,0,0);
    ux=uy=angle=0;
    uz=1;

}

Rotation::Rotation(double new_ux, double new_uy, double new_uz, double new_angle)
{
    ux=new_ux;
    uy=new_uy;
    uz=new_uz;
    angle=new_angle;

}

long Rotation::RotatePoint(double & px, double & py, double & pz)
{

    //rotation vector is computed as intrinsic vector rotated.
    kin::Quaternion q;
    q.FromAxisAngle(ux,uy,uz,angle);

    //Rotation is computed as p'=q0*p*q0^-1
    kin::Quaternion oldP(0,px,py,pz),newP;
    /*newP.FromProduct(oldP,q.Conjugate());
    newP.FromProduct(q,newP);*/
    newP=q*oldP*q.Conjugate();
    //remember that newP is not a Quaternion, but the new rotated point.

    px=newP.Qi();
    py=newP.Qj();
    pz=newP.Qk();

    return 0;//no error


}




long StateVariable::Initialize(std::vector<double> newFormer, std::vector<double> newInitial)
{
    state = newInitial;
    order=state.size();
    former = newFormer;
    return 0;

}

std::vector<double> StateVariable::getFormer() const
{
    return former;
}

std::vector<double> StateVariable::getState() const
{
    return state;
}

StateVariable::StateVariable()
{
    Initialize(std::vector<double>(3,0), std::vector<double>(3,0));

}

StateVariable::StateVariable(std::vector<double> formerState, std::vector<double> initialState)
{
    Initialize(formerState, initialState);

}

StateVariable::StateVariable(double newValue, double D1, double D2)
{

    state = std::vector<double>{newValue,D1,D2};
    order = state.size();
    former.clear();
    former.resize(order);

}


long StateVariable::Update(double newValue, double dt)
{
    //first of all, actual state becomes former state
    former=state;

    //then update values for state
    state[0]=newValue;

    for (uint i=1; i<order; i++)
    {
        state[i]=(state[i-1]-former[i-1])/dt;
    }
    return 0;


}

double StateVariable::GetOrder()
{
    return state.size();

}

double StateVariable::D(uint dOrder)
{
    if (dOrder>order)
    {
        return 0;
    }
    else
    {
        return state[dOrder];
    }

}


long TimedVariable::Initialize(const std::deque<double> & newValues, const std::deque<double> newTimes)
{
    values = newValues;
    times = newTimes;
    return 0;

}

TimedVariable::TimedVariable()
{
    Initialize(std::deque<double>(3,0), std::deque<double>(3,0));

}

double TimedVariable::BackwardFiniteDifference(uint diffOrder)
{
    if (diffOrder > values.size())
    {
        std::cout << "error in derivative order" << std::endl;
        return 0;
    }
    std::vector<double> results;


    for (uint i=0; i<diffOrder; i++)
    {
        std::cout<< "NOT READY!!! TODO:FINISH" << std::endl;
    }
    return 0;


}

double TimedVariable::D1()
{
    return (values[0]-values[1])/times[0];

}


double TimedVariable::D2()
{
    double v1=(values[0]-values[1])/times[0];
    double v2=(values[1]-values[2])/times[1];
    return (v1-v2)/times[0];

}

