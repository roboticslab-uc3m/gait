#include "tools.h"



using namespace std;
using namespace teo::kin;
using namespace teo::tra;


//quaternions
Quaternion::Quaternion()
{

}

Quaternion::Quaternion(double new_w, double new_i, double new_j, double new_k)
{
    qw = new_w;
    qi = new_i;
    qj = new_j;
    qk = new_k;

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

bool Quaternion::FromProduct(const Quaternion & q1, const Quaternion & q2)
{

    /*     x =  q1.x * q2.w + q1.y * q2.z - q1.z * q2.y + q1.w * q2.x;
    y = -q1.x * q2.z + q1.y * q2.w + q1.z * q2.x + q1.w * q2.y;
    z =  q1.x * q2.y - q1.y * q2.x + q1.z * q2.w + q1.w * q2.z;
    w = -q1.x * q2.x - q1.y * q2.y - q1.z * q2.z + q1.w * q2.w; */
    qi = (+q1.Qi() * q2.Qw() + q1.Qj() * q2.Qk() - q1.Qk() * q2.Qj() + q1.Qw() * q2.Qi());
    std::cout << "qi assignement: " << qi << ", " << (+q1.Qi() * q2.Qw() + q1.Qj() * q2.Qk() - q1.Qk() * q2.Qj() + q1.Qw() * q2.Qi()) << std::endl;
    qj = -q1.Qi() * q2.Qk() + q1.Qj() * q2.Qw() + q1.Qk() * q2.Qi() + q1.Qw() * q2.Qj();
    qk = +q1.Qi() * q2.Qj() - q1.Qj() * q2.Qi() + q1.Qk() * q2.Qw() + q1.Qw() * q2.Qk();
    qw = -q1.Qi() * q2.Qi() - q1.Qj() * q2.Qj() - q1.Qk() * q2.Qk() + q1.Qw() * q2.Qw();

//    qi =  q1.qi * q2.qw + q1.qj * q2.qk - q1.qk * q2.qj + q1.qw * q2.qi;
//    qj = -q1.qi * q2.qk + q1.qj * q2.qw + q1.qk * q2.qi + q1.qw * q2.qj;
//    qk =  q1.qi * q2.qj - q1.qj * q2.qi + q1.qk * q2.qw + q1.qw * q2.qk;
//    qw = -q1.qi * q2.qi - q1.qj * q2.qj - q1.qk * q2.qk + q1.qw * q2.qw;

    return true;
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

    x=x2-x1;
    y=y2-y1;
    z=z2-z1;

    //Rotation from initial to final trough origin

    //Compute rotation from initialPose to common origin.
    initialPose.GetRotation(ux,uy,uz,angle);
    angle = -angle;    //invert rotation.

    //compose with finalPose rotation from origin
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

bool Pose::GetRotation(double & axis_i, double & axis_j, double & axis_k, double & pose_angle) const
{
    axis_i=ux;
    axis_j=uy;
    axis_k=uz;
    pose_angle=angle;
    return true;
}

bool Pose::GetRotation(std::vector<double> & rotation)
{
    rotation.clear();
    rotation.push_back(ux);
    rotation.push_back(uy);
    rotation.push_back(uz);
    rotation.push_back(angle);
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

long Pose::SetRotation(Quaternion newRotation)
{
    newRotation.ToAxisAngle(ux,uy,uz,angle);
    return 0;
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

bool Pose::ChangePose(Pose variation)
{
    double x2,y2,z2;
    variation.GetPosition(x2,y2,z2);
    ChangePosition(x2,y2,z2);

    double u2x,u2y,u2z,angle2;
    variation.GetRotation(u2x,u2y,u2z,angle2);


    ChangeRotation(u2x,u2y,u2z,angle2);


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
Pose Pose::ExtrinsicTransform(const kin::Pose & pose0)
{
    kin::Pose extrinsic;


    //Extrinsic rotation vector is computed as intrinsic vector rotated.
    kin::Quaternion q0;
    q0.FromAxisAngle(pose0.Ux(),pose0.Uy(),pose0.Uz(),pose0.Angle());

    //Rotation is computed as r'=q0*r*q0^-1
    kin::Quaternion intRot(0,ux,uy,uz),extRot;
    extRot.FromProduct(intRot,q0.Conjugate());
    extRot.FromProduct(q0,extRot);
    //remember that extRot is not a Quaternion, but the unit rotation axis.
    extrinsic.SetRotation(extRot.Qi(),extRot.Qj(),extRot.Qk(),angle);

    //Position is computed as p'=q0*p*q0^-1
    kin::Quaternion intPos(0,x,y,z),extPos;
    extPos.FromProduct(intPos,q0.Conjugate());
    extPos.FromProduct(q0,extPos);

    //remember that extPos is not a Quaternion, but the (x,y,z) vector.
    extrinsic.SetPosition(extPos.Qi(),extPos.Qj(),extPos.Qk());


    return extrinsic;
}


Pose Pose::Inverse()
{
    Pose inverse(*this);

    kin::Rotation directRotation(ux,uy,uz,angle);

    double ix,iy,iz;
    inverse.GetPosition(ix,iy,iz);

    directRotation.RotatePoint(ix,iy,iz);

    return inverse;
}

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
    defaultRotationSpeed = 0.05; //[rad/sec]
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
    next_wp = last_wp+1; // next_wp >= 1 at this point
    next_wpTime = time_totals[next_wp];
    last_wpTime = time_totals[last_wp];

    //recalculate transform between last_wp and next_wp (as a pose) for interpolation
    //and store at segment variable
    segment = Pose(waypoints[last_wp],waypoints[next_wp]);
    segmentIndex = last_wp;

    std::cout << "New trajectory segment : " << last_wp << "->" << next_wp << ", Segment index: " << segmentIndex << std::endl;

    return segmentIndex;

}

bool SpaceTrajectory::ShowData()
{

    std::cout << "vector time_totals";
    for (int i=0;i<time_totals.size();i++)
    {
        std::cout << time_totals[i] << ", ";
    }
    std::cout << std::endl;

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
        dtr = fabs(angle) / defaultRotationSpeed;
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

    waypoints.push_back(newWaypoint);


    //compute velocities and update velocities vector
    Pose velocity;
    segment.PoseFraction(velocity,1/dt);

    std::cout << "velocity: " << velocity.GetX() << "," << velocity.GetY() << "," << velocity.GetZ() << std::endl;
    std::cout << "angular v: " << velocity.Ux() << "," << velocity.Uy() << "," << velocity.Uz() << "," << velocity.Angle() << std::endl;
    std::cout << "segment: " << segment.Ux() << "," << segment.Uy() << "," << segment.Uz() << "," << segment.Angle() << std::endl;

    velocities.push_back(velocity);

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

    //This (if statement) only happens sometimes. At waypoints, or when calling random.
    if( (sampleTime>next_wpTime)|(sampleTime<last_wpTime) )
    {

        if (UpdatePointers(sampleTime)<0) return false;
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


}


bool SpaceTrajectory::GetSampleVelocity(double sampleTime, Pose & samplePoseVelocity)
{

    //This (if statement) only happens sometimes. At waypoints, or when calling random.
    //if sampleTime is outside actual segment
    if( (sampleTime>next_wpTime)|(sampleTime<last_wpTime) )
    {
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
        if (UpdatePointers(sampleTime)<0) return false;
    }


    //This will happen most times when called sequentially.

    samplePoseVelocity= velocities[segmentIndex];

    return true;

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




bool teoRightFootInDH(const teo::kin::Pose & gaitRightFootPose, teo::kin::Pose & teoRightFootPose)
{

//    teoRightFootPose = gaitRightFootPose;

    return true;

}


bool teoLeftFootInDH(const teo::kin::Pose & gaitLeftFootPose, teo::kin::Pose & teoLeftFootPose)
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
    newP.FromProduct(oldP,q.Conjugate());
    newP.FromProduct(q,newP);
    //remember that newP is not a Quaternion, but the new rotated point.

    px=newP.Qi();
    py=newP.Qj();
    pz=newP.Qk();

    return 0;


}



