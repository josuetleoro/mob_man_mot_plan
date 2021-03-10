#include "mars_mot_plan/traj_plan/Trajectory.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include <iostream>
using namespace std;
using namespace Eigen;

Trajectory::Trajectory(double ti, double tf)
{
    this->ti = ti;
    this->tf = tf;
}

void Trajectory::validateTime(double t)
{
    if (t < ti || t > tf)
    {
        throw "requested time outside of trajectory period";
    }
}

double Trajectory::getTi()
{
    return ti;
}

double Trajectory::getTf()
{
    return tf;
}

double Trajectory::getPos(char coord, double t)
{
    validateTime(t);
    Vector3d pos = trajPos(t);
    switch (coord)
    {
    case 'x':
        return pos(0);
    case 'y':
        return pos(1);
    case 'z':
        return pos(2);
    default:
        string errStr = "The required position coordinate ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

double Trajectory::getLinVel(char coord, double t)
{
    validateTime(t);
    Vector3d vel = trajLinVel(t);
    switch (coord)
    {
    case 'x':
        return vel(0);
    case 'y':
        return vel(1);
    case 'z':
        return vel(2);
    default:
        string errStr = "The required velocity ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

double Trajectory::getOrientPart(char part, double t)
{
    validateTime(t);
    Quat Q = trajOrient(t);
    double quatPart;
    switch(part)
    {
    case 'w':
        quatPart = Q.w;
        break;
    case 'x':
        quatPart = Q.x;
        break;
    case 'y':
        quatPart = Q.y;
        break;
    case 'z':
        quatPart = Q.z;
        break;
    default:
        string errStr = "Quaternion does not contain element ";
        errStr.append(1, part);
        errStr.append(".");
        throw errStr;
    }
    return quatPart;
}

Quat Trajectory::getOrient(double t)
{
    return trajOrient(t);
}

double Trajectory::getAngVel(char coord, double t)
{
    validateTime(t);
    Vector3d w = trajAngVel(t);
    switch (coord)
    {
    case 'x':
        return w(0);
    case 'y':
        return w(1);
    case 'z':
        return w(2);
    default:
        string errStr = "The required position coordinate ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

Pose Trajectory::getPose(double t)
{
    validateTime(t);
    Vector3d pos = trajPos(t);
    Quat orient = trajOrient(t);    
    return Pose(pos, orient);
}

VectorXd Trajectory::getPoseVec(double t)
{
    return getPose(t).vecRep();
}

VectorXd Trajectory::getVel(double t)
{
    validateTime(t);
    VectorXd vel(6);
    vel.head(3) = trajLinVel(t);
    vel.tail(3) = trajAngVel(t);
    return vel;
}

