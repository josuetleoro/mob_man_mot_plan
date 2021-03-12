#include "mars_mot_plan/traj_plan/CircleWavePathTrajectory.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include <iostream>
using namespace std;
using namespace Eigen;

CirclePathTrajectory::CirclePathTrajectory()
{
    ti =0; tf = 0;
    radius = 0; z_height = 0; z_freq = 0;
    vel = 0;
    max_a = 0;
};

// Computes the coefficients for the trajectory
CirclePathTrajectory::CirclePathTrajectory(Pose posei, double ti, double tf, double tb, double radius, double z_height, double z_freq)
{
    this->ti = ti;
    this->tf = tf;
    this->posei = posei;

    // Parameters that define the size of circle path
    this->radius = radius;
    this->z_height = z_height;
    this->z_freq = z_freq;
    
    // Fixed parameters that define the circle path
    sf = 2*M_PI;

    // Parameters obtained from the give times
    vel = sf/(tf-tb);
    max_a = vel/tb;

    cout << "vel: " << vel << endl;
    cout << "max_a: " << max_a << endl;

    // Find the knot point
    this->tb = tb;
    pa_x = max_a/2*tb*tb;

    cout << "pax_a: " << pa_x << endl;
    
    // Orientation is calculated using the position for this path
    Quat quati = posei.orientation;
    Eigen::Quaterniond quat_i_eigen(quati.w, quati.x, quati.y, quati.z);
    Matrix3d R = quat_i_eigen.toRotationMatrix();
    cout << "Ri: " << endl << R << endl;
    normali = R.col(0);
    // cout << "orienti: " << orienti.transpose() << endl;
    cout << "normali: " << normali.transpose() << endl;

    // The orientation direction points to the center of the circle
    target = Vector3d(posei.position(0) - radius, posei.position(1), 0);
    // It is assumed the approach direction points down
    approach = Vector3d(0,0,-1);

    // Find the final pose
    Vector3d posf = trajPos(tf);
    Quat orientf = trajOrient(tf);
    posef = Pose(posf, orientf);
}

void CirclePathTrajectory::validateTime(double t)
{
    if (t < ti || t > tf)
    {
        cout << "Provided time t[" << t << "] out of trajectory time[" << ti << ", " << tf << "]." << endl;
        throw "requested time outside of trajectory period";
    }
}

Quat CirclePathTrajectory::quatDiff(const Quat &q0, const Quat &q1)
{
    Quat diff = q1*q0.conj();
    if (diff.w < 0)
        diff = -1*diff;
    return diff;
}

double CirclePathTrajectory::realmod(double x, double y)
{
    double result = fmod(x, y);
    return result >= 0 ? result : result + y;
};

double CirclePathTrajectory::getSAtTime(double t)
{
    double s;
    if (t < tb)
    {
        s = max_a/2.0*t*t;
    }
    else if (tb <= t && t <= (tf - tb))
    {
        s = pa_x + vel*(t-tb);
    }
    else
    {
        s = sf - max_a/2*(tf-t)*(tf-t);
    }
    return s;
}

double CirclePathTrajectory::getDsAtTime(double t)
{
    double ds;
    if (t < tb)
    {
        ds = max_a*t;
    }
    else if (tb <= t && t <= (tf - tb))
    {
        ds = vel;
    }
    else
    {
        ds = max_a*(tf-t);
    }
    return ds;
}

Vector3d CirclePathTrajectory::trajPos(double t)
{
    if (t > tf)
    {
        return posef.getPos();
    }

    s = getSAtTime(t);
    Vector3d pos;
    pos(0) = posei.position(0) + radius*cos(s)-radius;
    pos(1) = posei.position(1) + radius*sin(s);
    pos(2) = posei.position(2) + z_height*sin(z_freq*s);
    return pos;
}

Vector3d CirclePathTrajectory::trajLinVel(double t)
{
    if (t > tf)
    {
        return Vector3d::Zero();
    }

    s = getSAtTime(t);
    ds = getDsAtTime(t);
    Vector3d vel;
    vel(0) = -1*radius*sin(s)*ds;
    vel(1) = radius*cos(s)*ds;
    vel(2) = z_height*z_freq*cos(z_freq*s)*ds;
    return vel;
}

Quat CirclePathTrajectory::trajOrient(double t)
{
    if (t > tf)
    {
        return posef.getOrientation();
    }

    // The orientation direction points to the center of the circle
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
    Vector3d pos = trajPos(t);
    Vector3d normal = Vector3d(pos(0), pos(1), 0) - target;
    if (normal.norm() < 1e-04)
    {
        normal = normali;
    }
    normal.normalize();
    Vector3d orient = approach.cross(normal);
    R.col(0) = normal;
    R.col(1) = orient;
    R.col(2) = approach;
    return Quat::rotm2quat(R);
}

Vector3d CirclePathTrajectory::trajAngVel(double t, double ts, const Quat &quat_prev)
{
    if (t > tf)
    {
        return Vector3d::Zero();
    }

    Vector3d vel(0, 0, 0);
    if (t > ts) // The first angular velocity is zero, i.e. t <= ts
    {
        Quat quat_des = trajOrient(t);
        vel = 2*quatDiff(quat_prev, quat_des).getV()/ts;
    }
    return vel;
}

double CirclePathTrajectory::getPos(char coord, double t)
{
    // validateTime(t);
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

double CirclePathTrajectory::getLinVel(char coord, double t)
{
    // validateTime(t);
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

Quat CirclePathTrajectory::getOrient(double t)
{
    return trajOrient(t);
}

Pose CirclePathTrajectory::getPose(double t)
{
    // validateTime(t);
    Vector3d pos = trajPos(t);
    Quat orient = trajOrient(t);    
    return Pose(pos, orient);
}

VectorXd CirclePathTrajectory::getPoseVec(double t)
{
    return getPose(t).vecRep();
}

VectorXd CirclePathTrajectory::getVel(double t, double ts, const Quat &prev_orient)
{
    // validateTime(t);
    VectorXd vel(6);
    vel.head(3) = trajLinVel(t);
    vel.tail(3) = trajAngVel(t, ts, prev_orient);
    return vel;
}

