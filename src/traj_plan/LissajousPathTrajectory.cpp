#include "mars_mot_plan/traj_plan/LissajousPathTrajectory.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include <iostream>
using namespace std;
using namespace Eigen;

LissajousPathTrajectory::LissajousPathTrajectory()
:Trajectory(0, 0)
{
    a = 0; b = 0; c = 0; wa = 0; wb = 0; deltax = 0; deltay = 0;
    vel = 0;
    max_a = 0;
};

// Computes the coefficients for the trajectory
LissajousPathTrajectory::LissajousPathTrajectory(Pose posei, double ti, double tf, double tb)
:Trajectory(ti, tf)
{
    this->posei = posei;

    // Parameters that define the size of the lissajous path
    a = 1.3;    // Length
    b = 1.3;    // Width
    c = 0.27;   // Height

    // Fixed parameters that define the Lissajous path
    deltax = 0.0;
    deltay = M_PI_2;
    wa = 1;
    wb = 2;
    sf = 2*M_PI;

    // Parameters obtained from the give times
    vel = sf/(tf-tb);
    max_a = vel/tb;

    // Find the knot point
    this->tb = tb;
    pa_x = max_a/2*tb*tb;
    
    // Orientation is fixed for this path
    Quat orienti = this->posei.orientation;

    // Find the final pose
    Vector3d posf = trajPos(tf);
    Quat orientf = trajOrient(tf);
    posef = Pose(posf, orientf);
}

double LissajousPathTrajectory::getSAtTime(double t)
{
    double s;
    if (t < tb)
    {
        s = max_a/2.0*t*t;
    }
    else if (tb <= t && t <= (getTf() - tb))
    {
        s = pa_x + vel*(t-tb);
    }
    else
    {
        s = sf - max_a/2*(getTf()-t)*(getTf()-t);
    }
    return s;
}

double LissajousPathTrajectory::getDsAtTime(double t)
{
    double ds;
    if (t < tb)
    {
        ds = max_a*t;
    }
    else if (tb <= t && t <= (getTf() - tb))
    {
        ds = vel;
    }
    else
    {
        ds = max_a*(getTf()-t);
    }
    return ds;
}

Vector3d LissajousPathTrajectory::trajPos(double t)
{
    if (t > getTf())
    {
        return posef.getPos();
    }

    s = getSAtTime(t);
    Vector3d pos;
    pos(0) = posei.position(0) + a*cos(wa*(s+M_PI_2) + deltax);
    pos(1) = posei.position(1) + b*cos(wb*(s+M_PI_2) + deltay);
    pos(2) = posei.position(2) + c*cos(2*s)-c;
    return pos;
}

Vector3d LissajousPathTrajectory::trajLinVel(double t)
{
    if (t > getTf())
    {
        return Vector3d::Zero();
    }

    s = getSAtTime(t);
    ds = getDsAtTime(t);
    Vector3d vel;
    vel(0) = -a*sin(wa*(s+M_PI_2) + deltax)*wa*ds;
    vel(1) = -b*sin(wb*(s+M_PI_2) + deltay)*wb*ds;
    vel(2) = -2*c*sin(2*s)*ds;
    return vel;
}

Quat LissajousPathTrajectory::trajOrient(double t)
{
    if (t > getTf())
    {
        return posef.getOrientation();
    }

    return posei.orientation;
}

Vector3d LissajousPathTrajectory::trajAngVel(double t)
{
    if (t > getTf())
    {
        return Vector3d::Zero();
    }

    return Vector3d::Zero();
}
