#include "mars_mot_plan/traj_plan/SineWavePathTrajectory.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include <iostream>
using namespace std;
using namespace Eigen;

SineWavePathTrajectory::SineWavePathTrajectory()
:Trajectory(0, 0)
{
    d = 0; lambda = 0; n_waves = 0; height = 0;
    vel = 0;
    max_a = 0;
};

// Computes the coefficients for the trajectory
SineWavePathTrajectory::SineWavePathTrajectory(Pose posei, double ti, double tf, double tb, double d, double n_waves, double height)
:Trajectory(ti, tf)
{
    this->posei = posei; this->posef = posef;

    // Parameters that define the size of the sine wave path
    this->d = d;
    this->n_waves = n_waves;
    this->height = height;
    
    // Calculate the path parameters
    lambda = d / n_waves;
    sf = d;

    // Parameters obtained from the give times
    vel = sf/(tf-tb);
    max_a = vel/tb;

    // Find the knot point
    this->tb = tb;
    pa_x = max_a/2*tb*tb;
    
    // Orientation is fixed for this path
    Quat orienti = this->posei.orientation;
}

double SineWavePathTrajectory::getSAtTime(double t)
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

double SineWavePathTrajectory::getDsAtTime(double t)
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

Vector3d SineWavePathTrajectory::trajPos(double t)
{
    s = getSAtTime(t);
    Vector3d pos;
    pos(0) = posei.position(0) + s;
    pos(1) = posei.position(1) + 0;
    pos(2) = posei.position(2) + height*sin(2*M_PI/lambda*s);
    return pos;
}

Vector3d SineWavePathTrajectory::trajLinVel(double t)
{
    s = getSAtTime(t);
    ds = getDsAtTime(t);
    Vector3d vel;
    vel(0) = ds;
    vel(1) = 0;
    vel(2) = 2*M_PI/lambda*height*cos(2*M_PI/lambda*s)*ds;
    return vel;
}

Quat SineWavePathTrajectory::trajOrient(double t)
{
    return posei.orientation;
}

Vector3d SineWavePathTrajectory::trajAngVel(double t)
{
    return Vector3d::Zero();
}
