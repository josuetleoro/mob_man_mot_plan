#include "mars_mot_plan/traj_plan/PoseTrajectory.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include <iostream>
using namespace std;
using namespace Eigen;

std::vector<double> PoseTrajectory::getTrajPosition(char sel)
{
    switch(sel)
    {
    case 'x':
        return x;
    case 'y':
        return y;
    case 'z':
        return z;
    }
}

std::vector<double> PoseTrajectory::getTrajLinVel(char sel)
{
    switch(sel)
    {
    case 'x':
        return dx;
    case 'y':
        return dy;
    case 'z':
        return dz;
    }
}

std::vector<double> PoseTrajectory::getTrajLinAcc(char sel)
{
    switch(sel)
    {
    case 'x':
        return ddx;
    case 'y':
        return ddy;
    case 'z':
        return ddz;
    }
}

std::vector<double> PoseTrajectory::getTrajOrientation(char sel)
{
    std::vector<double> quat_part;
    quat_part.resize(n);
    switch(sel)
    {
    case 'w':
        for (int k = 0; k < n; k++)
        {
            quat_part.at(k) = orientation.at(k).w;
        }
        break;
    case 'x':
        for (int k = 0; k < n; k++)
        {
            quat_part.at(k) = orientation.at(k).x;
        }
        break;
    case 'y':
        for (int k = 0; k < n; k++)
        {
            quat_part.at(k) = orientation.at(k).y;
        }
        break;
    case 'z':
        for (int k = 0; k < n; k++)
        {
            quat_part.at(k) = orientation.at(k).z;
        }
        break;
    }
    return quat_part;
}
 
std::vector<double> PoseTrajectory::getTrajAngVel(char sel)
{
    std::vector<double> vel;
    vel.resize(n);
    switch(sel)
    {    
    case 'x':
        for (int k = 0; k < n; k++)
        {
            vel.at(k) = w.at(k)(0);
        }
        break;
    case 'y':
        for (int k = 0; k < n; k++)
        {
            vel.at(k) = w.at(k)(1);
        }
        break;
    case 'z':
        for (int k = 0; k < n; k++)
        {
            vel.at(k) = w.at(k)(2);
        }
        break;
    }
    return vel;
}

std::vector<double> PoseTrajectory::getTrajAngAcc(char sel)
{
    std::vector<double> acc;
    acc.resize(n);
    switch(sel)
    {    
    case 'x':
        for (int k = 0; k < n; k++)
        {
            acc.at(k) = dw.at(k)(0);
        }
        break;
    case 'y':
        for (int k = 0; k < n; k++)
        {
            acc.at(k) = dw.at(k)(1);
        }
        break;
    case 'z':
        for (int k = 0; k < n; k++)
        {
            acc.at(k) = dw.at(k)(2);
        }
        break;
    }
    return acc;
}

PoseTrajectory::PoseTrajectory(Pose posei, Vector3d linVeli, Vector3d linAcci, Vector3d angVeli, Vector3d angAcci,
                   Pose posef, Vector3d linVelf, Vector3d linAccf, Vector3d angVelf, Vector3d angAccf,
                   double ti, double tf, double ts)
{
    // Position
    // x
    TrajPlan::fithOrderInterp(posei.position(0), linVeli(0), linAcci(0), posef.position(0), linVelf(0), linAccf(0), ti, tf, ts, x, dx, ddx, time);
    // y
    TrajPlan::fithOrderInterp(posei.position(1), linVeli(1), linAcci(1), posef.position(1), linVelf(1), linAccf(1), ti, tf, ts, y, dy, ddy, time);
    // z
    TrajPlan::fithOrderInterp(posei.position(2), linVeli(2), linAcci(2), posef.position(2), linVelf(2), linAccf(2), ti, tf, ts, z, dz, ddz, time);
    
    // Orientation
    TrajPlan::quatPolynomInterp(posei.orientation, angVeli, angAcci, posef.orientation, angVelf, angAccf, ti, tf, ts, orientation, w, dw, time);
    
    // Store the trajectory in the corresponding vectors
    double n = time.size();
    this->n = n;
    poses.resize(n);
    poses_vec_rep.resize(n);
    velocities.resize(n);
    accelerations.resize(n);
    for (int k=0; k<n; k++)
    {
        poses.at(k)=(Pose(this->x.at(k), this->y.at(k), this->z.at(k), this->orientation.at(k).w, this->orientation.at(k).x, this->orientation.at(k).y, this->orientation.at(k).z));
        poses_vec_rep.at(k) = poses.at(k).vecRep();
        velocities.at(k) = VectorXd(6);
        velocities.at(k) << dx.at(k), dy.at(k), dz.at(k), w.at(k)(0), w.at(k)(1), w.at(k)(2);
        accelerations.at(k) = VectorXd(6);
        accelerations.at(k) << ddx.at(k), ddy.at(k), ddz.at(k), dw.at(k)(0), dw.at(k)(1), dw.at(k)(2);
    }
}