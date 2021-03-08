#ifndef ELLIPTIC_PATH_TRAJ_H
#define ELLIPTIC_PATH_TRAJ_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/kinematics/Quat.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include <vector>

using namespace std;
using namespace Eigen;

class EllipticPathTraj
{
private:
    // Path parameters
    Vector2d center;
    double a, b, thi, thf, m_z, zi;

    int n;
    std::vector<double> s, ds, dds;  // Timing variable s
    std::vector<double> x, dx, ddx;
    std::vector<double> y, dy, ddy;
    std::vector<double> z, dz, ddz;
    std::vector<Quat> orientation;
    std::vector<Vector3d> w, dw;

    std::vector<double> time;
    std::vector<Pose> poses;
    std::vector<VectorXd> poses_vec_rep;
    std::vector<VectorXd> velocities;
    std::vector<VectorXd> accelerations;

    double realmod (double x, double y)
    {
        double result = fmod(x, y);
        return result >= 0 ? result : result + y;
    };

    Vector3d trajPos(double s);
    Vector3d trajVel(double s, double ds);

public:
    EllipticPathTraj(Pose posei, Pose posef, double ti, double tf, double ts);
    int size()
    {
        return n;
    }
    /**
     * Returns the trajectory of the coordiante given by sel.
     * The variable sel can have the values {'x','y','z'}.
     */
    std::vector<double> getTrajPosition(char sel);

    /**
     * Returns the trajectory of the linear velocity given by sel.
     * The variable sel can have the values {'x','y','z'}.
     */    
    std::vector<double> getTrajLinVel(char sel);

    /**
     * Returns the trajectory of the quaternion parts given by sel.
     * The variable sel can have the values {w',x','y','z'}.
     */
    std::vector<double> getTrajOrientation(char sel);

    /**
     * Returns the quaternion trajectory.
     */
    std::vector<Quat> getTrajQuat()
    {
        return orientation;
    }

    /**
     * Returns the trajectory of the angular velocity given by sel.
     * The variable sel can have the values {'x','y','z'}.
     */    
    std::vector<double> getTrajAngVel(char sel);

    /**
     * Returns the vector of time values.
     */
    std::vector<double> getTrajTime()
    {
        return time;
    }

    /**
     * Returns the kth pose of the trajectory.
     */
    Pose poseAt(int k)
    {
        return poses.at(k);
    }
   
    /**
     * Returns the kth pose of the trajectory in vector representation.
     */
    VectorXd poseVecAt(int k)
    {
        return poses_vec_rep.at(k);
    }

    /**
     * Returns the kth velocity vector of the trajectory.
     */
    VectorXd velocitiesAt(int k)
    {
        return velocities.at(k);
    }
   
};

#endif