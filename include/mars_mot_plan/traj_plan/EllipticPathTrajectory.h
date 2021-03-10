#ifndef ELLIPTIC_PATH_TRAJECTORY_H
#define ELLIPTIC_PATH_TRAJECTORY_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/traj_plan/Trajectory.h"
#include "mars_mot_plan/kinematics/Quat.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include <vector>

using namespace std;
using namespace Eigen;

class EllipticPathTrajectory: public Trajectory
{
protected:
    // Path parameters
    Vector2d center;
    double a, b, thi, thf, m_z, zi;
    Pose posei, posef;
    double s, ds, dds;  // Timing variable s
    std::vector<double> sCoeff;
    std::vector<Quat> orientCoeff;    

    double realmod (double x, double y)
    {
        double result = fmod(x, y);
        return result >= 0 ? result : result + y;
    };

    /**
     * Returns the timing variable (s) at time t.
     */
    double getSAtTime(double t);

    /**
     * Returns the derivative of the timing variable (ds) at time t.
     */
    double getDsAtTime(double t);

    /**
     * Returns the position vector given time t.
     */
    Vector3d trajPos(double t);

    /**
     * Returns the linear velocity given time t.
     */
    Vector3d trajLinVel(double t);

    /**
     * Returns the quaternion orientation given time t.
     */
    Quat trajOrient(double t);

    /**
     * Returns the angular velocity given time t.
     */
    Vector3d trajAngVel(double t);

public:

    // Default constructor
    EllipticPathTrajectory();

    // Computes trajectory parameters
    EllipticPathTrajectory(Pose posei, Pose posef, double ti, double tf);
    
};

#endif