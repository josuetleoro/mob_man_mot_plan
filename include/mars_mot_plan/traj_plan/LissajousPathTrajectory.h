#ifndef LISSAJOUS_PATH_TRAJECTORY_H
#define LISSAJOUS_PATH_TRAJECTORY_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/traj_plan/Trajectory.h"
#include "mars_mot_plan/kinematics/Quat.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include <vector>

using namespace std;
using namespace Eigen;

class LissajousPathTrajectory: public Trajectory
{
protected:
    Pose posei, posef;
    Quat orienti;
    // Path parameters
    double a, b, c, wa, wb, deltax, deltay;
    double si, sf, pa_x;
    double vel, max_a;
    double s, ds, tb;  // Timing variable s

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
    LissajousPathTrajectory();

    // Computes trajectory parameters
    LissajousPathTrajectory(Pose posei, double ti, double tf, double tb);
    
};

#endif