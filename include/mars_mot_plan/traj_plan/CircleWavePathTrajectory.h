#ifndef CIRCLE_PATH_TRAJECTORY_H
#define CIRCLE_PATH_TRAJECTORY_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/kinematics/Quat.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include <vector>

using namespace std;
using namespace Eigen;

class CirclePathTrajectory
{
protected:
    double ti, tf;
    Pose posei, posef;
    Vector3d orienti;
    // Path parameters
    double radius, z_height, z_freq;
    Vector3d target;
    Vector3d approach;
    
    // Timing variables
    double si, sf, pa_x;
    double vel, max_a;
    double s, ds, tb;  // Timing variable s

    // Needed to find the angular velocity
    Quat prev_orient;
    Quat quatDiff(const Quat &q0, const Quat &q1);

    double realmod(double x, double y);

    /**
     * Validate the time is inside the trajectory
     */
    void validateTime(double t);

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
    Vector3d trajAngVel(double t, double ts, const Quat &prev_orient);

public:

    // Default constructor
    CirclePathTrajectory();

    // Computes trajectory parameters
    CirclePathTrajectory(Pose posei, double ti, double tf, double tb, double radius = 1.6, double z_height = 0.25, double z_freq = 4);
    
    /**
     * Returns the position coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */
    double getPos(char coord, double t);

    /**
     * Returns the linear velocity coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */    
    double getLinVel(char coord, double t);

    /**
     * Returns the quaternion orientation at time t.
     */
    Quat getOrient(double t);

    /**
     * Returns the pose at time t.
     */
    Pose getPose(double t);
       
    /**
     * Returns the pose at time t in vector respresentation.
     */
    VectorXd getPoseVec(double t);

    /**
     * Returns the velocity vector at time t.
     */
    VectorXd getVel(double t, double ts, const Quat &prev_orient);

};

#endif