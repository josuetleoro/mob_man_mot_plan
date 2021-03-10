#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/kinematics/Quat.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include <vector>

using namespace std;
using namespace Eigen;

class Trajectory
{
protected:
    double ti, tf;
    double s, ds;   // Timing variables

    /**
     * Validate the time is inside the trajectory
     */
    void validateTime(double t);    

    // /**
    //  * Returns the timing variable (s) at time t.
    //  */
    // virtual double getSAtTime(double t) = 0;

    // /**
    //  * Returns the derivative of the timing variable (ds) at time t.
    //  */
    // virtual double getDsAtTime(double t) = 0;

    /**
     * Returns the position vector given time t.
     */
    virtual Vector3d trajPos(double t) = 0;

    /**
     * Returns the linear velocity given time t.
     */
    virtual Vector3d trajLinVel(double t) = 0;

    /**
     * Returns the quaternion orientation given time t.
     */
    virtual Quat trajOrient(double t) = 0;

    /**
     * Returns the angular velocity given time t.
     */
    virtual Vector3d trajAngVel(double t) = 0;

public:

    // Constructor
    Trajectory(double ti, double tf);
    
    /**
     * Returns the position coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */
    virtual double getPos(char coord, double t);

    /**
     * Returns the linear velocity coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */    
    virtual double getLinVel(char coord, double t);

    /**
     * Returns the quaternion orientation 'part' at time t.
     * The variable sel can have the values {w',x','y','z'}.
     */
    virtual double getOrientPart(char part, double t);

    /**
     * Returns the quaternion orientation at time t.
     */
    virtual Quat getOrient(double t);

    /**
     * Returns the angular velocity coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */    
    virtual double getAngVel(char coord, double t);

    /**
     * Returns the pose at time t.
     */
    virtual Pose getPose(double t);
       
    /**
     * Returns the pose at time t in vector respresentation.
     */
    virtual VectorXd getPoseVec(double t);

    /**
     * Returns the velocity vector at time t.
     */
    virtual VectorXd getVel(double t);
    
};

#endif