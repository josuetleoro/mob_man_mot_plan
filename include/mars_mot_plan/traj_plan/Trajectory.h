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
private:
    double ti, tf;

protected:    

    double realmod (double x, double y)
    {
        double result = fmod(x, y);
        return result >= 0 ? result : result + y;
    };

    /**
     * Validate the time is inside the trajectory
     */
    void validateTime(double t);

    /**
     * Get the trajectory starting time
     */
    double getTi();

    /**
     * Get the trajectory end time
     */
    double getTf();

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
    double getPos(char coord, double t);

    /**
     * Returns the linear velocity coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */    
    double getLinVel(char coord, double t);

    /**
     * Returns the quaternion orientation 'part' at time t.
     * The variable sel can have the values {w',x','y','z'}.
     */
    double getOrientPart(char part, double t);

    /**
     * Returns the quaternion orientation at time t.
     */
    Quat getOrient(double t);

    /**
     * Returns the angular velocity coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */    
    double getAngVel(char coord, double t);

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
    VectorXd getVel(double t);
    
};

#endif