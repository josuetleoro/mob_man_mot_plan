#ifndef LINE_PATH_TRAJECTORY_H
#define LINE_PATH_TRAJECTORY_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/traj_plan/Trajectory.h"
#include "mars_mot_plan/kinematics/Quat.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include <vector>

using namespace std;
using namespace Eigen;

class LinePathTrajectory: public Trajectory
{
protected:
    // Path parameters
    Pose posei, posef;
    std::vector<double> posXCoeff, posYCoeff, posZCoeff;
    std::vector<Quat> orientCoeff;

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
    LinePathTrajectory();

    // Computes trajectory parameters
    LinePathTrajectory(Pose posei, Vector3d linVeli, Vector3d linAcci, Vector3d angVeli, Vector3d angAcci,
                   Pose posef, Vector3d linVelf, Vector3d linAccf, Vector3d angVelf, Vector3d angAccf,
                   double ti, double tf);
    
};

#endif