#ifndef POSE_TRAJ_H
#define POSE_TRAJ_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/kinematics/Quat.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include <vector>

using namespace std;
using namespace Eigen;

class PoseIterTrajectory
{
private:
    double ti, tf;
    Pose posei, posef;
    std::vector<double> posXCoeff, posYCoeff, posZCoeff;
    std::vector<Quat> orientCoeff;
    std::vector<Pose> poses;
    std::vector<VectorXd> poses_vec_rep;
    std::vector<VectorXd> velocities;
    /**
     * Validate the time is inside the trajectory
     */
    void validateTime(double t);

public:
    // Creates object and computes the coefficients for the trajectory
    PoseIterTrajectory(Pose posei, Vector3d linVeli, Vector3d linAcci, Vector3d angVeli, Vector3d angAcci,
                   Pose posef, Vector3d linVelf, Vector3d linAccf, Vector3d angVelf, Vector3d angAccf,
                   double ti, double tf);
   
    /**
     * Returns the polynomial coefficients of the coordinate 'coord'.
     * The variable sel can have the values {'x','y','z'}.
     */
    std::vector<double> getPosCoeff(char coord);

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
     * Returns the linear acceleration coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */
    double getLinAcc(char coord, double t);

    /**
     * Returns the maximum value of all the linear velocities,
     * and stores the coordinate with its maximum velocity in coord.
     */    
    double getMaxLinVel(char &coord);

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
     * Returns the angular acceleration coordinate 'coord' at time t.
     * The variable sel can have the values {'x','y','z'}.
     */
    double getAngAcc(char coord, double t);

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
    
    /**
     * Returns the acceleration at time t.
     */
    //VectorXd accelerationsAt(double t);
    
};

#endif