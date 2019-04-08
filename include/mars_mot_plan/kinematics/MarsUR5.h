#ifndef MARSUR5_H
#define MARSUR5_H

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/kinematics/DualQuat.h"
using namespace std;

class MarsUR5
{
private:
    //lenght of UR5 links
	double l1, l2, l3, l4, l5, l6;

    //Position of UR5 base with respect to the center of the wheels
    double a, b;

    //Directions and points for each joint
    //Points and Directions of each joint
	Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;
	Eigen::Vector3d d1, d2, d3, d4, d5, d6, d7, d8;

	//Moments of the axes
	Eigen::Vector3d m1, m2, m3, m4, m5, m6, m7, m8;

	//Dual-quaternion representation of each axis of the UR5
	DualQuat dQ1, dQ2, dQ3, dQ4, dQ5, dQ6;

    // Dual Quaternion for mobile platform
    DualQuat dqmp;

	//Dual-quaternion representation in plucker coordinates of the tool axes
    DualQuat dqL6;	//approach direction
	DualQuat dqL7;	//orientation direction

	//Current position and orientation matrix
	Eigen::Matrix4d T0e;	

public:
	MarsUR5();
    Eigen::Matrix4d forwardKin(const Eigen::VectorXd& q);
protected:
	Eigen::Vector3d LinesIntersection(DualQuat dqL1, DualQuat dqL2);
};


#endif
