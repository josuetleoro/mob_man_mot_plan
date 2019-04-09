#ifndef POSE_H
#define POSE_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/kinematics/Quat.h"
using namespace std;
using namespace Eigen;

class Pose
{
public:
    Vector3d position;
    Quat orientation;
	Pose();
	Pose(double x, double y, double z, double quat_w, double quat_x, double quat_y, double quat_z);
	Pose(Vector3d position, Quat orientation);
	Pose(const Matrix4d &T);
	Vector3d getPos() const;
	Quat getOrientation() const;
	VectorXd vecRep();
	Matrix4d matrixRep();	
    static VectorXd pose_diff(const Pose & pose1, const Pose & pose2);
	static Pose matrixToPose(const Matrix4d &T);
	static Matrix4d poseToMatrix(const Pose & pose);
	friend ostream& operator<<(ostream & os, const Pose & q);
};

#endif