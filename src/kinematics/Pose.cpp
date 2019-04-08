#include "mars_mot_plan/kinematics/Pose.h"

Pose::Pose()
{
	this->position = Vector3d::Zero();
	this->orientation = Quat();
}

Pose::Pose(double x, double y, double z, double quat_w, double quat_x, double quat_y, double quat_z)
{
	this->position(0) = x;
	this->position(1) = y;
	this->position(2) = z;
	this->orientation = Quat(quat_w, quat_x, quat_y, quat_z);
}

Pose::Pose(Vector3d position, Quat orientation)
{
	this->position = position;
	this->orientation = orientation;
}

Vector3d Pose::getPos()
{
	return this->position;
}

Quat Pose::getOrientation()
{
    return this->orientation;
}

VectorXd Pose::vecRep()
{
    VectorXd pose_vector(7);
    pose_vector << position(0) , position(1) , position(2) , orientation.w , orientation.x , orientation.y , orientation.z;
    return pose_vector;
}

VectorXd Pose::pose_diff(const Pose & pose1, const Pose & pose2)
{
    VectorXd diff(6);
    diff.head(3) = pose1.position - pose2.position;
    Quat orient_diff = pose1.orientation*pose2.orientation.conj();
    if (orient_diff.w < 0)
    {
        orient_diff = -1*orient_diff;
    }
    diff.tail(3) = orient_diff.getV();
    return diff;
}


ostream & operator<<(ostream & os, const Pose & pose)
{
    os <<"x: " <<  pose.position(0) << ", ";
    os <<"y: " <<  pose.position(1) << ", ";
    os <<"z: " <<  pose.position(2) << ", ";
	
	os <<"quat: " << pose.orientation;
	return os;
}