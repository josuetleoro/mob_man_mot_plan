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

Pose::Pose(const Matrix4d &T)
{
    this->position = T.col(3).head(3);
    this->orientation = Quat::rotm2quat(T.block<3, 3>(0, 0));
}

Vector3d Pose::getPos() const
{
	return this->position;
}

Quat Pose::getOrientation() const
{
    return this->orientation;
}

VectorXd Pose::vecRep()
{
    VectorXd pose_vector(7);
    pose_vector << position(0) , position(1) , position(2) , orientation.w , orientation.x , orientation.y , orientation.z;
    return pose_vector;
}

Matrix4d Pose::matrixRep()
{
    Matrix4d T = Matrix4d::Zero();
    T(3,3) = 1.0;
    Quaterniond quat = Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z);
    T.block<3, 3>(0, 0) = quat.toRotationMatrix();
    T.col(3).head(3) = position;
    return T;
}

VectorXd Pose::pose_diff(const Pose &pose1, const Pose &pose2)
{
    VectorXd diff(6);
    diff.head(3) = pose1.position - pose2.position;

    //sO=qd(1)*qe(1)+qd(2:4)'*qe(2:4)
    //eO=qe(1)*qd(2:4)-qd(1)*qe(2:4)-cross(qd(2:4),qe(2:4))
    
    /*Quat orient_diff = pose1.orientation*pose2.orientation.conj();
    //cout << "orient_diff.w: " << orient_diff.w << endl;
    if (orient_diff.w < 0)
    {        
        orient_diff = -1*orient_diff;
    }
    diff.tail(3) = orient_diff.getV();*/

    /*cout << "pose1: " << pose1 << endl;
    cout << "pose2: " << pose2 << endl;*/
    double w1 = pose1.orientation.getS(), w2 = pose2.orientation.getS();
    Vector3d v1 = pose1.orientation.getV(), v2 = pose2.orientation.getV();
    double w0 = w1 * w2 + v1.dot(v2);
    Vector3d e0 = w2*v1 - w1 * v2 - v1.cross(v2);
    //cout << "w0: " << w0 << endl;
    if (abs(w0) > 1e-6)
    {
        if (w0 < 0)
        {        
            e0 = -1*e0;
        }
    }
    diff.tail(3) = e0;

    return diff;
}

Pose Pose::matrixToPose(const Matrix4d &T)
{
    Pose out_pose = Pose(T.col(3).head(3), Quat::rotm2quat(T.block<3, 3>(0, 0)));
    return out_pose;
}

Matrix4d Pose::poseToMatrix(const Pose & pose)
{
    Matrix4d T = Matrix4d::Zero();
    T(3,3) = 1.0;
    Quaterniond quat = Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    T.block<3, 3>(0, 0) = quat.toRotationMatrix();
    T.col(3).head(3) = pose.position;
    return T;    
}

ostream & operator<<(ostream & os, const Pose & pose)
{
    os <<"x: " <<  pose.position(0) << ", ";
    os <<"y: " <<  pose.position(1) << ", ";
    os <<"z: " <<  pose.position(2) << ", ";	
	os <<"quat: " << pose.orientation;
	return os;
}