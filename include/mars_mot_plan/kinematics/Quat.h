#ifndef QUAT_H
#define QUAT_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
using namespace std;
using namespace Eigen;

class Quat
{
public:
	double w;
	double x;
	double y;
	double z;
	Quat();
	Quat(double w, double x, double y, double z);
	Quat(double w, Vector3d v);
	double getS();
	Vector3d getV();	
	double norm() const;
	double squaredNorm() const;
	void normalize();
	Quat conj() const;
	Quat inv() const;
	Quat operator+ (const Quat& q2) const;
	Quat operator- (const Quat& q2) const;
	Quat operator* (const Quat& q2) const; //Quaternion-quaternion multiplication
	//friend Quat operator* (const Quat &q1, const Quat &q2); 		//Quaternion-quaternion multiplication
	friend Quat operator* (const double scalar, const Quat& q); //Scalar-quaternion multiplication
	friend Quat operator* (const Quat& q, const double scalar); //Quaternion-scalar multiplication
	friend Quat operator/ (const Quat& q, const double scalar); //Quaternion-scalar division	
	static Quat rotationOperator(double angle, Vector3d axis);
	static Quat rotm2quat(Matrix3d R);
	friend ostream& operator<<(ostream & os, const Quat & q);

	//Methods for rotation
	Vector3d rotPoint(Vector3d p);
	Vector3d rotVector(Vector3d p);
};


#endif