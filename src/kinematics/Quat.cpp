#include "mars_mot_plan/kinematics/Quat.h"
using namespace std;

Quat::Quat()
{
	this->w = 0;
	this->x = 0;
	this->y = 0;
	this->z = 0;
}

Quat::Quat(double w, double x, double y, double z)
{
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}

Quat::Quat(double w, Eigen::Vector3d v)
{
	this->w = w;
	this->x = v(0);
	this->y = v(1);
	this->z = v(2);
}

double Quat::getS()
{
	return this->w;
}

Eigen::Vector3d Quat::getV()
{
	return Eigen::Vector3d(this->x,this->y,this->z);
}

double Quat::norm() const
{
	return sqrt(this->w*this->w+this->x*this->x+this->y*this->y+this->z*this->z);
}

double Quat::squaredNorm() const
{
	return this->w*this->w+this->x*this->x+this->y*this->y+this->z*this->z;
}

void Quat::normalize()
{
	double n = this->squaredNorm();
	this->w = this->w/n;
	this->x = this->x/n;
	this->y = this->y/n;
	this->z = this->z/n;
}

Quat Quat::conj() const
{
	return Quat(this->w,-1*this->x,-1*this->y,-1*this->z);
}

Quat Quat::inv() const
{
	return this->conj()/this->squaredNorm();
}

Quat Quat::operator+ (const Quat& q2) const
{
	return Quat(this->w + q2.w,
				this->x + q2.x,
				this->y + q2.y,
				this->z + q2.z);
}

Quat Quat::operator-(const Quat & q2) const
{
	return Quat(this->w - q2.w,
		this->x - q2.x,
		this->y - q2.y,
		this->z - q2.z);
}

Quat Quat::operator*(const Quat & q2) const
{
	return Quat(this->w*q2.w-this->x*q2.x-this->y*q2.y-this->z*q2.z,
		this->w*q2.x + this->x*q2.w + this->y*q2.z - this->z*q2.y,
		this->w*q2.y + this->y*q2.w + this->z*q2.x - this->x*q2.z,
		this->w*q2.z + this->z*q2.w + this->x*q2.y - this->y*q2.x);
}

Vector3d Quat::rotPoint(Vector3d p)
{
	Quat qp = Quat(0, p);
	Quat qp_rot = (*this)*qp*(*this).conj();
	return qp_rot.getV();
}

Vector3d Quat::rotVector(Vector3d v)
{
	Vector3d qV = this->getV();
	Vector3d t = 2 * qV.cross(v);	
	return v + this->getS()*t + qV.cross(t);
}

Quat operator*(const double scalar, const Quat & q)
{
	return Quat(scalar*q.w, scalar*q.x, scalar*q.y, scalar*q.z);
}

Quat operator*(const Quat & q, const double scalar)
{
	return Quat(scalar*q.w, scalar*q.x, scalar*q.y, scalar*q.z);
}

Quat operator/(const Quat& q, const double scalar)
{
	return Quat(1/scalar*q.w, 1/scalar*q.x, 1/scalar*q.y, 1/scalar*q.z);
}

Quat Quat::rotationOperator(double angle, Eigen::Vector3d axis)
{
	return Quat(cos(angle * 0.5), sin(angle * 0.5)*axis);
}

Quat rotm2quat(Matrix3d R)
{
	//Validate the rotation matrix
	bool isOrthonormal;
	Matrix3d test;
	test = R.transpose()*R;
	isOrthonormal = test.isApprox(Matrix3d::Identity(),0.001);
	if (!isOrthonormal)
	{
		throw "Received rotation matrix is not valid";
	}
	Quaterniond q(R);
	return Quat(q.w(), q.vec());
}

ostream & operator<<(ostream & os, const Quat & q)
{
	os << q.w;
	if (q.x >= 0.0)
		os << "+";
	os << q.x <<"i";
	if (q.y >= 0.0)
		os << "+";
	os << q.y << "j";
	if (q.z >= 0.0)
		os << "+";
	os << q.z << "k";
	return os;
}