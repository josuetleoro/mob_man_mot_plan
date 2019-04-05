#include "mars_mot_plan/DualQuat.h"

DualQuat::DualQuat()
{
	this->prim = Quat();
	this->dual = Quat();
}

DualQuat::DualQuat(Quat prim, Quat dual)
{
	this->prim = prim;
	this->dual = dual;
}

DualQuat::DualQuat(double w0, double x0, double y0, double z0, double wE, double xE, double yE, double zE)
{
	this -> prim = Quat(w0, x0, y0, z0);
	this -> dual = Quat(wE, xE, yE, zE);
}

Quat DualQuat::getPrim()
{
	return this->prim;
}

Quat DualQuat::getDual()
{
	return this->dual;
}

DualQuat DualQuat::operator+(const DualQuat & dq2)
{
	return DualQuat(this->prim + dq2.prim, this->dual + dq2.dual);
}

DualQuat DualQuat::operator-(const DualQuat & dq2)
{
	return DualQuat(this->prim - dq2.prim, this->dual - dq2.dual);
}

DualQuat DualQuat::operator*(const DualQuat & q2)
{
	return DualQuat(this->prim*q2.prim,
					this->prim*q2.dual + this->dual*q2.prim);
}

DualQuat operator*(const double scalar, const DualQuat & q)
{
	return DualQuat(scalar*q.prim, scalar*q.dual);
}

DualQuat operator*(const DualQuat & q, const double scalar)
{
	return DualQuat(scalar*q.prim, scalar*q.dual);
}

DualQuat DualQuat::conj()
{
	return DualQuat(this->prim.conj(), this->dual.conj());
}

ostream & operator<<(ostream & os, const DualQuat & q)
{
	//First output the primary part
	os << q.prim;

	//Then the dual part
	cout << "+E(" << q.dual << ")";

	return os;
}

DualQuat DualQuat::rigidTransf(double angle, Eigen::Vector3d d, Eigen::Vector3d m)
{
	double angle_by_2 = angle*0.5;
	return DualQuat(Quat(cos(angle_by_2), sin(angle_by_2)*d), Quat(0, sin(angle_by_2)*m));
}

Vector3d DualQuat::rotPoint(Vector3d p)
{
	//Quat Qp = Quat(0, p);	//Quaternion representation of a point
	//Quat Qq = this->prim * Qp * this->prim.conj() + this->dual*this->prim.conj() - this->prim * this->dual.conj();

	Quat Qq =  this->dual*this->prim.conj() - this->prim * this->dual.conj();
	return this->prim.rotVector(p) + Qq.getV();
}

Vector3d DualQuat::rotVector(Vector3d v)
{
	return this->prim.rotVector(v);
}

Eigen::Vector3d DualQuat::linesIntPlucker(DualQuat l1, DualQuat l2)
{
	Eigen::Vector3d m1 = l1.getDual().getV();
	Eigen::Vector3d d1 = l1.getPrim().getV();
	Eigen::Vector3d m2 = l2.getDual().getV();
	Eigen::Vector3d d2 = l2.getPrim().getV();
	
	return d1.cross(m1) + d2.cross(m2).dot(d1)*d1;
}
