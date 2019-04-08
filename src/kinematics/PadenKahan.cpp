#include "mars_mot_plan/kinematics/PadenKahan.h"
#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>

using namespace std;
double zeroTh = 1.0e-08;

double PadenKahan::subproblem1(Vector3d d, Vector3d p, Vector3d q, Vector3d r)
{
	Vector3d u = p - r;
	Vector3d v = q - r;
	Vector3d up = u - d*(d.dot(u));
	Vector3d vp = v - d*(d.dot(v));	
	return atan2(d.dot(up.cross(vp)),up.dot(vp));
}

int PadenKahan::subproblem2(Vector3d d1, Vector3d d2, Vector3d p, Vector3d q, Vector3d r, double * th1, double * th2)
{
	Vector3d u = p - r;
	Vector3d v = q - r;
	
	double dot_d1d2 = d1.dot(d2);
	Vector3d cross_d1d2 = d1.cross(d2);

	//Calculate alpha, beta and gamma
	double alpha = (dot_d1d2*d2.dot(u) - d1.dot(v)) / (pow(dot_d1d2, 2) - 1);
	double beta = (dot_d1d2*d1.dot(v) - d2.dot(u)) / (pow(dot_d1d2, 2) - 1);
	double gamma2 = (u.dot(u) - alpha*alpha - beta*beta - 2 * alpha*beta*d1.dot(d2)) / (cross_d1d2.dot(cross_d1d2));

	//Start assumung we have to solutions
	int sol = 2;
	if (abs(gamma2) < zeroTh) //If gamma2 = zero just one solution
	{
		sol = 1;
		//Calculate z1
		Vector3d z1 = alpha*d1 + beta*d2;
		//One solution
		th1[0] = subproblem1(d1, z1, v, Vector3d::Zero());
		th2[0] = subproblem1(d2, u, z1, Vector3d::Zero());
	}
	else if (gamma2 < 0.0) //If gamma2 is negative there is no solution
	{
		sol = 0;
	}
	else //Otherwise there are two solutions
	{
		double gamma = sqrt(gamma2);

		//Calculate z1 and z2
		Vector3d z1 = alpha*d1 + beta*d2 + gamma*cross_d1d2;
		Vector3d z2 = alpha*d1 + beta*d2 - gamma*cross_d1d2;

		//First solution
		th1[0] = subproblem1(d1, z1, v, Vector3d::Zero());
		th2[0] = subproblem1(d2, u, z1, Vector3d::Zero());

		//Second solution
		th1[1] = subproblem1(d1, z2, v, Vector3d::Zero());
		th2[1] = subproblem1(d2, u, z2, Vector3d::Zero());
	}
	return sol;
}

int PadenKahan::subproblem2pa(Vector3d d1, Vector3d p, Vector3d q, Vector3d r1, Vector3d r2x, double *th1, double *th2)
{
	Vector3d rq = (r1 - r2x).dot(d1)*d1;
	Vector3d r2 = r2x + rq;
	Vector3d u = p - r1;
	Vector3d v = q - r2;
	Vector3d rp = r1 - r2;
	Vector3d d2 = rp.normalized();

	//Calculate alpha, beta and gamma
	double alpha = d1.dot(u);
	double beta = (u.squaredNorm() - v.squaredNorm() - rp.squaredNorm()) / (-2 * rp.dot(d2));
	double gamma2 = v.squaredNorm() - alpha*alpha - beta*beta;

	//Start assumung we have to solutions
	int sol = 2;
	if (abs(gamma2) < zeroTh) //If gamma2 = zero just one solution
	{
		sol = 1;
		//Calculate z1 and z2
		Vector3d z2 = alpha*d1 + beta*d2;
		Vector3d z1 = z2 - rp;
		//One solution
		th1[0] = subproblem1(d1, z2, v, Vector3d::Zero());
		th2[0] = subproblem1(d1, u, z1, Vector3d::Zero());
	}
	else if (gamma2 < 0.0) //If gamma2 is negative there is no solution
	{
		sol = 0;
	}
	else //Otherwise there are two solutions
	{
		double gamma = sqrt(gamma2);
		Vector3d cross_d1d2 = d1.cross(d2);
		
		//First solution
		Vector3d z2 = alpha*d1 + beta*d2 + gamma*cross_d1d2;
		Vector3d z1 = z2 - rp;
		th1[0] = subproblem1(d1, z2, v, Vector3d::Zero());
		th2[0] = subproblem1(d1, u, z1, Vector3d::Zero());

		//Second solution
		z2 = alpha*d1 + beta*d2 - gamma*cross_d1d2;
		z1 = z2 - rp;
		th1[1] = subproblem1(d1, z2, v, Vector3d::Zero());
		th2[1] = subproblem1(d1, u, z1, Vector3d::Zero());
	}
return sol;
}