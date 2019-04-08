#ifndef PADENKAHAN_H
#define PADENKAHAN_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

class PadenKahan
{
public:
	static double subproblem1(Vector3d d, Vector3d p, Vector3d q, Vector3d r);
	static int subproblem2(Vector3d d1, Vector3d d2, Vector3d p, Vector3d q, Vector3d r, double *th1, double *th2);
	static int subproblem2pa(Vector3d d1, Vector3d p, Vector3d q, Vector3d r1, Vector3d r2x, double *th1, double *th2);
};

#endif