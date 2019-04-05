#ifndef POLYNOM_INTERP_H
#define POLYNOM_INTERP_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/Quat.h"
#include <vector>

using namespace std;
using namespace Eigen;

class PolynomInterp
{
private:
    static std::vector<double> polynomCoef(double qi, double dqi, double ddqi, double qf, double dqf, double ddqf, double ti, double tf);
    static double qPol(double t, std::vector<double> coef);
    static double dqPol(double t, std::vector<double> coef);
    static double ddqPol(double t, std::vector<double> coef);

    static Quat quatDerNorm(Quat w, double dN, Quat q);
    static Quat quatSecDerNorm(Quat w, Quat dw, double dN, double ddN, Quat q);    
    static std::vector<Quat> quatPolynomCoef(Quat qk, Quat qwk, Quat dqwk, Quat qf, Quat dqf, Quat ddqf, double tk, double tf);
    static Quat quatPol(double tau, std::vector<Quat> coef);
    static Quat dquatPol(double t, double ti, double tf, std::vector<Quat> coef);
    static Quat ddquatPol(double t, double ti, double tf, std::vector<Quat> coef);

public:
    /**
     * Fifth order polynomial interpolation
     * qi: initial position
     * dqi: initial velocity
     * ddqi: initial acceleration
     * qf: final position
     * dqf: final velocity
     * ddqf: final acceleration
     * ti: start time
     * tf: end time
     * ts: sampling time
     * q: vector of output positions
     * dq: vector of output velocities
     * ddq: vector of output accelerations
     */
    static void fithOrderInterp(double qi, double dqi, double ddqi, double qf, double dqf, double ddqf, double ti, double tf, double ts, std::vector<double> &q, std::vector<double> &dq, std::vector<double> &ddq, std::vector<double> &time);
    
    /**
     * Quaternion polynomial interpolation
     * qi: initial quaterion
     * wi: initial angular velocity
     * dwi: initial angular acceleration
     * qf: final quaterion
     * wf: final angular velocity
     * dwf: final angular acceleration
     * ti: start time
     * tf: end time
     * ts: sampling time
     * quat: vector of output quaternion trajectory
     * w: vector of output angular velocities
     * dw: vector of output angular accelerations
     */
    static void quatPolynomInterp(Quat qi, Vector3d wi, Vector3d dwi, Quat qf, Vector3d wf, Vector3d dwf, double ti, double tf, double ts, std::vector<Quat> &quat, std::vector<Vector3d> &w, std::vector<Vector3d> &dw, std::vector<double> &time);
};


#endif