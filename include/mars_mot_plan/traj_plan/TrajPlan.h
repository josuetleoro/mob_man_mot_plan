#ifndef TRAJ_PLAN_H
#define TRAJ_PLAN_H

#include <math.h>
#include <Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/kinematics/Quat.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include <vector>

using namespace std;
using namespace Eigen;

class TrajPlan
{
private:
    static Quat quatDerNorm(Quat w, double dN, Quat q);
    static Quat quatSecDerNorm(Quat w, Quat dw, double dN, double ddN, Quat q);    
    static Quat dquatPol(double t, double ti, double tf, const std::vector<Quat> &coef);
    static Quat ddquatPol(double t, double ti, double tf, const std::vector<Quat> &coef);

public:
    static std::vector<double> polynomCoef(double qi, double dqi, double ddqi, double qf, double dqf, double ddqf, double ti, double tf);
    static double qPol(double t, const std::vector<double> &coef);
    static double dqPol(double t, const std::vector<double> &coef);
    static double ddqPol(double t, const std::vector<double> &coef);

    static std::vector<Quat> quatPolynomCoef(Quat qk, Vector3d wi, Vector3d dwi, Quat qf, Vector3d wf, Vector3d dwf, double ti, double tf);
    static Quat quatPol(double t, double ti, double tf, const std::vector<Quat> &coef);
    static Vector3d wPol(double t, double ti, double tf, const std::vector<Quat> &coef);
    static Vector3d dwPol(double t, double ti, double tf, const std::vector<Quat> &coef);

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
     * time: vector of output time
     */
    static void fithOrderInterp(double qi, double dqi, double ddqi, double qf, double dqf, double ddqf, double ti, double tf, double ts, std::vector<double> &q, std::vector<double> &dq, std::vector<double> &ddq, std::vector<double> &time);
    
    /**
     * Quaternion polynomial interpolation
     * qi: initial quaternion
     * wi: initial angular velocity
     * dwi: initial angular acceleration
     * qf: final quaternion
     * wf: final angular velocity
     * dwf: final angular acceleration
     * ti: start time
     * tf: end time
     * ts: sampling time
     * quat: vector of output quaternion trajectory
     * w: vector of output angular velocities
     * dw: vector of output angular accelerations
     * time: vector of output time
     */
    static void quatPolynomInterp(Quat qi, Vector3d wi, Vector3d dwi, Quat qf, Vector3d wf, Vector3d dwf, double ti, double tf, double ts, std::vector<Quat> &quat, std::vector<Vector3d> &w, std::vector<Vector3d> &dw, std::vector<double> &time);

    /**
     * Pose polynomial interpolation: Fifth order for position Quaternion polynomial interpolation for orientation
     * Posei: initial pose
     * linVeli: vector of initial linear velocities
     * linAcci: vector of initial linear accelerations
     * angVeli: vector of initial angular velocities
     * angAcci: vector of initial angular acceleration
     * Posef: final pose
     * linVelf: vector of final linear velocities
     * linAccf: vector of final linear accelerations
     * angVelf: vector of final angular velocities
     * angAccf: vector of final angular acceleration
     * ti: start time
     * tf: end time
     * ts: sampling time
     * pose_traj: vector of output poses
     * dpos: vector of output linear velocities
     * ddpos: vector of output linear accelerations
     * w: vector of output angular velocities
     * dw: vector of output angular accelerations
     * time: vector of output time
     */
    static void posePolynomInterp(Pose posei, Vector3d linVeli, Vector3d linAcci, Vector3d angVeli, Vector3d angAcci,
                                  Pose posef, Vector3d linVelf, Vector3d linAccf, Vector3d angVelf, Vector3d angAccf,
                                  double ti, double tf, double ts,
                                  std::vector<Pose> &pose_traj, std::vector<Vector3d> &dpos, std::vector<Vector3d> &ddpos, std::vector<Vector3d> &w, std::vector<Vector3d> &dw, std::vector<double> &time);

};

#endif