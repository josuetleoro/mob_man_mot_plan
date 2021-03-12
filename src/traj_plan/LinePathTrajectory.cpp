#include "mars_mot_plan/traj_plan/LinePathTrajectory.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include <iostream>
using namespace std;
using namespace Eigen;

LinePathTrajectory::LinePathTrajectory()
:Trajectory(0, 0)
{
    posXCoeff.clear();
    posYCoeff.clear();
    posZCoeff.clear();
    orientCoeff.clear();
};

// Computes the coefficients for the trajectory
LinePathTrajectory::LinePathTrajectory(Pose posei, Vector3d linVeli, Vector3d linAcci, Vector3d angVeli, Vector3d angAcci,
                   Pose posef, Vector3d linVelf, Vector3d linAccf, Vector3d angVelf, Vector3d angAccf,
                   double ti, double tf)
:Trajectory(ti, tf)
{
    this->posei = posei; this->posef = posef;

    // Compute coefficients
    // Position
    Vector3d posi = this->posei.getPos();
    Vector3d posf = this->posef.getPos();
    // x
    this->posXCoeff = TrajPlan::polynomCoef(posi(0), linVeli(0), linAcci(0), posf(0), linVelf(0), linAccf(0), ti, tf);
    // y
    this->posYCoeff = TrajPlan::polynomCoef(posi(1), linVeli(1), linAcci(1), posf(1), linVelf(1), linAccf(1), ti, tf);
    // z
    this->posZCoeff = TrajPlan::polynomCoef(posi(2), linVeli(2), linAcci(2), posf(2), linVelf(2), linAccf(2), ti, tf);
    
    // Orientation
    Quat orienti = this->posei.orientation;
    Quat orientf = this->posef.orientation;
    this->orientCoeff = TrajPlan::quatPolynomCoef(orienti, angVeli, angAcci, orientf, angVelf, angAccf, ti, tf);
}

Vector3d LinePathTrajectory::trajPos(double t)
{
    if (t > getTf())
    {
        return posef.getPos();
    }

    Vector3d pos;
    pos(0) = TrajPlan::qPol(posXCoeff, t);
    pos(1) = TrajPlan::qPol(posYCoeff, t);
    pos(2) = TrajPlan::qPol(posZCoeff, t);
    cout << "pos: " << pos.transpose() << endl;
    return pos;
}

Vector3d LinePathTrajectory::trajLinVel(double t)
{
    if (t > getTf())
    {
        return Vector3d::Zero();
    }

    Vector3d vel;
    vel(0) = TrajPlan::dqPol(posXCoeff, t);
    vel(1) = TrajPlan::dqPol(posYCoeff, t);
    vel(2) = TrajPlan::dqPol(posZCoeff, t);
    cout << "vel: " << vel.transpose() << endl;
    return vel;
}

Quat LinePathTrajectory::trajOrient(double t)
{
    if (t > getTf())
    {
        return posef.getOrientation();
    }

    return TrajPlan::quatPol(orientCoeff, getTi(), getTf(), t);
}

Vector3d LinePathTrajectory::trajAngVel(double t)
{
    if (t > getTf())
    {
        return Vector3d::Zero();
    }

    return TrajPlan::wPol(orientCoeff, getTf(), getTf(), t);
}
