#include "mars_mot_plan/traj_plan/PoseIterTrajectory.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include "matplotlibcpp.h" //Temporal for plots
#include <iostream>
using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

// Computes the coefficients for the trajectory
PoseIterTrajectory::PoseIterTrajectory(Pose posei, Vector3d linVeli, Vector3d linAcci, Vector3d angVeli, Vector3d angAcci,
                                       Pose posef, Vector3d linVelf, Vector3d linAccf, Vector3d angVelf, Vector3d angAccf,
                                       double ti, double tf)
{
    this-> ti = ti; this->tf = tf; this->posei = posei; this->posef = posef;

    // Compute coefficients
    // Position
    Vector3d posi = this->posei.getPos();
    Vector3d posf = this->posef.getPos();
    // x
    this->posXCoeff = TrajPlan::polynomCoef(posi(0), linVeli(0), linAcci(0), posf(0), linVelf(0), linAccf(0), ti, tf);
    /*for (int i = 0; i < posXCoeff.size(); i++)
    {
        cout << "posXCoeff(" << i << "): " << posXCoeff.at(i) << endl;
    }*/
    // y
    this->posYCoeff = TrajPlan::polynomCoef(posi(1), linVeli(1), linAcci(1), posf(1), linVelf(1), linAccf(1), ti, tf);
    // z
    this->posZCoeff = TrajPlan::polynomCoef(posi(2), linVeli(2), linAcci(2), posf(2), linVelf(2), linAccf(2), ti, tf);
    
    // Orientation
    Quat orienti = this->posei.orientation;
    Quat orientf = this->posef.orientation;
    this->orientCoeff = TrajPlan::quatPolynomCoef(orienti, angVeli, angAcci, orientf, angVelf, angAccf, ti, tf);
}

void PoseIterTrajectory::validateTime(double t)
{
    if (t < ti || t > tf)
    {
        throw "requested time outside of trajectory period";
    }    
}

std::vector<double> PoseIterTrajectory::getPosCoeff(char coord)
{
    switch (coord)
    {
    case 'x':
        return posXCoeff;
    case 'y':
        return posYCoeff;
    case 'z':
        return posZCoeff;
    default:
        string errStr = "The required position coordinate ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

double PoseIterTrajectory::getPos(char coord, double t)
{
    validateTime(t);    
    switch (coord)
    {
    case 'x':
        return TrajPlan::qPol(posXCoeff, t);
    case 'y':
        return TrajPlan::qPol(posYCoeff, t);        
    case 'z':
        return TrajPlan::qPol(posZCoeff, t);
    default:
        string errStr = "The required position coordinate ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

double PoseIterTrajectory::getLinVel(char coord, double t)
{
    validateTime(t);    
    switch (coord)
    {
    case 'x':
        return TrajPlan::dqPol(posXCoeff, t);
    case 'y':
        return TrajPlan::dqPol(posYCoeff, t);
    case 'z':
        return TrajPlan::dqPol(posZCoeff, t);
    default:
        string errStr = "The required velocity ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

double PoseIterTrajectory::getLinAcc(char coord, double t)
{
    validateTime(t);    
    switch (coord)
    {
    case 'x':
        return TrajPlan::ddqPol(posXCoeff, t);
    case 'y':
        return TrajPlan::ddqPol(posYCoeff, t);
    case 'z':
        return TrajPlan::ddqPol(posZCoeff, t);
    default:
        string errStr = "The required acceleration coordinate ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

double PoseIterTrajectory::getMaxLinVel(char &coord)
{
    double maxt;
    double maxdx, maxdy, maxdz, maxVel;
    double R;
    double a, b, c, d;

    a = 20 * posXCoeff.at(5);
    b = 12 * posXCoeff.at(4);
    c = 6 * posXCoeff.at(3);
    d = 2 * posXCoeff.at(2);

    //R = (9*a*b*c - 27*a*a*d-2*b*b*b)/(54*a*a*a);    
    /*cout << "R: " << R << endl;
    cout << "S: " << S << endl;*/
    maxt = - b/(3*a);
    maxdx = abs(TrajPlan::dqPol(posXCoeff, maxt));
    maxdy = abs(TrajPlan::dqPol(posYCoeff, maxt));
    maxdz = abs(TrajPlan::dqPol(posZCoeff, maxt));

    if (maxdx > maxdy)
    {
        if (maxdx > maxdz)
        {
            coord = 'x';
            maxVel = maxdx;
        }
        else
        {
            coord = 'z';
            maxVel = maxdz;
        }        
    }
    else
    {
        if (maxdy > maxdz)
        {
            coord = 'y';
            maxVel = maxdy;
        }
        else
        {
            coord = 'z';
            maxVel = maxdz;
        }        
    }
    return maxVel;
}

double PoseIterTrajectory::getOrientPart(char part, double t)
{
    validateTime(t);
    Quat Q = TrajPlan::quatPol(orientCoeff, ti, tf, t);
    double quatPart;
    switch(part)
    {
    case 'w':
        quatPart = Q.w;
        break;
    case 'x':
        quatPart = Q.x;
        break;
    case 'y':
        quatPart = Q.y;
        break;
    case 'z':
        quatPart = Q.z;
        break;
    default:
        string errStr = "Quaternion does not contain element ";
        errStr.append(1, part);
        errStr.append(".");
        throw errStr;
    }
    return quatPart;
}

Quat PoseIterTrajectory::getOrient(double t)
{
    validateTime(t);
    return TrajPlan::quatPol(orientCoeff, ti, tf, t);
}

double PoseIterTrajectory::getAngVel(char coord, double t)
{
    validateTime(t);
    Vector3d w = TrajPlan::wPol(orientCoeff, ti, tf, t);
    switch (coord)
    {
    case 'x':
        return w(0);
    case 'y':
        return w(1);
    case 'z':
        return w(2);
    default:
        string errStr = "The required position coordinate ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

double PoseIterTrajectory::getAngAcc(char coord, double t)
{
    validateTime(t);
    Vector3d dw = TrajPlan::dwPol(orientCoeff, ti, tf, t);
    switch (coord)
    {
    case 'x':
        return dw(0);
    case 'y':
        return dw(1);
    case 'z':
        return dw(2);
    default:
        string errStr = "The required position coordinate ";
        errStr.append(1, coord);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

Pose PoseIterTrajectory::getPose(double t)
{
    Vector3d pos;
    pos(0) = TrajPlan::qPol(posXCoeff, t);
    pos(1) = TrajPlan::qPol(posYCoeff, t);
    pos(2) = TrajPlan::qPol(posZCoeff, t);
    return Pose(pos, TrajPlan::quatPol(orientCoeff, ti, tf, t));
}

VectorXd PoseIterTrajectory::getPoseVec(double t)
{
    return getPose(t).vecRep();
}

VectorXd PoseIterTrajectory::getVel(double t)
{
    VectorXd vel(6);
    vel(0) = TrajPlan::dqPol(posXCoeff, t);
    vel(1) = TrajPlan::dqPol(posYCoeff, t);
    vel(2) = TrajPlan::dqPol(posZCoeff, t);
    vel.tail(3) = TrajPlan::wPol(orientCoeff, ti, tf, t);
    return vel;
}

