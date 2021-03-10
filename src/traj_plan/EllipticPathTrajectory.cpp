#include "mars_mot_plan/traj_plan/EllipticPathTrajectory.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include <iostream>
using namespace std;
using namespace Eigen;

EllipticPathTrajectory::EllipticPathTrajectory()
:Trajectory(0, 0)
{
    a =0; b=0; thi=0; thf=0; m_z=0, zi=0;
    
    orientCoeff.clear();
};

// Computes the coefficients for the trajectory
EllipticPathTrajectory::EllipticPathTrajectory(Pose posei, Pose posef, double ti, double tf)
:Trajectory(ti, tf)
{
    this->posei = posei; this->posef = posef;

    Vector2d p1 = posei.getPos().head(2);
    Vector2d p2 = posef.getPos().head(2);
    zi = posei.getPos()(2);
    double zf = posef.getPos()(2);

    // Define the center as the closest point to the origin
    Vector2d q = p2 - p1; 
    Vector2d c1, c2;
    c1 = p1 + Vector2d(q(0), 0);
    c2 = p1 + Vector2d(0, q(1));
    if (c1.norm() < c2.norm())
    {
        center = c1;
    }
    else
    {
        center = c2;
    }

    // Find the angles of p1 and p2 inside the ellipse
    Vector2d v1 = p1 - center;
    Vector2d v2 = p2 - center;
    thi = atan2(v1(1), v1(0));
    thf = atan2(v2(1), v2(0));

    // Make sure the shortest distance is being used
    double diff = realmod(thf-thi+M_PI,2*M_PI)-M_PI;
    thf = thi + diff;

    // Find the radii
    double th_a = fabs(cos(thi));
    double th_b = fabs(sin(thi));

    if (th_a > 0.001)
        a = (p1(0)-center(0))/cos(thi);
    else
        a = (p2(0)-center(0))/cos(thf);

    if (th_b > 0.001)
        b = (p1(1)-center(1))/sin(thi);
    else
        b = (p2(1)-center(1))/sin(thf);

    // Slope for z coordinate
    m_z = (zf - zi)/(thf-thi);

    // Find the polynomial coefficients of the timing variable s

    this->sCoeff = TrajPlan::polynomCoef(thi, 0, 0, thf, 0, 0, ti, tf);
    
    // Orientation
    Quat orienti = this->posei.orientation;
    Quat orientf = this->posef.orientation;
    this->orientCoeff = TrajPlan::quatPolynomCoef(orienti, Vector3d::Zero(), Vector3d::Zero(), orientf, Vector3d::Zero(), Vector3d::Zero(), ti, tf);
}

double EllipticPathTrajectory::getSAtTime(double t)
{
    return TrajPlan::qPol(sCoeff, t);
}

double EllipticPathTrajectory::getDsAtTime(double t)
{
    return TrajPlan::dqPol(sCoeff, t);
}

Vector3d EllipticPathTrajectory::trajPos(double t)
{
    s = getSAtTime(t);
    Vector3d pos;
    pos(0) = a*cos(s) + center(0);
    pos(1) = b*sin(s) + center(1);
    pos(2) = m_z*(s-thi) + zi;
    return pos;
}

Vector3d EllipticPathTrajectory::trajLinVel(double t)
{
    s = getSAtTime(t);
    ds = getDsAtTime(t);
    Vector3d vel;
    vel(0) = -a*sin(s)*ds;
    vel(1) = b*cos(s)*ds;
    vel(2) = m_z*ds;
    return vel;
}

Quat EllipticPathTrajectory::trajOrient(double t)
{
    return TrajPlan::quatPol(orientCoeff, getTi(), getTf(), t);
}

Vector3d EllipticPathTrajectory::trajAngVel(double t)
{
    return TrajPlan::wPol(orientCoeff, getTi(), getTf(), t);
}
