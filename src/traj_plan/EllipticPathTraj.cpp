#include "mars_mot_plan/traj_plan/EllipticPathTraj.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include "matplotlibcpp.h"  //Temporal for plots
#include <iostream>
using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

std::vector<double> EllipticPathTraj::getTrajPosition(char sel)
{
    switch(sel)
    {
    case 'x':
        return x;
    case 'y':
        return y;
    case 'z':
        return z;
    default:
        string errStr = "The required position: ";
        errStr.append(1, sel);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

std::vector<double> EllipticPathTraj::getTrajLinVel(char sel)
{
    switch(sel)
    {
    case 'x':
        return dx;
    case 'y':
        return dy;
    case 'z':
        return dz;
    default:
        string errStr = "The required linear velocity: ";
        errStr.append(1, sel);
        errStr.append(" does not exist.");
        throw errStr;
    }
}

std::vector<double> EllipticPathTraj::getTrajOrientation(char sel)
{
    std::vector<double> quat_part;
    quat_part.resize(n);
    switch(sel)
    {
    case 'w':
        for (int k = 0; k < n; k++)
        {
            quat_part.at(k) = orientation.at(k).w;
        }
        break;
    case 'x':
        for (int k = 0; k < n; k++)
        {
            quat_part.at(k) = orientation.at(k).x;
        }
        break;
    case 'y':
        for (int k = 0; k < n; k++)
        {
            quat_part.at(k) = orientation.at(k).y;
        }
        break;
    case 'z':
        for (int k = 0; k < n; k++)
        {
            quat_part.at(k) = orientation.at(k).z;
        }
        break;
    }
    return quat_part;
}
 
std::vector<double> EllipticPathTraj::getTrajAngVel(char sel)
{
    std::vector<double> vel;
    vel.resize(n);
    switch(sel)
    {    
    case 'x':
        for (int k = 0; k < n; k++)
        {
            vel.at(k) = w.at(k)(0);
        }
        break;
    case 'y':
        for (int k = 0; k < n; k++)
        {
            vel.at(k) = w.at(k)(1);
        }
        break;
    case 'z':
        for (int k = 0; k < n; k++)
        {
            vel.at(k) = w.at(k)(2);
        }
        break;
    }
    return vel;
}

EllipticPathTraj::EllipticPathTraj(Pose posei, Pose posef, double ti, double tf, double ts)
{
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

    // FInd the radii
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


    // Find the trajectory of the timing variable s
    TrajPlan::fithOrderInterp(thi, 0, 0, thf, 0, 0, ti, tf, ts, s, ds, dds, time);

    // Use the evolution of parameter s to generate the trajectory in each of the coordinates
    Vector3d pos, vel;
    for (int k=0; k < s.size(); k++)
    {
        // Position
        pos = trajPos(s.at(k));
        x.push_back(pos(0));
        y.push_back(pos(1));
        z.push_back(pos(2));
        // Velocity
        vel = trajVel(s.at(k), ds.at(k));
        dx.push_back(vel(0));
        dy.push_back(vel(1));
        dz.push_back(vel(2));
        // Acceleration not implemented
        ddx.push_back(0);
        ddy.push_back(0);
        ddz.push_back(0);
    }
   
    // Orientation
    TrajPlan::quatPolynomInterp(posei.orientation, Vector3d::Zero(), Vector3d::Zero(), posef.orientation, Vector3d::Zero(), Vector3d::Zero(), ti, tf, ts, orientation, w, dw, time);
    
    // Store the trajectory in the corresponding vectors
    double n = time.size();
    this->n = n;
    poses.resize(n);
    poses_vec_rep.resize(n);
    velocities.resize(n);
    accelerations.resize(n);
    for (int k=0; k<n; k++)
    {
        poses.at(k)=(Pose(this->x.at(k), this->y.at(k), this->z.at(k), this->orientation.at(k).w, this->orientation.at(k).x, this->orientation.at(k).y, this->orientation.at(k).z));
        poses_vec_rep.at(k) = poses.at(k).vecRep();
        velocities.at(k) = VectorXd(6);
        velocities.at(k) << dx.at(k), dy.at(k), dz.at(k), w.at(k)(0), w.at(k)(1), w.at(k)(2);
        accelerations.at(k) = VectorXd(6);
        accelerations.at(k) << ddx.at(k), ddy.at(k), ddz.at(k), dw.at(k)(0), dw.at(k)(1), dw.at(k)(2);
    }

    // // Plot the obtained trajectory
    // // Path plot    
    // plt::figure(1);
    // plt::subplot(2, 1, 1);
    // plt::named_plot("XY", getTrajPosition('x'), getTrajPosition('y'), "b");
    // plt::xlabel("X(m)");
    // plt::ylabel("Y(m)");
    // plt::grid(true);
    // plt::subplot(2, 1, 2);
    // plt::named_plot("Z", time, getTrajPosition('z'), "b");    
    // plt::xlabel("time(s)");
    // plt::ylabel("Z(m)");
    // plt::grid(true);

    // // Plot position and velocities
    // plt::figure(2);
    // plt::suptitle("Position");
    // plt::subplot(3, 2, 1);
    // plt::plot(time, getTrajPosition('x'));
    // plt::ylabel("x");
    // plt::xlabel("time");
    // plt::grid(true);
    // plt::subplot(3, 2, 2);
    // plt::plot(time, getTrajLinVel('x'));
    // plt::ylabel("dx");
    // plt::xlabel("time");
    // plt::grid(true);

    // plt::subplot(3, 2, 3);
    // plt::plot(time, getTrajPosition('y'));
    // plt::ylabel("y");
    // plt::xlabel("time");
    // plt::grid(true);
    // plt::subplot(3, 2, 4);
    // plt::plot(time, getTrajLinVel('y'));
    // plt::ylabel("dy");
    // plt::xlabel("time");
    // plt::grid(true);

    // plt::subplot(3, 2, 5);
    // plt::plot(time, getTrajPosition('z'));
    // plt::ylabel("z");
    // plt::xlabel("time");
    // plt::grid(true);
    // plt::subplot(3, 2, 6);
    // plt::plot(time, getTrajLinVel('z'));
    // plt::ylabel("dz");
    // plt::xlabel("time");
    // plt::grid(true);

    // // Plot orientation
    // plt::figure(3);
    // plt::suptitle("Orientation(quaternion)");
    // plt::subplot(2, 2, 1);
    // plt::plot(time, getTrajOrientation('w'));
    // plt::ylabel("w");
    // plt::grid(true);
    // plt::subplot(2, 2, 2);
    // plt::plot(time, getTrajOrientation('x'));
    // plt::ylabel("x");
    // plt::grid(true);
    // plt::subplot(2, 2, 3);
    // plt::plot(time, getTrajOrientation('y'));
    // plt::ylabel("y");
    // plt::grid(true);
    // plt::subplot(2, 2, 4);
    // plt::plot(time, getTrajOrientation('z'));
    // plt::ylabel("z");
    // plt::grid(true);
    // plt::xlabel("time");
    // plt::show();
}

Vector3d EllipticPathTraj::trajPos(double s)
{
    Vector3d pos;
    pos(0) = a*cos(s) + center(0);
    pos(1) = b*sin(s) + center(1);
    pos(2) = m_z*(s-thi) + zi;
    return pos;
}

Vector3d EllipticPathTraj::trajVel(double s, double ds)
{
    Vector3d vel;
    vel(0) = -a*sin(s)*ds;
    vel(1) = b*cos(s)*ds;
    vel(2) = m_z*ds;
    return vel;
}