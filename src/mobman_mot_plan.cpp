#include "mars_mot_plan/MarsUR5.h"
#include "mars_mot_plan/PolynomInterp.h"
#include "matplotlibcpp.h"
#include <vector>

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <chrono>
using namespace std::chrono;

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mars_mot_plan");
    ros::NodeHandle nhLocal("~");
    MarsUR5 robot;

    /*
        **********************************************************
        *************** MARS Forward Kinematics ******************
        **********************************************************
    */
    /*double tx = -0.8, ty =2.1, phi = -0.26, tz = 0.1;    
    //Values of the generalized coordinates
    double q1, q2, q3, q4, q5, q6;
    q1 = 0.0 * M_PI / 180.0;
    q2 = 0.0 * M_PI / 180.0;
    q3 = 0.0 * M_PI / 180.0;
    q4 = 0.0 * M_PI / 180.0;
    q5 = 0.0 * M_PI / 180.0;
    q6 = 0.0 * M_PI / 180.0;
    q1 = 0.0;
    q2 = -1*M_PI/2;
    q3 = 3*M_PI/4;
    q4 = 5*M_PI/4;
    q5 = -1*M_PI/2;
    q6 = 0.0;
    
    cout << "Desired joint angles: " << endl
         << "tx: " << tx << endl
         << "ty: " << ty << endl
         << "phi: " << phi * 180 / M_PI << endl
         << "tz: " << tz << endl
         << "q1: " << q1 * 180 / M_PI << endl
         << "q2: " << q2 * 180 / M_PI << endl
         << "q3: " << q3 * 180 / M_PI << endl
         << "q4: " << q4 * 180 / M_PI << endl
         << "q5: " << q5 * 180 / M_PI << endl
         << "q6: " << q6 * 180 / M_PI << endl;

    //A vector to store the joint angles before sending them to the algorithm
    VectorXd q(10);
    q << tx, ty, phi, tz, q1, q2, q3, q4, q5, q6;

    int n = 1;
    Matrix4d T0e; //A Matrix to store the Transformation matrix of the forward kinematics
    high_resolution_clock::time_point start = high_resolution_clock::now();
    for (int i = 0; i < n; i++)
    {
        //Calculate the forward kinematics
        T0e = robot.forwardKin(q);
    }
    high_resolution_clock::time_point end = high_resolution_clock::now();
    //Calculate the execution time in us
    double time_ns = (double)(duration_cast<nanoseconds>(end - start).count());

    cout << "Execution time: " << time_ns / 1000.0 / n << "us" << endl;
    cout << T0e << endl;*/

    /*
        **********************************************************
        ************* Position trajectory planning ***************
        **********************************************************
    */
    
    /*high_resolution_clock::time_point start = high_resolution_clock::now();
    
    double x0 = 0, xf=5, t0=0, tf=10, ts=0.01;
    std::vector<double> x, dx, ddx, time;
    PolynomInterp::fithOrderInterp(x0, 0, 0, xf, 0, 0, 0, tf, ts, x, dx, ddx, time);
    high_resolution_clock::time_point end = high_resolution_clock::now();
    
    
    //Calculate the execution time in us
    double time_ns = (double)(duration_cast<nanoseconds>(end - start).count());
    cout << "Execution time: " << time_ns / 1000.0 << "us" << endl;

    //plt::figure_size(1200, 780);
    plt::subplot(3, 1, 1);
    plt::plot(time,x);
    plt::subplot(3, 1, 2);
    plt::plot(time,dx);
    plt::subplot(3, 1, 3);
    plt::plot(time,ddx);
    plt::show();*/

    /*
        **********************************************************
        ************ Orientation trajectory planning *************
        **********************************************************
    */


    high_resolution_clock::time_point start = high_resolution_clock::now();
    
    double t0=0, tf=7, ts=0.02;
    Quat q0(0.0, -0.7071, 0.7071, 0.0);
    Quat qf(0.5, 0.5, 0.5, -0.5);

    std::vector<double> time;
    std::vector<Quat> quat;
    std::vector<Vector3d> w, dw;

    PolynomInterp::quatPolynomInterp(q0, Vector3d::Zero(), Vector3d::Zero(), qf, Vector3d::Zero(), Vector3d::Zero(), t0, tf, ts, quat, w, dw, time);
    high_resolution_clock::time_point end = high_resolution_clock::now();
    
    //Calculate the execution time in us
    double time_ns = (double)(duration_cast<nanoseconds>(end - start).count());
    cout << "Execution time: " << time_ns / 1000.0 << "us" << endl;

    //Extract parts of the quaternion
    std::vector<double> quat_w, quat_x, quat_y, quat_z;
    double N = quat.size();
    std::vector<Quat>::iterator it;
    for (it = quat.begin(); it != quat.end(); it++)
    {
        quat_w.push_back((*it).w);
        quat_x.push_back((*it).x);
        quat_y.push_back((*it).y);
        quat_z.push_back((*it).z);        
    }

    cout << "size(quat): " << quat.size() << endl;
    cout << "size(quat_w): " << quat_w.size() << endl;
    cout << "size(time): " << time.size() << endl;

    //plt::figure_size(1200, 780);
    plt::subplot(2, 2, 1);
    plt::plot(time,quat_w);
    plt::grid(true);
    plt::subplot(2, 2, 2);
    plt::grid(true);
    plt::plot(time,quat_x);
    plt::subplot(2, 2, 3);
    plt::grid(true);
    plt::plot(time,quat_y);
    plt::subplot(2, 2, 4);
    plt::grid(true);
    plt::plot(time,quat_z);
    plt::show();

    return 0;
}