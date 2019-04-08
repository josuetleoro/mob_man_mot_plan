#include "mars_mot_plan/kinematics/MarsUR5.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include "mars_mot_plan/traj_plan/PoseTrajectory.h"
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
    ros::NodeHandle nh("~");
    MarsUR5 robot;

    /*
        **********************************************************
        *************** MARS Forward Kinematics ******************
        **********************************************************
    */
    /*double tx = -0.8, ty =2.1, phi = -0.26, tz = 0.1;    
    //Values of the generalized coordinates
    double q1, q2, q3, q4, q5, q6;
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

    //A vector to store the joint angles before setting them in the MarsUR5 object
    VectorXd q(10);
    q << tx, ty, phi, tz, q1, q2, q3, q4, q5, q6;
    robot.setJointPositions(q);

    int n = 1000;
    Matrix4d T0e; //A Matrix to store the Transformation matrix of the forward kinematics
    high_resolution_clock::time_point start = high_resolution_clock::now();
    for (int i = 0; i < n; i++)
    {
        //Calculate the forward kinematics
        T0e = robot.getEETransform();
    }
    high_resolution_clock::time_point end = high_resolution_clock::now();
    //Calculate the execution time in us
    double time_ns = (double)(duration_cast<nanoseconds>(end - start).count());

    cout << "Execution time: " << time_ns / 1000.0 / n << "us" << endl;
    cout << T0e << endl;*/
    
    /*
        **********************************************************
        **************** Pose trajectory planning ****************
        **********************************************************
    */

    high_resolution_clock::time_point start = high_resolution_clock::now();
    
    Pose pose0(Vector3d(0.3830, 0.1091, 0.9137),Quat(0,-0.7071,0.7071,0));
    Pose posef(Vector3d(1.0, -0.8, 0.4),Quat(0.5,0.5,0.5,-0.5));    
    double t0=0, tf=18, ts=0.02; 
    PoseTrajectory desiredTraj(pose0, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                              posef, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                              t0, tf, ts);
    double N = desiredTraj.size();

    high_resolution_clock::time_point end = high_resolution_clock::now();
    
    //Calculate the execution time in us
    double time_ns = (double)(duration_cast<nanoseconds>(end - start).count());
    cout << "TrajPlan execution time: " << time_ns / 1000 << "us" << endl;

    //Plot position and linear velocities
    /*plt::figure(1);
    plt::suptitle("Position");
    plt::subplot(3, 1, 1);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajPosition('x'));
    plt::ylabel("x");
    plt::grid(true);
    plt::subplot(3, 1, 2);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajPosition('y'));
    plt::ylabel("y");
    plt::grid(true);
    plt::subplot(3, 1, 3);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajPosition('z'));    
    plt::ylabel("z");
    plt::grid(true);
    plt::xlabel("time");

    plt::figure(2);
    plt::suptitle("Linear velocities");
    plt::subplot(3, 1, 1);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajLinVel('x'));
    plt::ylabel("dx");
    plt::grid(true);
    plt::subplot(3, 1, 2);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajLinVel('y'));
    plt::ylabel("dy");
    plt::grid(true);
    plt::subplot(3, 1, 3);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajLinVel('z'));
    plt::ylabel("dz");
    plt::grid(true);
    plt::xlabel("time");

    //Plot orientation and angular velocities
    plt::figure(3);
    plt::suptitle("Orientation(quaternion)");
    plt::subplot(2, 2, 1);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajOrientation('w'));
    plt::ylabel("w");
    plt::grid(true);
    plt::subplot(2, 2, 2);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajOrientation('x'));
    plt::ylabel("x");
    plt::grid(true);
    plt::subplot(2, 2, 3);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajOrientation('y'));
    plt::ylabel("y");
    plt::grid(true);
    plt::subplot(2, 2, 4);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajOrientation('z'));
    plt::ylabel("z");
    plt::grid(true);
    plt::xlabel("time");

    plt::figure(4);
    plt::suptitle("Angular velocities");
    plt::subplot(3, 1, 1);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajAngVel('x'));
    plt::ylabel("wx");
    plt::grid(true);
    plt::subplot(3, 1, 2);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajAngVel('y'));
    plt::ylabel("wy");
    plt::grid(true);
    plt::subplot(3, 1, 3);
    plt::plot(desiredTraj.getTrajTime(),desiredTraj.getTrajAngVel('z'));
    plt::ylabel("wz");
    plt::grid(true);
    plt::xlabel("time");
    plt::show();*/

    /*
        **********************************************************
        **************** Step size variation law  ****************
        **********************************************************
    */

    /*Vector3d maxdxi;
    std::vector<double> dx_abs = desiredTraj.getTrajLinVel('x');
    std::vector<double> dy_abs = desiredTraj.getTrajLinVel('y');
    std::vector<double> dz_abs = desiredTraj.getTrajLinVel('z');
    double dx_max = 0.0, dy_max = 0.0, dz_max=0.0;
    for (int i=0; i<dx_abs.size(); i++)
    {
        if (dx_abs.at(i) < 0) dx_abs.at(i) *= -1;
        if (dx_max < dx_abs.at(i)) dx_max = dx_abs.at(i);
        if (dy_abs.at(i) < 0) dy_abs.at(i) *= -1;
        if (dy_max < dy_abs.at(i)) dy_max = dy_abs.at(i);
        if (dz_abs.at(i) < 0) dz_abs.at(i) *= -1;
        if (dz_max < dz_abs.at(i)) dz_max = dz_abs.at(i);

    }
    if (dx_max < 1e-5) dx_max = 1;
    if (dy_max < 1e-5) dy_max = 1;
    if (dz_max < 1e-5) dz_max = 1;
    maxdxi << dx_max , dy_max , dz_max;

    double tb = tf*(1-0.15);
    double T = tf - tb;
    double a0=1, a1=0, a2=0, a3=-10/pow(T,3), a4=15/pow(T,4), a5=-6/pow(T,5);
    std::vector<double> step_size_trans;
    std::vector<double> aux;
    std::vector<double>::iterator aux_it;
    double delta_time;
    {
        int k;
        double t;
        for (k=0, t=0; k < N; k++, t+=ts)
        {
            if (k > (N/2))
            {
                if (k > (N*(1-0.15)))
                {
                    delta_time = t - tb;
                    step_size_trans.push_back(a0+a1*(delta_time)+a2*pow(delta_time,2)+a3*pow(delta_time,3)+a4*pow(delta_time,4)+a5*pow(delta_time,5));
                }
                else
                {
                    step_size_trans.push_back(1.0);
                }
            }
            else
            {
                aux.push_back(dx_abs.at(k)/dx_max);
                aux.push_back(dy_abs.at(k)/dy_max);
                aux.push_back(dz_abs.at(k)/dz_max);
                aux_it = std::max_element(aux.begin(), aux.end());
                step_size_trans.push_back(*aux_it);
                aux.clear();                
            }
        }
    }
    plt::figure(5);
    plt::plot(desiredTraj.getTrajTime(),step_size_trans);
    plt::grid(true);
    plt::show();*/


    //Tests for Jacobian and Jacobian derivative
    double tx = 0, ty =0, phi = 0, tz = 0.2;
    double q1, q2, q3, q4, q5, q6;
    q1 = 0;
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

    //A vector to store the joint angles before setting them in the MarsUR5 object
    VectorXd q0(10);
    q0 << tx, ty, phi, tz, q1, q2, q3, q4, q5, q6;
    MatrixXd JBar;
    Matrix4d T0e;
    VectorXd MM_dP, UR5_dP;
    double MM_manip, UR5_manip;

    start = high_resolution_clock::now();
    for (int k = 0; k < N; k++)
    {
        robot.setJointPositions(q0);
        T0e = robot.getEETransform();
        //cout << "T0e:" << endl << T0e << endl;
        JBar = robot.getJacobianBar();
        //cout << "JBar:" << endl << JBar << endl;
        robot.getManipGradients(MM_dP, MM_manip, UR5_dP, UR5_manip);
        /*cout << "MM_dP:" << endl << MM_dP.transpose() << endl;
        cout << "MM_manip:" << endl << MM_manip << endl;
        cout << "UR5_dP:" << endl << UR5_dP.transpose() << endl;
        cout << "UR5_manip:" << endl << UR5_manip << endl;*/

        //getchar();
    }
    end = high_resolution_clock::now();    
    
    //Calculate the execution time in us
    time_ns = (double)(duration_cast<nanoseconds>(end - start).count());
    cout << "Number of iterations: " << N << endl;
    cout << "Execution time: " << time_ns/1000 << "us" << endl;
    cout << "Mean time: " << time_ns/(1000*N) << "us" << endl;


    return 0;
}