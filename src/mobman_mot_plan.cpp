#include "mars_mot_plan/kinematics/MarsUR5.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include "mars_mot_plan/traj_plan/PoseTrajectory.h"
#include "matplotlibcpp.h"
#include <vector>

#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR> 
#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>

#include <stdio.h>
#include <stdlib.h>
#include <chrono>
using namespace std::chrono;

using namespace std;
using namespace Eigen;
namespace plt = matplotlibcpp;

std::vector<double> step_size_trans(std::vector<double> dx, std::vector<double> dy, std::vector<double> dz, double tf, double ts);
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pinv(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}); // choose appropriately

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mars_mot_plan");
    ros::NodeHandle nh("~");
    MarsUR5 robot;

    // Algorithm parameters
    double t0 = 0, tf = 18, ts = 1 / 50.0;
    double alpha = 8;
    double KpPos = 10, KpOr = 20;

    // Initial joint states
    double tx = 0, ty =0, phi = 0, tz=0.2;
    double q1, q2, q3, q4, q5, q6;
    q1 = 0;
    q2 = -1*M_PI/2;
    q3 = 3*M_PI/4;
    q4 = 5*M_PI/4;
    q5 = -1*M_PI/2;
    q6 = 0.0;   
    cout << "Initial joint angles: " << endl
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
    VectorXd q0(10);
    q0 << tx, ty, phi, tz, q1, q2, q3, q4, q5, q6;
    robot.setJointPositions(q0);    
    Matrix4d Te0; //A Matrix to store the Transformation matrix of the forward kinematics
    Te0 = robot.getEETransform();
    Pose pose0(Te0);
    cout << "T0:" << endl << Te0 << endl;
    cout << "Pose0:" << endl << pose0 << endl << endl;

    // Desired final pose
    Vector3d rotAxis(1, 1, -1);
    rotAxis.normalize();
    Quaterniond eigen_quatf = Quaterniond(AngleAxisd(120*M_PI/180, rotAxis));
    Pose posef(Vector3d(1.0, -0.8, 0.4), Quat(eigen_quatf));
    Matrix4d Tfe = posef.matrixRep();
    cout << "Tf:" << endl << Tfe << endl;
    cout << "Posef:" << endl << posef << endl << endl;
    
    /*
        **********************************************************
        **************** Pose trajectory planning ****************
        **********************************************************
    */
    high_resolution_clock::time_point start = high_resolution_clock::now();
    PoseTrajectory desiredTraj(pose0, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                              posef, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                              t0, tf, ts);
    cout << "TrajPlan completed" << endl;
    double N = desiredTraj.size();

    high_resolution_clock::time_point end = high_resolution_clock::now();
    
    //Compute the execution time in us
    double time_ns = (double)(duration_cast<nanoseconds>(end - start).count());
    cout << "TrajPlan number of poses: " << N << endl;
    cout << "TrajPlan execution time: " << time_ns / 1000000 << "ms" << endl;

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
    plt::show();

    /*plt::figure(4);
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
    std::vector<double> trans = step_size_trans(desiredTraj.getTrajLinVel('x'), 
                                                desiredTraj.getTrajLinVel('y'), 
                                                desiredTraj.getTrajLinVel('z'),
                                                tf, ts);
    /*plt::figure(5);
    plt::plot(desiredTraj.getTrajTime(), trans);
    plt::grid(true);
    plt::show();*/
    
    /*
        **********************************************************
        ******** Initialize motion planning variables  ***********
        **********************************************************
    */
    std::vector<VectorXd> q;    q.resize(N);    // 10 elements
    //std::vector<VectorXd> xi;   xi.resize(N);   // 7 elements
    std::vector<VectorXd> eta;  eta.resize(N);  // 9 elements
    std::vector<VectorXd> dq;   dq.resize(N);   // 10 elements
    std::vector<Pose> xi;       xi.resize(N);   
    std::vector<VectorXd> poseError; poseError.resize(N);   // 6 elements
    std::vector<double> w_measure; w_measure.resize(N);
    std::vector<double> MMmanip;  MMmanip.resize(N);
    std::vector<double> UR5manip; UR5manip.resize(N);

    // Error weighting matrix
    MatrixXd wError = MatrixXd::Zero(6, 6);
    wError.block<3,3>(0,0) = Matrix3d::Identity()*KpPos;
    wError.block<3,3>(3,3) = Matrix3d::Identity()*KpOr;
    //cout << "wError: " << endl << wError << endl;

    // Joint limits and collision avoidance variables
    MatrixXd invTq = robot.getVelsNormMatrix();
    MatrixXd Wjlim, WColElbow, WColWrist;
    std::vector<double> elbowDist; elbowDist.resize(N);
    std::vector<double> wristDist; wristDist.resize(N);
    std::vector<double> wristHeight; wristHeight.resize(N);   
    Vector3d wristPos;
    VectorXd errorFb = VectorXd::Zero(6);
    VectorXd partSol = VectorXd::Zero(6);
    VectorXd homSol = VectorXd::Zero(6);

    // Identity matrix of size delta=9, delta=M-1 =>10DOF-1
    MatrixXd Id = MatrixXd::Identity(9, 9);

    // Create non-holonomic constrains matrix
    MatrixXd S = MatrixXd::Zero(10,9);
    S.block<8, 8>(2, 1).setIdentity();
    //cout << "S: " << endl << S << endl;
    
    // Copy the initial values of the motion planning
    q.at(0) = q0;
    xi.at(0) = desiredTraj.poseAt(0);

    MatrixXd JBar, JBarW, invJBarW, Wmatrix;
    VectorXd MMdP, UR5dP, dP;
    double maxAlpha, minAlpha;
    start = high_resolution_clock::now();
    int k;
    for (k = 0; k < N; k++)
    {
        // Set the current joint positions to the robot
        robot.setJointPositions(q.at(k));

        // Get the kth pose
        xi.at(k) = Pose::matrixToPose(robot.getEETransform());

        //cout << "k:" << k + 1 << endl;
        // Replace the elements of the nonholonomic constraints matrix
        S(0, 0) = cos(q.at(k)(2));  S(1, 0) = sin(q.at(k)(2));

        // Compute the manipulability gradients
        robot.getManipGrads(MMdP, MMmanip.at(k), UR5dP, UR5manip.at(k));
        dP =  UR5manip.at(k)*MMdP + MMmanip.at(k)*UR5dP;
        dP =  S.transpose()*dP;
        w_measure.at(k) = MMmanip.at(k) * UR5manip.at(k);

        // Compute the weighting matrices
        robot.getJLimWeight(Wjlim);
        robot.getElbowColWeight(0.001, 50, 1, WColElbow, elbowDist.at(k));
        robot.getWristColWeight(0.001, 50, 1, WColWrist, wristDist.at(k), wristPos);
        wristHeight.at(k) = wristPos(2);
        Wmatrix = Wjlim * WColElbow * WColWrist * invTq;
        /*cout << "Wjlim: " << endl << Wjlim << endl;
        cout << "invTq: " << endl << invTq << endl;        
        cout << "Wmatrix: " << endl << Wmatrix << endl;*/

        // Compute the position and orientation error
        //cout << "pose_des: " << endl << desiredTraj.poseAt(k) << endl;
        poseError.at(k) = Pose::pose_diff(desiredTraj.poseAt(k), xi.at(k));

        // Compute the particular and homogeneous solutions
        // particular solution
        JBar = robot.getJacobianBar();
        JBarW = JBar * Wmatrix;
        invJBarW = pinv(JBarW);
        partSol = invJBarW*(desiredTraj.velocitiesAt(k) + wError*poseError.at(k));
        partSol = Wmatrix*partSol;

        // homogeneous solution
        homSol = trans.at(k) * (Id - invJBarW*JBarW)*Wmatrix * dP;
        homSol = Wmatrix*homSol;

        //cout << "poseError:" << endl << poseError.at(k).transpose() << endl;
        //cout << "partSol:" << endl << partSol.transpose() << endl;
        //cout << "homSol:" << endl << homSol.transpose() << endl;        

        // Compute the maximum and minimum step size for the homogeneous solution
        robot.getSelfMotionLims(partSol, homSol, maxAlpha, minAlpha);
        if (minAlpha < maxAlpha)    
        {
            cout << "Could not acheive task that complies with joint velocity limits";
            break;
        }
        // Saturate alpha in case is out of limits
        if (alpha > maxAlpha) alpha = maxAlpha;
        if (alpha < minAlpha) alpha = minAlpha;

        // Compute the joint velocities
        eta.at(k) = partSol + alpha * homSol;
        dq.at(k) = S * eta.at(k);
        //cout << "eta:" << endl << eta.at(k).transpose() << endl;
        // << "dq:" << endl << dq.at(k).transpose() << endl;

        // Update joint positions for next iteration
        if (k < (N - 1))
        {
            q.at(k+1) = q.at(k) + dq.at(k)*ts;
            //cout << "q(k+1):" << endl << q.at(k+1).transpose() << endl;
        }
        //getchar();
    }
    end = high_resolution_clock::now();
    //Compute the execution time in us
    time_ns = (double)(duration_cast<nanoseconds>(end - start).count());
    cout << "Motion planning completed. Number of iterations: " << k << endl;
    cout << "Execution time: " << time_ns/1000000 << "ms" << endl;
    cout << "Mean time: " << time_ns/(1000*k) << "us" << endl;

    k = k - 1; //Final iteration
    // Show final position and orientation errors
    cout << "Final pos error: " << endl << poseError.at(k).head(3).transpose() << endl;
    cout << "Pos error norm: " << endl << poseError.at(k).head(3).norm() << endl;

    cout << "Final orientation error: " << endl << poseError.at(k).tail(3).transpose() << endl;
    cout << "Orientation error norm: " << endl << poseError.at(k).tail(3).norm() << endl;

    // Plot obtained x
    /*std::vector<double> posErrorX, posErrorY, posErrorZ;
    for (int k = 0; k < N; k++)
    {
        //x_traj.push_back(xi.at(k).position(0));
        posErrorX.push_back(poseError.at(k)(0));
        posErrorY.push_back(poseError.at(k)(1));
        posErrorZ.push_back(poseError.at(k)(2));
    }
    plt::figure(1);
    plt::plot(desiredTraj.getTrajTime(), posErrorX);
    plt::plot(desiredTraj.getTrajTime(), posErrorY);
    plt::plot(desiredTraj.getTrajTime(), posErrorZ);
    plt::grid(true);
    plt::show();   */


    return 0;
}

std::vector<double> step_size_trans(std::vector<double> dx, std::vector<double> dy, std::vector<double> dz, double tf, double ts)
{
    Vector3d maxdxi;
    double dx_max = 0.0, dy_max = 0.0, dz_max = 0.0;
    // Get the maximum of the absolute values of each vector
    for (uint i = 0; i < dx.size(); i++)
    {
        if (dx.at(i) < 0)
            dx.at(i) *= -1;
        if (dx_max < dx.at(i))
            dx_max = dx.at(i);
        if (dy.at(i) < 0)
            dy.at(i) *= -1;
        if (dy_max < dy.at(i))
            dy_max = dy.at(i);
        if (dz.at(i) < 0)
            dz.at(i) *= -1;
        if (dz_max < dz.at(i))
            dz_max = dz.at(i);
    }
    if (dx_max < 1e-5)
        dx_max = 1;
    if (dy_max < 1e-5)
        dy_max = 1;
    if (dz_max < 1e-5)
        dz_max = 1;
    maxdxi << dx_max, dy_max, dz_max;

    double N = dx.size();
    double tb = tf * (1 - 0.15);
    double T = tf - tb;
    double a0 = 1, a1 = 0, a2 = 0, a3 = -10 / pow(T, 3), a4 = 15 / pow(T, 4), a5 = -6 / pow(T, 5);
    std::vector<double> trans;
    std::vector<double> aux;
    std::vector<double>::iterator aux_it;
    double delta_time;
    int k;
    double t;
    for (k = 0, t = 0; k < N; k++, t += ts)
    {
        if (k > (N / 2))
        {
            if (k > (N * (1 - 0.15)))
            {
                delta_time = t - tb;
                trans.push_back(a0 + a1 * (delta_time) + a2 * pow(delta_time, 2) + a3 * pow(delta_time, 3) + a4 * pow(delta_time, 4) + a5 * pow(delta_time, 5));
            }
            else
            {
                trans.push_back(1.0);
            }
        }
        else
        {
            aux.push_back(dx.at(k) / dx_max);
            aux.push_back(dy.at(k) / dy_max);
            aux.push_back(dz.at(k) / dz_max);
            aux_it = std::max_element(aux.begin(), aux.end());
            trans.push_back(*aux_it);
            aux.clear();
        }
    }
    return trans;
}

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pinv(const MatT &mat, typename MatT::Scalar tolerance) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}