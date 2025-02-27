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

std::vector<double> stepSizeTrans(std::vector<double> dx, std::vector<double> dy, std::vector<double> dz, double tf, double ts);
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pinv(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}); // choose appropriately

void testPointsMaxLinVel(int testN, VectorXd &q, Pose &posef, double &tf);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mars_mot_plan");
    ros::NodeHandle nh("~");
    MarsUR5 robot;

    int testN;
    if (!nh.hasParam("testN"))
    {
        std::cout << "Parameter testN not found. Please set node testN parameter." << endl;
        return -1;
    }
    nh.getParam("testN", testN);

    // Algorithm parameters
    double t0 = 0, ts = 1 / 50.0;
    double alpha = 8;
    double KpPos = 10, KpOr = 20;

    // Get the initial joint states and desired final pose
    VectorXd q0;
    Pose posef;
    double tf;
    testPointsMaxLinVel(testN, q0, posef, tf);    
    cout << "Initial joint angles: " << endl
         << "tx: " << q0(0) << endl
         << "ty: " << q0(1) << endl
         << "phi: " << q0(2) * 180 / M_PI << endl
         << "tz: " << q0(3) << endl
         << "q1: " << q0(4) * 180 / M_PI << endl
         << "q2: " << q0(5) * 180 / M_PI << endl
         << "q3: " << q0(6) * 180 / M_PI << endl
         << "q4: " << q0(7) * 180 / M_PI << endl
         << "q5: " << q0(8) * 180 / M_PI << endl
         << "q6: " << q0(9) * 180 / M_PI << endl << endl;
    robot.setJointPositions(q0);    
    Pose pose0(robot.getEETransform());
    cout << "T0:" << endl << pose0.matrixRep() << endl;
    cout << "Pose0:" << endl << pose0 << endl << endl;
    cout << "Tf:" << endl << posef.matrixRep() << endl;
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

    /*
        **********************************************************
        **************** Step size variation law  ****************
        **********************************************************
    */
    std::vector<double> trans = stepSizeTrans(desiredTraj.getTrajLinVel('x'), 
                                                desiredTraj.getTrajLinVel('y'), 
                                                desiredTraj.getTrajLinVel('z'),
                                                tf, ts);
    /*plt::figure(5);
    plt::plot(desiredTraj.getTrajTime(), trans);
    plt::grid(true);
    plt::show();*/
    
    /*
        **********************************************************
        ********************* Motion planning  *******************
        **********************************************************
    */

    // Initialize all the variables
    std::vector<VectorXd> q;    q.resize(N);    // 10 elements
    //std::vector<VectorXd> xi;   xi.resize(N);   // 7 elements
    std::vector<VectorXd> eta;  eta.resize(N);  // 9 elements
    std::vector<VectorXd> dq;   dq.resize(N);   // 10 elements
    std::vector<Pose> xi;       xi.resize(N);   
    std::vector<VectorXd> poseError; poseError.resize(N);   // 6 elements
    std::vector<double> wMeasure; wMeasure.resize(N);
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
    VectorXd partSol = VectorXd::Zero(9);
    VectorXd homSol = VectorXd::Zero(9);

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
    double MMman, UR5man, alphak, maxAlpha, minAlpha;
    start = high_resolution_clock::now();
    int k;
    for (k = 0; k < N; k++)
    {
        // Set the current joint positions to the robot
        robot.setJointPositions(q.at(k));

        // Get the kth pose
        xi.at(k) = Pose::matrixToPose(robot.getEETransform());

        // Replace the elements of the nonholonomic constraints matrix
        S(0, 0) = cos(q.at(k)(2));  S(1, 0) = sin(q.at(k)(2));

        // Compute the manipulability gradients
        robot.getManipGrads(MMdP, MMman, UR5dP, UR5man);
        dP =  UR5man*MMdP + MMman*UR5dP;
        dP =  S.transpose()*dP;
        wMeasure.at(k) = MMman * UR5man;
        MMmanip.at(k) = MMman; UR5manip.at(k) = UR5man;

        //cout << "step: " << k + 1 << endl;
        /*cout << "MMman: " << MMman << endl;
        cout << "UR5man: " << UR5man << endl;*/

        // Compute the weighting matrices
        robot.getJLimWeight(Wjlim);
        robot.getElbowColWeight(0.001, 50, 1, WColElbow, elbowDist.at(k));
        robot.getWristColWeight(0.001, 50, 1, WColWrist, wristDist.at(k), wristPos);
        wristHeight.at(k) = wristPos(2);
        Wmatrix = Wjlim * WColElbow * WColWrist * invTq;
        //cout << "Wjlim: " << endl << Wjlim << endl;
        //cout << "Wmatrix: " << endl << Wmatrix << endl;
        //getchar();

        /*cout << "WColElbow: " << endl << WColElbow << endl;
        cout << "WColWrist: " << endl << WColWrist << endl;
        cout << "Wmatrix: " << endl << Wmatrix << endl;
        cout << "pose_des: " << endl << desiredTraj.poseAt(k) << endl;*/

        // Compute the position and orientation error        
        poseError.at(k) = Pose::pose_diff(desiredTraj.poseAt(k), xi.at(k));

        // Compute the particular and homogeneous solutions
        // particular solution
        JBar = robot.getJacobianBar();
        JBarW = JBar * Wmatrix;
        invJBarW = pinv(JBarW);
        partSol = invJBarW*(desiredTraj.velocitiesAt(k) + wError*poseError.at(k));
        partSol = Wmatrix*partSol;

        // homogeneous solution
        homSol = trans.at(k) * (Id - invJBarW*JBarW) * Wmatrix * dP;
        homSol = Wmatrix*homSol;

        /*cout << "poseError:" << endl << poseError.at(k).transpose() << endl;
        cout << "partSol:" << endl << partSol.transpose() << endl;
        cout << "homSol:" << endl << homSol.transpose() << endl;  */      

        // Compute the maximum and minimum step size for the homogeneous solution
        robot.getSelfMotionLims(partSol, homSol, maxAlpha, minAlpha);
        if (maxAlpha < minAlpha)
        {
            cout << "\033[1;31mCould not acheive task that complies with joint velocity limits\033[0m" << endl;
            break;
        }
        // Saturate alpha in case is out of limits
        alphak = alpha;
        if (alphak > maxAlpha)
        {
            cout << "\033[1;33mPotential velocity limit reached. Saturating step size.\033[0m" << endl;
            alphak = maxAlpha;
        }
        if (alphak < minAlpha)
        {
            cout << "\033[1;33mPotential velocity limit reached. Saturating step size.\033[0m" << endl;
            alphak = minAlpha;
        }

        // Compute the joint velocities
        eta.at(k) = partSol + alphak * homSol;
        dq.at(k) = S * eta.at(k);
        /*if (k >= 1)
        {
            cout << "JBarW: " << endl << JBarW << endl << endl;
            cout << "partSol: " << partSol.transpose() << endl;
            cout << "homSol: " << homSol.transpose() << endl;
            cout << "alphak: " << alphak << endl;
            cout << "maxAlpha: " << maxAlpha << endl;
            cout << "minAlpha: " << minAlpha << endl;
            cout << "eta: " << eta.at(k).transpose() << endl;
            cout << "pose_des: " << endl << desiredTraj.poseAt(k) << endl;
            getchar();
        }*/


        // Update joint positions for next iteration
        if (k < (N - 1))
        {
            q.at(k+1) = q.at(k) + dq.at(k)*ts;
            //cout << "q(k+1):" << endl << q.at(k+1).transpose() << endl;
        }        
        /*cout << "eta:" << endl << eta.at(k).transpose() << endl;
        cout << "dq:" << endl << dq.at(k).transpose() << endl;*/
        //getchar();
    }
    end = high_resolution_clock::now();
    if (k == N)
    {
        // Compute the execution time in us
        time_ns = (double)(duration_cast<nanoseconds>(end - start).count());
        cout << "Motion planning completed. Number of iterations: " << k << endl;
        cout << "Motion planning execution time: " << time_ns/1000000 << "ms" << endl;
        cout << "Motion planning mean time: " << time_ns/(1000*k) << "us" << endl;
    }
    else
    {
        cout << "\033[1;31mTask could not be completed\033[0m" << endl;
    }

    k = k - 1; // Final element for the obtained data 
    // Show final position and orientation errors
    cout << "Desired final pos: " << endl << desiredTraj.poseAt(k) << endl;
    cout << "Obtained final pos: " << endl << xi.at(k) << endl;
    cout << "Final pos error: " << endl << poseError.at(k).head(3).transpose() << endl;
    cout << "Pos error norm: " << endl << poseError.at(k).head(3).norm() << endl;
    cout << "Final orientation error: " << endl << poseError.at(k).tail(3).transpose() << endl;
    cout << "Orientation error norm: " << endl << poseError.at(k).tail(3).norm() << endl;

    /*
        **********************************************************
        ********** Plot the obtained trajectory data  ************
        **********************************************************
    */
    double NObt = k + 1;
    std::vector<double> epos_x, epos_y, epos_z, eor_x, eor_y, eor_z;
    std::vector<double> mobPlat_x, mobPlat_y, z_pj, qa1, qa2, qa3, qa4, qa5, qa6;
    std::vector<double> mobPlat_v, mobPlat_w, dz_pj, dqa1, dqa2, dqa3, dqa4, dqa5, dqa6;
    for (int i = 0; i < NObt; i++)
    {   
        // Position and orientation error data
        epos_x.push_back(poseError.at(i)(0));
        epos_y.push_back(poseError.at(i)(1));
        epos_z.push_back(poseError.at(i)(2));
        eor_x.push_back(poseError.at(i)(3));
        eor_y.push_back(poseError.at(i)(4));
        eor_z.push_back(poseError.at(i)(5));

        // Joint positions
        mobPlat_x.push_back(q.at(i)(0));
        mobPlat_y.push_back(q.at(i)(1));
        z_pj.push_back(q.at(i)(3));
        qa1.push_back(q.at(i)(4));
        qa2.push_back(q.at(i)(5));
        qa3.push_back(q.at(i)(6));
        qa4.push_back(q.at(i)(7));
        qa5.push_back(q.at(i)(8));
        qa6.push_back(q.at(i)(9));

        // Joint velocities
        mobPlat_v.push_back(eta.at(i)(0));
        mobPlat_w.push_back(eta.at(i)(1));
        dz_pj.push_back(eta.at(i)(2));
        dqa1.push_back(eta.at(i)(3));
        dqa2.push_back(eta.at(i)(4));
        dqa3.push_back(eta.at(i)(5));
        dqa4.push_back(eta.at(i)(6));
        dqa5.push_back(eta.at(i)(7));
        dqa6.push_back(eta.at(i)(8));
    }
    MatrixXd qlimits = robot.getJointLim();
    VectorXd dqlimits = robot.getJointVelLim();
    vector<double>::const_iterator first = desiredTraj.getTrajTime().begin();
    vector<double>::const_iterator last = desiredTraj.getTrajTime().begin() + NObt;
    std::vector<double> timeObt(first, last);
    cout << "timeObt size:" << timeObt.size() << endl;

    if (NObt < N)
    {
        wMeasure.resize(NObt);
        MMmanip.resize(NObt);
        UR5manip.resize(NObt);
    }

    // Figure (1)
    // Manipulability plots
    plt::figure_size(1600, 900);
    plt::subplot(2,2,1);
    plt::named_plot("w_MM", timeObt, wMeasure);
    plt::named_plot("w_{p+a}", timeObt, MMmanip);
    plt::named_plot("w_{a}", timeObt, UR5manip);
    plt::grid(true);
    plt::xlabel("time");    
    plt::legend();
    // Mob plat trajectory
    plt::subplot(2,2,2);
    std::vector<double> mobPlatStart_x(2, mobPlat_x.at(0)),
                        mobPlatEnd_x(2, mobPlat_x.at(NObt-1)), 
                        mobPlatStart_y(2, mobPlat_y.at(0)), 
                        mobPlatEnd_y(2, mobPlat_y.at(NObt-1));        
    plt::named_plot("Traj", mobPlat_x, mobPlat_y);
    plt::named_plot("Start Pos", mobPlatStart_x, mobPlatStart_y,"o");
    plt::named_plot("Final Pos", mobPlatEnd_x, mobPlatEnd_y,"s");
    plt::grid(true);
    plt::ylabel("y(m)");
    plt::xlabel("x(m)");
    plt::legend();
    // Position and Orientation errors
    plt::subplot(2,2,3);
    plt::named_plot("e_{Px}", timeObt, epos_x);
    plt::named_plot("e_{Py}", timeObt, epos_y);
    plt::named_plot("e_{Pz}", timeObt, epos_z);
    plt::grid(true);
    plt::xlabel("time");
    plt::legend();
    plt::subplot(2,2,4);
    plt::named_plot("e_{Ox}", timeObt, eor_x);
    plt::named_plot("e_{Oy}", timeObt, eor_y);
    plt::named_plot("e_{Oz}", timeObt, eor_z);
    plt::grid(true);
    plt::xlabel("time");
    plt::legend();

    // Figure (2)
    // Mob plat velocities
    plt::figure_size(1600, 900);
    plt::subplot(2, 3, 1);
    plt::named_plot("v(m/s)", timeObt, mobPlat_v, "b");
    plt::named_plot("w(rad/s)", timeObt, mobPlat_w, "r");
    plt::named_plot("v-", timeObt, std::vector<double>(NObt, -1*dqlimits(0)), "--b");
    plt::named_plot("v+", timeObt, std::vector<double>(NObt, dqlimits(0)), "-.b");
    plt::named_plot("w-", timeObt, std::vector<double>(NObt, -1*dqlimits(1)), "--r");
    plt::named_plot("w+", timeObt, std::vector<double>(NObt, dqlimits(1)), "-.r");    
    plt::grid(true);
    plt::xlabel("time");    
    plt::legend();
    // Prismatic joint
    plt::subplot(2, 3, 2);
    plt::named_plot("z(m)", timeObt, z_pj, "y");
    plt::named_plot("dz(m/s)", timeObt, dz_pj, "c");
    plt::named_plot("z-", timeObt, std::vector<double>(NObt, qlimits(2, 0)), "--y");
    plt::named_plot("z+", timeObt, std::vector<double>(NObt, qlimits(2, 1)), "-.y");    
    plt::named_plot("dz-", timeObt, std::vector<double>(NObt, -1*dqlimits(2)), "--c");
    plt::named_plot("dz+", timeObt, std::vector<double>(NObt, dqlimits(2)), "-.c");
    plt::grid(true);
    plt::xlabel("time");    
    plt::legend();
    // UR5 Joints 1 and 2
    plt::subplot(2, 3, 3);
    plt::named_plot("qa1", timeObt, qa1, "b");
    plt::named_plot("qa2", timeObt, qa2, "r");
    plt::named_plot("qa1-", timeObt, std::vector<double>(NObt, qlimits(3, 0)), "--b");
    plt::named_plot("qa1+", timeObt, std::vector<double>(NObt, qlimits(3, 1)), "-.b");    
    plt::named_plot("qa2-", timeObt, std::vector<double>(NObt, qlimits(4, 0)), "--r");
    plt::named_plot("qa2+", timeObt, std::vector<double>(NObt, qlimits(4, 1)), "-.r");
    plt::grid(true);
    plt::xlabel("time");    
    plt::legend();
    // UR5 Joint 3 and 4
    plt::subplot(2, 3, 4);
    plt::named_plot("qa3", timeObt, qa3, "y");
    plt::named_plot("qa4", timeObt, qa4, "m");
    plt::named_plot("qa3-", timeObt, std::vector<double>(NObt, qlimits(5, 0)), "--y");
    plt::named_plot("qa3+", timeObt, std::vector<double>(NObt, qlimits(5, 1)), "-.y");    
    plt::named_plot("qa4-", timeObt, std::vector<double>(NObt, qlimits(6, 0)), "--m");
    plt::named_plot("qa4+", timeObt, std::vector<double>(NObt, qlimits(6, 1)), "-.m");
    plt::grid(true);
    plt::xlabel("time");    
    plt::legend();
    // UR5 Joint 5 and 6
    plt::subplot(2, 3, 5);
    plt::named_plot("qa5", timeObt, qa5, "g");
    plt::named_plot("qa6", timeObt, qa6, "c");
    plt::named_plot("qa5-", timeObt, std::vector<double>(NObt, qlimits(7, 0)), "--g");
    plt::named_plot("qa5+", timeObt, std::vector<double>(NObt, qlimits(7, 1)), "-.g");    
    plt::named_plot("qa6-", timeObt, std::vector<double>(NObt, qlimits(8, 0)), "--c");
    plt::named_plot("qa6+", timeObt, std::vector<double>(NObt, qlimits(8, 1)), "-.c");
    plt::grid(true);
    plt::xlabel("time");    
    plt::legend();
    // UR5 Joint velocities
    plt::subplot(2, 3, 6);
    plt::named_plot("dqa1", timeObt, dqa1, "b");
    plt::named_plot("dqa2", timeObt, dqa2, "r");
    plt::named_plot("dqa3", timeObt, dqa3, "y");
    plt::named_plot("dqa4", timeObt, dqa4, "m");
    plt::named_plot("dqa5", timeObt, dqa5, "g");
    plt::named_plot("dqa6", timeObt, dqa6, "c");
    plt::grid(true);
    plt::xlabel("time");    
    plt::legend();
    plt::show();

    return 0;
}

std::vector<double> stepSizeTrans(std::vector<double> dx, std::vector<double> dy, std::vector<double> dz, double tf, double ts)
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
    double deltaTime;
    int k;
    double t;
    for (k = 0, t = 0; k < N; k++, t += ts)
    {
        if (k > (N / 2))
        {
            if (k > (N * (1 - 0.15)))
            {
                deltaTime = t - tb;
                trans.push_back(a0 + a1 * (deltaTime) + a2 * pow(deltaTime, 2) + a3 * pow(deltaTime, 3) + a4 * pow(deltaTime, 4) + a5 * pow(deltaTime, 5));
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

void testPointsMaxLinVel(int testN, VectorXd &q, Pose &posef, double &tf)
{
    double tx, ty, phi, tz;         // Mobile platform initial states
    VectorXd qa(6);                 // Robot arm initial joint values
    Vector3d pos_des;               // Desired Position
    AngleAxisd eigen_angaxis_des;   // Desired Orientation angle axis
    Quaterniond eigen_quatf;        // Desired Orientation quaternion
    switch (testN)
    {
        case 1:
            tx = 0, ty = 0, phi = 0, tz=0.2;  
            qa << 0, -0.4, 1.06, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 3.0, -0.1091, 0.18;
            eigen_angaxis_des = AngleAxisd(M_PI, Vector3d(0, 1, 0).normalized());
            tf = 7;
            break;
        case 2:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -0.4, 1.06, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 3.0, 0.0, 0.32;
            eigen_angaxis_des = AngleAxisd(M_PI/2.0, Vector3d(0, 1, 0).normalized());
            tf = 9;
            break;
        case 3:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -1*M_PI/2, 3*M_PI/4, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 2.0, 3.0, 1.3;
            eigen_angaxis_des = AngleAxisd(M_PI, Vector3d(0, 1, 0).normalized());
            tf = 10;
            break;
        case 4:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -1*M_PI/2, 3*M_PI/4, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 1.0, -0.8, 0.4;
            eigen_angaxis_des = AngleAxisd(120*M_PI/180, Vector3d(1, 1, -1).normalized());
            tf = 18;
            break;
        case 5:
            tx = 0, ty = 0.1, phi = 0, tz=0.2;
            qa << 0, -1*M_PI/2, 3*M_PI/4, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << -1.2, -1.2, 0.15;
            eigen_angaxis_des = AngleAxisd(M_PI, Vector3d(1, 0, 0).normalized());
            tf = 10;
            break;
        case 6: //Example case that cannot be achieved with the given time
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -0.4, 1.06, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << -2.2, 2.2, 1.5;
            eigen_angaxis_des = AngleAxisd(M_PI, Vector3d(0, -1, -1).normalized());
            tf = 10;
            break;
        case 7:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -0.4, 1.06, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 1, 1, 0.6;
            eigen_angaxis_des = AngleAxisd(M_PI/2, Vector3d(0, 1, 0).normalized());
            tf = 6;
            break;
        case 8:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -1*M_PI/2, 3*M_PI/4, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 1.2, 1.2, 0.7;
            eigen_angaxis_des = AngleAxisd(120*M_PI/180, Vector3d(-1, -1, 1).normalized());
            tf = 8;
            break;
        case 9:
            tx = 0, ty = 0, phi = 0, tz=0.05;
            qa << 0, -0.4, 1.06, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 0.5, -3, 0.15;
            eigen_angaxis_des = AngleAxisd(120*M_PI/180, Vector3d(-1, 1, 1).normalized());
            tf = 18;
            break;
        case 10:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -0.4, 1.06, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << -3, 0.1091, 0.8;
            eigen_angaxis_des = AngleAxisd(120*M_PI/180, Vector3d(1, 1, -1).normalized());
            tf = 18;
            break;
        case 11:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -1*M_PI/2, 3*M_PI/4, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 2, 1, 0.4;
            eigen_angaxis_des = AngleAxisd(3.0075, Vector3d(0.9351, 0.2506, -0.2506).normalized());
            tf = 10;
            break;
        case 12:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -1*M_PI/2, 3*M_PI/4, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 5.5, 6.5, 1.2;
            eigen_angaxis_des = AngleAxisd(M_PI, Vector3d(0, 1, 0).normalized());
            tf = 25;
            break;
        case 13:
            tx = 0, ty = 0, phi = 0, tz=0.2;
            qa << 0, -0.4, 1.06, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << 2, 3, 0.15;
            eigen_angaxis_des = AngleAxisd(M_PI, Vector3d(0, 1, 0).normalized());
            tf = 18;
            break;
        case 14:
            tx = 0, ty = 0, phi = 0, tz=0.05;
            qa << 0, -0.4, 1.06, 5*M_PI/4, -1*M_PI/2, 0.0;
            pos_des << -0.3, 0, 0.13;
            eigen_angaxis_des = AngleAxisd(M_PI, Vector3d(0, 1, 0).normalized());
            tf = 8;
            break;
        case 15:
            // Dificult case, where the final position is far and the final
            // orientation puts the joint 5 in a singular position
            tx = 0, ty = 0, phi = M_PI/2, tz=0.05;
            qa << -M_PI/2, -M_PI/4, M_PI/2, 3*M_PI/4, -M_PI/2, 0.0;
            pos_des << 0.7221, 8, 0.7246;
            eigen_angaxis_des = AngleAxisd(120*M_PI/180, Vector3d(-1, 1, -1).normalized());
            tf = 14;
            break;
        default:
            break;
    }

    // Store initial joint states in q
    q = VectorXd(10);
    q << tx, ty, phi, tz, qa;
    // Store desired pose in the posef object
    eigen_quatf = Quaterniond(eigen_angaxis_des);
    posef = Pose(pos_des, Quat(eigen_quatf));
}