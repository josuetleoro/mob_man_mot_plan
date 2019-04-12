#include "mars_mot_plan/kinematics/MarsUR5.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include "mars_mot_plan/traj_plan/PoseIterTrajectory.h"
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

double stepSizeTrans(std::vector<double> coeff, double maxVel, double tf, double t);
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pinv(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}); // choose appropriately

void testPointsMaxLinVel(int testN, VectorXd &q, Pose &posef, double &tf);
double findFourthOrderMax(std::vector<double> coef);

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
    std::cout << "Initial joint angles: " << endl
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
    std::cout << "T0:" << endl << pose0.matrixRep() << endl;
    std::cout << "Pose0:" << endl << pose0 << endl << endl;
    std::cout << "Tf:" << endl << posef.matrixRep() << endl;
    std::cout << "Posef:" << endl << posef << endl << endl;
    
    /*
        **********************************************************
        **************** Pose trajectory planning ****************
        **********************************************************
    */
    PoseIterTrajectory desiredTraj(pose0, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                              posef, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                              t0, tf);
    
    vector<double> time, x, y, z, dx, dy, dz, ddx, ddy, ddz;
    vector<double> quatW, quatX, quatY, quatZ, wX, wY, wZ;
    Quat quat;
    VectorXd poseVec;
    VectorXd vel;
    //vector<>
    double currTime = t0;
    int k = 0;
    std::cout << "Planning trajectory" << endl;
    while (currTime < tf)
    {
        //time.push_back(currTime);
        /*x.push_back(desiredTraj.getPos('x', currTime));
        y.push_back(desiredTraj.getPos('y', currTime));
        z.push_back(desiredTraj.getPos('z', currTime));*/
        /*dx.push_back(desiredTraj.getLinVel('x', currTime));
        dy.push_back(desiredTraj.getLinVel('y', currTime));
        dz.push_back(desiredTraj.getLinVel('z', currTime));*/
        /*ddx.push_back(desiredTraj.getLinAcc('x', currTime));
        ddy.push_back(desiredTraj.getLinAcc('y', currTime));
        ddz.push_back(desiredTraj.getLinAcc('z', currTime));*/
        /*quatW.push_back(desiredTraj.getOrientPart('w', currTime));
        quatX.push_back(desiredTraj.getOrientPart('x', currTime));
        quatY.push_back(desiredTraj.getOrientPart('y', currTime));
        quatZ.push_back(desiredTraj.getOrientPart('z', currTime));*/
        /*wX.push_back(desiredTraj.getAngVel('x', currTime));
        wY.push_back(desiredTraj.getAngVel('y', currTime));
        wZ.push_back(desiredTraj.getAngVel('z', currTime));*/

        poseVec = desiredTraj.getPoseVec(currTime);
        x.push_back(poseVec(0));
        y.push_back(poseVec(1));
        z.push_back(poseVec(2));
        quatW.push_back(poseVec(3));
        quatX.push_back(poseVec(4));
        quatY.push_back(poseVec(5));
        quatZ.push_back(poseVec(6));

        vel = desiredTraj.getVel(currTime);
        dx.push_back(vel(0)); dy.push_back(vel(1)); dz.push_back(vel(2));
        wX.push_back(vel(3)); wY.push_back(vel(4)); wZ.push_back(vel(5));       

        //pose.push_back(desiredTraj.getPose(currTime));     
        currTime += ts;
    }

    // Position and Orientation errors
    /*plt::figure(1);
    plt::subplot(1,2,1);
    plt::named_plot("x", time, x);
    plt::named_plot("y", time, y);
    plt::named_plot("z", time, z);
    plt::grid(true);
    plt::xlabel("time");
    plt::legend();

    plt::subplot(1,2,2);
    plt::named_plot("dx", time, dx);
    plt::named_plot("dy", time, dy);
    plt::named_plot("dz", time, dz);
    plt::grid(true);
    plt::xlabel("time");
    plt::legend();   

    plt::figure(2);
    plt::subplot(2,2,1);
    plt::named_plot("quatw", time, quatW);
    plt::grid(true);
    plt::legend();
    plt::subplot(2,2,2);
    plt::named_plot("quatx", time, quatX);
    plt::grid(true);
    plt::legend();
    plt::subplot(2,2,3);
    plt::named_plot("quaty", time, quatY);
    plt::grid(true);
    plt::legend();
    plt::subplot(2,2,4);
    plt::named_plot("quatz", time, quatZ);
    plt::grid(true);
    plt::legend();
    plt::xlabel("time");

    plt::figure(3);
    plt::named_plot("wx", time, wX);
    plt::named_plot("wy", time, wY);
    plt::named_plot("wz", time, wZ);
    plt::grid(true);
    plt::legend();
    plt::show();*/
    
    /*
        **********************************************************
        **************** Step size variation law  ****************
        **********************************************************
    */
    char maxLinVelCoord;
    double maxLinVel = desiredTraj.getMaxLinVel(maxLinVelCoord);
    std::vector<double> maxLinVelCoeff = desiredTraj.getPosCoeff(maxLinVelCoord);
    std::vector<double> trans;
    /*
    std::vector<double> time2;
    currTime = t0;
    k = 0;
    while (currTime < tf)
    {
        time2.push_back(currTime);
        trans.push_back(stepSizeTrans(maxLinVelCoeff, maxLinVel, tf, currTime));

        //pose.push_back(desiredTraj.getPose(currTime));     
        currTime += ts;
    }
    plt::figure(5);
    plt::plot(time2, trans);
    plt::grid(true);
    plt::show();
    */

    /*
        **********************************************************
        ********************* Motion planning  *******************
        **********************************************************
    */

    // Initialize all the variables
    std::vector<double> timeObt;
    std::vector<VectorXd> q;    // 10 elements
    std::vector<VectorXd> eta;  // 9 elements
    std::vector<VectorXd> dq;   // 10 elements
    std::vector<Pose> xi;   
    std::vector<VectorXd> poseError; // 6 elements
    std::vector<double> wMeasure;
    std::vector<double> MMmanip; 
    std::vector<double> UR5manip;

    // Error weighting matrix
    MatrixXd wError = MatrixXd::Zero(6, 6);
    wError.block<3,3>(0,0) = Matrix3d::Identity()*KpPos;
    wError.block<3,3>(3,3) = Matrix3d::Identity()*KpOr;
    //cout << "wError: " << endl << wError << endl;

    // Joint limits and collision avoidance variables
    MatrixXd invTq = robot.getVelsNormMatrix();
    MatrixXd Wjlim, WColElbow, WColWrist;
    std::vector<double> elbowDistVector;
    std::vector<double> wristDistVector;
    std::vector<double> wristHeightVector;
    double elbowDist, wristDist, wristHeight;
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
    
    // Copy the initial values of the joint positions
    q.push_back(q0);

    MatrixXd JBar, JBarW, invJBarW, Wmatrix;
    VectorXd MMdP, UR5dP, dP;
    double MMman, UR5man, alphak, maxAlpha, minAlpha;
    high_resolution_clock::time_point start, end;
    start = high_resolution_clock::now();
    k = 0;
    currTime = 0.0;
    std::cout << "Motion planning started" << endl;
    while (currTime < tf)
    {
        timeObt.push_back(currTime);
        // Set the current joint positions to the robot
        robot.setJointPositions(q.at(k));
        
        // Get the kth pose
        xi.push_back(Pose::matrixToPose(robot.getEETransform()));

        // Replace the elements of the nonholonomic constraints matrix
        S(0, 0) = cos(q.at(k)(2));  S(1, 0) = sin(q.at(k)(2));

        // Compute the manipulability gradients
        robot.getManipGrads(MMdP, MMman, UR5dP, UR5man);
        dP =  UR5man*MMdP + MMman*UR5dP;
        dP =  S.transpose()*dP;
        wMeasure.push_back(MMman * UR5man);
        MMmanip.push_back(MMman); UR5manip.push_back(UR5man);

        // Compute the weighting matrices
        robot.getJLimWeight(Wjlim);
        robot.getElbowColWeight(0.001, 50, 1, WColElbow, elbowDist);
        robot.getWristColWeight(0.001, 50, 1, WColWrist, wristDist, wristPos);
        elbowDistVector.push_back(elbowDist);
        wristDistVector.push_back(wristDist);
        wristHeightVector.push_back(wristPos(2));
        Wmatrix = Wjlim * WColElbow * WColWrist * invTq;
        
        // Compute the position and orientation error        
        poseError.push_back(Pose::pose_diff(desiredTraj.getPose(currTime), xi.at(k)));

        // Compute the particular and homogeneous solutions
        // particular solution
        JBar = robot.getJacobianBar();
        JBarW = JBar * Wmatrix;
        invJBarW = pinv(JBarW);
        partSol = invJBarW*(desiredTraj.getVel(currTime) + wError*poseError.at(k));
        partSol = Wmatrix*partSol;

        // Compute the step size transition
        trans.push_back(stepSizeTrans(maxLinVelCoeff, maxLinVel, tf, currTime));

        // homogeneous solution
        homSol = trans.at(k) * (Id - invJBarW*JBarW) * Wmatrix * dP;
        homSol = Wmatrix*homSol;

        // Compute the maximum and minimum step size for the homogeneous solution
        robot.getSelfMotionLims(partSol, homSol, maxAlpha, minAlpha);
        if (maxAlpha < minAlpha)
        {
            std::cout << "\033[1;31mCould not acheive task that complies with joint velocity limits\033[0m" << endl;
            break;
        }
        // Saturate alpha in case is out of limits
        alphak = alpha;
        if (alphak > maxAlpha)
        {
            std::cout << "\033[1;33mPotential velocity limit reached. Saturating step size.\033[0m" << endl;
            alphak = maxAlpha;
        }
        if (alphak < minAlpha)
        {
            std::cout << "\033[1;33mPotential velocity limit reached. Saturating step size.\033[0m" << endl;
            alphak = minAlpha;
        }

        // Compute the joint velocities
        eta.push_back(partSol + alphak * homSol);
        dq.push_back(S * eta.at(k));

        // Update joint positions for next iteration
        if (currTime < tf)
        {
            q.push_back(q.at(k) + dq.at(k)*ts);
        }
        currTime += ts;
        k++;
    }
    end = high_resolution_clock::now();

    // Substract final iteration and time
    k = k - 1;
    currTime = currTime - ts;
    double finalTimeDiff = tf - currTime;
    //cout << "Time diff: " << finalTimeDiff << endl;
    if (abs(finalTimeDiff) < 1e-3)
    {
        // Compute the execution time in us
        double time_ns = (double)(duration_cast<nanoseconds>(end - start).count());
        std::cout << "Motion planning completed. Number of iterations: " << k << endl;
        std::cout << "Motion planning execution time: " << time_ns/1000000 << "ms" << endl;
        std::cout << "Motion planning mean time: " << time_ns/(1000*k) << "us" << endl;
    }
    else
    {
        std::cout << "\033[1;31mTask could not be completed\033[0m" << endl;
    }

    // Show final position and orientation errors
    std::cout << "Desired final pos: " << endl << desiredTraj.getPose(currTime) << endl;
    std::cout << "Obtained final pos: " << endl << xi.at(k) << endl;
    std::cout << "Final pos error: " << endl << poseError.at(k).head(3).transpose() << endl;
    std::cout << "Pos error norm: " << endl << poseError.at(k).head(3).norm() << endl;
    std::cout << "Final orientation error: " << endl << poseError.at(k).tail(3).transpose() << endl;
    std::cout << "Orientation error norm: " << endl << poseError.at(k).tail(3).norm() << endl;

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
    std::cout << "timeObt size:" << timeObt.size() << endl;

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

double stepSizeTrans(std::vector<double> coeff, double maxVel, double tf, double t)
{
    double deltaTime;
    double tb = tf * (1 - 0.15);
    double stepSize;
    double T = tf - tb;
    double a0 = 1, a1 = 0, a2 = 0, a3 = -10 / pow(T, 3), a4 = 15 / pow(T, 4), a5 = -6 / pow(T, 5);
    if (t > (tf / 2))
    {
        if (t > tb)
        {
            deltaTime = t - tb;
            stepSize = a0 + a1 * (deltaTime) + a2 * pow(deltaTime, 2) + a3 * pow(deltaTime, 3) + a4 * pow(deltaTime, 4) + a5 * pow(deltaTime, 5);
        }
        else
        {
            stepSize = 1.0;
        }
    }
    else
    {
        stepSize = abs(TrajPlan::dqPol(coeff, t)) / maxVel;
    }
    return stepSize;
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

