#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <mars_mot_plan/PoseTrajectoryAction.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>

#include "mars_mot_plan/kinematics/MarsUR5.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include "mars_mot_plan/traj_plan/PoseIterTrajectory.h"
#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pinv(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}); // choose appropriately

class MarsPoseTrajActionSim
{
  public:
    MarsPoseTrajActionSim(std::string name) : as(nh_, name, boost::bind(&MarsPoseTrajActionSim::executeCB, this, _1), false),
                                              action_name(name)
    {
        freq = 50.0;
        alpha = 8;
        KpPos = 10;
        KpOr = 20;

        wError = MatrixXd::Zero(6, 6);
        wError.block<3, 3>(0, 0) = Matrix3d::Identity() * KpPos;
        wError.block<3, 3>(3, 3) = Matrix3d::Identity() * KpOr;

        robot = MarsUR5();
        invTq = robot.getVelsNormMatrix();
        Id = MatrixXd::Identity(9, 9);
        S = MatrixXd::Zero(10, 9);
        S.block<8, 8>(2, 1).setIdentity();
        as.start();
        ROS_INFO_STREAM("Mars motion planning action server started");
    }

    ~MarsPoseTrajActionSim(void)
    {
    }

    void executeCB(const mars_mot_plan::PoseTrajectoryGoalConstPtr &goal)
    {
        ROS_INFO_STREAM("Goal received");

        // Get the initial joint states
        VectorXd q0 = VectorXd(10);
        double tx = 0, ty = 0, phi = 0, tz = 0.2;
        VectorXd qa(6);
        qa << 0, -0.4, 1.06, 5 * M_PI / 4, -1 * M_PI / 2, 0.0;
        q0 << tx, ty, phi, tz, qa;
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
                  << "q6: " << q0(9) * 180 / M_PI << endl
                  << endl;
        robot.setJointPositions(q0);
        pose0 = Pose(robot.getEETransform());
        std::cout << "T0:" << endl
                  << pose0.matrixRep() << endl;
        std::cout << "Pose0:" << endl
                  << pose0 << endl
                  << endl;

        // read desired time and pose from goal
        tf = goal->trajTime;
        if (tf < 0.1)
        {
            ROS_ERROR("%s: received trajectory time is too short.", action_name.c_str());
            // set the action state to abort
            as.setAborted();
            return;
        }
        posef = Pose(goal->desPose);
        std::cout << "Tf:" << endl
                  << posef.matrixRep() << endl;
        std::cout << "Posef:" << endl
                  << posef << endl
                  << endl;

        /*
            **********************************************************
            **************** Pose trajectory planning ****************
            **********************************************************
        */
        std::cout << "Planing trajectory:" << endl;
        desiredTraj = PoseIterTrajectory(pose0, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                                         posef, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
                                         0.0, tf);

        /*
            **********************************************************
            **************** Step size variation law  ****************
            **********************************************************
        */
        trans.clear();
        maxLinVelCoeff.clear();
        char maxLinVelCoord;
        double maxLinVel = desiredTraj.getMaxLinVel(maxLinVelCoord);
        maxLinVelCoeff = desiredTraj.getPosCoeff(maxLinVelCoord);

        // Clear and initialize variables that are reused
        time.clear();
        q.clear();
        eta.clear();
        dq.clear();
        pose.clear();
        poseError.clear();
        wMeasure.clear();
        MMmanip.clear();
        UR5manip.clear();
        elbowDistVector.clear();
        wristDistVector.clear();
        wristHeightVector.clear();
        partSol = VectorXd::Zero(9);
        homSol = VectorXd::Zero(9);

        bool success = true;
        int k = 0;
        trajDuration = 0.0;
        std::cout << "Motion planning started" << endl;
        startTime = ros::Time::now();
        nowTime = startTime;
        prevTime = nowTime;
        ros::Rate rate(freq);
        // Copy the initial values of the joint positions
        q.push_back(q0);
        while (trajDuration <= tf)
        {
            // check that preempt has not been requested by the client
            if (as.isPreemptRequested() || !ros::ok())
            {
                ROS_WARN("%s: Preempted", action_name.c_str());

                // TO DO: Stop all the joints

                // IMPORTANT!

                // set the action state to preempted
                as.setPreempted();
                success = false;
                break;
            }

            time.push_back(trajDuration);
            // Set the current joint positions to the robot
            robot.setJointPositions(q.at(k));

            // Get the kth pose
            pose.push_back(Pose::matrixToPose(robot.getEETransform()));

            // Replace the elements of the nonholonomic constraints matrix
            S(0, 0) = cos(q.at(k)(2));
            S(1, 0) = sin(q.at(k)(2));

            // Compute the manipulability gradients
            robot.getManipGrads(MMdP, MMman, UR5dP, UR5man);
            dP = UR5man * MMdP + MMman * UR5dP;
            dP = S.transpose() * dP;
            wMeasure.push_back(MMman * UR5man);
            MMmanip.push_back(MMman);
            UR5manip.push_back(UR5man);

            // Compute the weighting matrices
            robot.getJLimWeight(Wjlim);
            robot.getElbowColWeight(0.001, 50, 1, WColElbow, elbowDist);
            robot.getWristColWeight(0.001, 50, 1, WColWrist, wristDist, wristPos);
            elbowDistVector.push_back(elbowDist);
            wristDistVector.push_back(wristDist);
            wristHeightVector.push_back(wristPos(2));
            Wmatrix = Wjlim * WColElbow * WColWrist * invTq;

            // Compute the position and orientation error
            poseError.push_back(Pose::pose_diff(desiredTraj.getPose(trajDuration), pose.at(k)));

            // Compute the particular and homogeneous solutions
            // particular solution
            JBar = robot.getJacobianBar();
            JBarW = JBar * Wmatrix;
            invJBarW = pinv(JBarW);
            partSol = invJBarW * (desiredTraj.getVel(trajDuration) + wError * poseError.at(k));
            partSol = Wmatrix * partSol;

            // Compute the step size transition
            trans.push_back(stepSizeTrans(maxLinVelCoeff, maxLinVel, tf, trajDuration));

            // homogeneous solution
            homSol = trans.at(k) * (Id - invJBarW * JBarW) * Wmatrix * dP;
            homSol = Wmatrix * homSol;

            // Compute the maximum and minimum step size for the homogeneous solution
            robot.getSelfMotionLims(partSol, homSol, maxAlpha, minAlpha);
            if (maxAlpha < minAlpha)
            {
                ROS_ERROR("Could not acheive task that complies with joint velocity limits. ");

                // TO DO: Stop all the joints

                // IMPORTANT!

                success = false;
                as.setAborted(result);
                break;
            }
            // Saturate alpha in case is out of limits
            alphak = alpha;
            if (alphak > maxAlpha)
            {
                ROS_WARN("Potential velocity limit reached. Saturating step size");
                alphak = maxAlpha;
            }
            if (alphak < minAlpha)
            {
                ROS_WARN("Potential velocity limit reached. Saturating step size");
                alphak = minAlpha;
            }

            // Compute the joint velocities
            eta.push_back(partSol + alphak * homSol);
            dq.push_back(S * eta.at(k));

            // Update joint positions for next iteration
            if (trajDuration < tf)
            {
                rate.sleep();
                nowTime = ros::Time::now();
                trajDuration = (nowTime - startTime).toSec();
                ts = (nowTime - prevTime).toSec();

                q.push_back(q.at(k) + dq.at(k)*ts);

                prevTime = nowTime;
                // Publish the feedback
                feedback.currTime = trajDuration;
                feedback.currPose = Pose::poseToGeomMsgPose(pose.at(k));
                as.publishFeedback(feedback);
            }
            k++;
        }
        // Substract sampling time from time
        //trajDuration -= ts;
        double finalTimeDiff = tf - trajDuration;
        cout << "tf: " << tf << endl;
        cout << "trajDuration: " << trajDuration << endl;
        cout << "Time diff: " << finalTimeDiff << endl;

        if (abs(finalTimeDiff) < 1e-3)
        {
            ROS_INFO_STREAM("Motion planning completed. Number of iterations: " << k + 1 << endl);
            ROS_INFO("%s: Succeeded", action_name.c_str());
            result.finalTime = trajDuration;
            result.finalPose = Pose::poseToGeomMsgPose(pose.back());
            VectorXd finalError = poseError.back();
            result.finalError.linear.x = finalError(0);
            result.finalError.linear.y = finalError(1);
            result.finalError.linear.z = finalError(2);
            result.finalError.angular.x = finalError(3);
            result.finalError.angular.y = finalError(4);
            result.finalError.angular.z = finalError(5);
            as.setSucceeded(result);
        }
        else
        {
            ROS_ERROR("Task could not be completed. ");
            // remove last element of vectors that have already been increased
            time.pop_back();    
            wMeasure.pop_back();
            MMmanip.pop_back();
            UR5manip.pop_back();
        }

        // Show final position and orientation errors
        std::cout << "Desired final pos: " << endl
                  << desiredTraj.getPose(tf) << endl;
        std::cout << "Obtained final pos: " << endl
                  << pose.back() << endl;
        std::cout << "Final pos error: " << endl
                  << poseError.back().head(3).transpose() << endl;
        std::cout << "Pos error norm: " << endl
                  << poseError.back().head(3).norm() << endl;
        std::cout << "Final orientation error: " << endl
                  << poseError.back().tail(3).transpose() << endl;
        std::cout << "Orientation error norm: " << endl
                  << poseError.back().tail(3).norm() << endl;
    }

  private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<mars_mot_plan::PoseTrajectoryAction> as;
    std::string action_name;
    mars_mot_plan::PoseTrajectoryFeedback feedback;
    mars_mot_plan::PoseTrajectoryResult result;
    ros::Subscriber as_sub;

    MarsUR5 robot; // robot object

    // Configuration parameters
    double ts, tf, freq, alpha, KpPos, KpOr;
    double trajDuration;
    // Variables to be calculated before motion planning
    Pose pose0, posef;
    PoseIterTrajectory desiredTraj;
    std::vector<double> maxLinVelCoeff;
    std::vector<double> trans;

    std::vector<VectorXd> q, eta, dq; // Joint position and velocities
    std::vector<Pose> pose;           // pose at each iteration
    std::vector<VectorXd> poseError;  // pose error at each tireation
    std::vector<double> time, wMeasure, MMmanip, UR5manip;
    ros::Time startTime, nowTime, prevTime;

    MatrixXd wError; // Error weighting matrix
    MatrixXd invTq, Wjlim, WColElbow, WColWrist;

    std::vector<double> elbowDistVector, wristDistVector, wristHeightVector;
    double elbowDist, wristDist, wristHeight;
    Vector3d wristPos;
    VectorXd partSol, homSol;
    MatrixXd Id, S; // Identity and non-holonomic constraints matrix

    MatrixXd JBar, JBarW, invJBarW, Wmatrix;
    VectorXd MMdP, UR5dP, dP;
    double MMman, UR5man, alphak, maxAlpha, minAlpha;

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
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mars_mot_plan_exp_server");

    MarsPoseTrajActionSim trajAction(ros::this_node::getName());
    ros::spin();

    return 0;
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
    for (unsigned int i = 0; i < singularValues.size(); ++i)
    {
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
