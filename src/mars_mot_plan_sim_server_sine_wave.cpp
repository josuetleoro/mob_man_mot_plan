#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <mars_mot_plan/PoseTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/QR>
#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>

#include "mars_mot_plan/kinematics/MarsUR5.h"
#include "mars_mot_plan/kinematics/Pose.h"
#include "mars_mot_plan/traj_plan/TrajPlan.h"
#include "mars_mot_plan/traj_plan/LinePathTrajectory.h"
#include "mars_mot_plan/traj_plan/EllipticPathTrajectory.h"
#include "mars_mot_plan/traj_plan/LissajousPathTrajectory.h"
#include "mars_mot_plan/traj_plan/SineWavePathTrajectory.h"
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
        alpha = 2;
        KpPos = 10;
        KpOr = 20;

        stepVarCoeff.resize(3);

        wError = MatrixXd::Zero(6, 6);
        wError.block<3, 3>(0, 0) = Matrix3d::Identity() * KpPos;
        wError.block<3, 3>(3, 3) = Matrix3d::Identity() * KpOr;

        robot = MarsUR5();
        invTq = robot.getVelsNormMatrix();
        //cout << "InvTq: " << endl << invTq << endl;
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
        // read desired time and pose from goal
        tf = goal->trajTime;
        if (tf < 0.1)
        {
            ROS_ERROR("%s: received trajectory time is too short.", action_name.c_str());
            // set the action state to abort
            as.setAborted();
            return;
        }
        ROS_INFO("%s: Goal received", action_name.c_str());

        // Get the initial mobile platform position and orientation
        VectorXd qa(6);
        try
        {
            listener.waitForTransform("/vive_world", "/base_footprint",
                                      ros::Time(0), ros::Duration(0.2));
            listener.lookupTransform("/vive_world", "/base_footprint",
                                     ros::Time(0), mob_plat_base_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Transform from frame vive_world to base_footprint not available yet");
            ROS_ERROR("%s", ex.what());
            return;
        }
        // Get the initial joint states
        double tx, ty, phi, tz;
        VectorXd q0 = VectorXd(10);
        tx = mob_plat_base_transform.getOrigin().x();
        ty = mob_plat_base_transform.getOrigin().y();
        phi = tf::getYaw(mob_plat_base_transform.getRotation());

        joint_state = *(ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", nh_, ros::Duration(0.5)));
        std::vector<double> joint_pos = joint_state.position;
        tz = joint_pos.at(2);
        qa << joint_pos.at(3), joint_pos.at(4), joint_pos.at(5),
            joint_pos.at(6), joint_pos.at(7), joint_pos.at(8);
        q0 << tx, ty, phi, tz, qa;
        // Show the initial joint positions
        std::cout << "Initial joint positions: " << endl
                  << "tx: " << q0(0) << "\t"
                  << "ty: " << q0(1) << "\t"
                  << "phi: " << q0(2) * 180 / M_PI << "\t"
                  << "tz: " << q0(3) << "\n"
                  << "q1: " << q0(4) * 180 / M_PI << "\t"
                  << "q2: " << q0(5) * 180 / M_PI << "\t"
                  << "q3: " << q0(6) * 180 / M_PI << "\t"
                  << "q4: " << q0(7) * 180 / M_PI << "\t"
                  << "q5: " << q0(8) * 180 / M_PI << "\t"
                  << "q6: " << q0(9) * 180 / M_PI << "\n"
                  << endl;
        robot.setJointPositions(q0);
        pose0 = Pose(robot.getEETransform());
        std::cout << "T0:" << endl
                  << pose0.matrixRep() << endl;
        std::cout << "Pose0:" << endl
                  << pose0 << endl
                  << endl;
        std::cout << "Trajectory time: " << tf << endl;
        
        if (goal->pathType != mars_mot_plan::PoseTrajectoryGoal::PATH_TYPE_CLOSED_PATH)
        {
            ROS_ERROR("Invalid path type received. Only closed path type is valid for circle trajectory.");
            as.setAborted();
            return;
        }

        /*
            **********************************************************
            **************** Pose trajectory planning ****************
            **********************************************************
        */

        std::cout << "Planing trajectory:" << endl;
        desiredTraj = new SineWavePathTrajectory(pose0, 0.0, tf, tf*0.125, 3.5, 3, 0.4);        

        // desiredTraj = PoseIterTrajectory(pose0, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
        //                                  posef, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
        //                                  0.0, tf);

        // Straight line path
        // desiredTraj = new LinePathTrajectory(pose0, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
        //                            posef, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(),
        //                            0.0, tf);

        // Elliptic path
        // desiredTraj = new EllipticPathTrajectory(pose0, posef, 0.0, tf);

        /*
            **********************************************************
            **************** Step size variation law  ****************
            **********************************************************
        */
        trans.clear();
        calcTransVars(tf, 0.2);

        // Clear and initialize variables that are reused
        ts = 1.0 / freq; // Needed for simulation
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
        // Copy the initial values of the joint positions
        q.push_back(q0);
        while (trajDuration <= tf)
        {
            // check that preempt has not been requested by the client
            if (as.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name.c_str());
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
            dP = UR5dP;
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
            poseError.push_back(Pose::pose_diff(desiredTraj->getPose(trajDuration), pose.at(k)));

            // Compute the particular and homogeneous solutions
            // particular solution
            JBar = robot.getJacobianBar();
            JBarW = JBar * Wmatrix;
            invJBarW = pinv(JBarW);
            partSol = invJBarW * (desiredTraj->getVel(trajDuration) + wError * poseError.at(k));
            partSol = Wmatrix * partSol;
            //cout << "invJBarW: " << invJBarW << endl;

            // Compute the step size transition
            trans.push_back(stepSizeTrans(trajDuration));

            // homogeneous solution
            homSol = trans.at(k) * (Id - invJBarW * JBarW) * Wmatrix * dP;
            homSol = Wmatrix * homSol;

            // Compute the maximum and minimum step size for the homogeneous solution
            robot.getSelfMotionLims(partSol, homSol, maxAlpha, minAlpha);
            if (maxAlpha < minAlpha)
            {
                ROS_ERROR("Could not acheive task that complies with joint velocity limits. ");
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
                trajDuration += ts;
                q.push_back(q.at(k) + dq.at(k) * ts);
                // Publish the feedback
                feedback.currTime = trajDuration;
                feedback.currPose = Pose::poseToGeomMsgPose(pose.at(k));
                as.publishFeedback(feedback);
            }
            k++;
        }
        // Substract sampling time from time
        trajDuration -= ts;
        double finalTimeDiff = tf - trajDuration;
        cout << "tf: " << tf << endl;
        cout << "trajDuration: " << trajDuration << endl;
        cout << "Time diff: " << finalTimeDiff << endl;

        if (abs(finalTimeDiff) <= 1.5*ts)
        {
            std::cout << "Motion planning completed. Number of iterations: " << k << endl;
            ROS_INFO("%s: Succeeded", action_name.c_str());
            result.finalTime = time.back();
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
                  << desiredTraj->getPose(tf) << endl;
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

        // Plot evolution of variables
        try
        {
            plotResults(time.size());
        }
        catch (const std::exception &ex)
        {
            cout << "Could not plot results. " << endl
                 << ex.what() << endl;
        }
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
    Trajectory *desiredTraj;
    // Step size trasition variables
    std::vector<double> stepVarCoeff;
    double stepTb1;
    double stepTb2;
    std::vector<double> trans;

    std::vector<VectorXd> q, eta, dq; // Joint position and velocities
    std::vector<Pose> pose;           // pose at each iteration
    std::vector<VectorXd> poseError;  // pose error at each tireation
    std::vector<double> time, wMeasure, MMmanip, UR5manip;

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
    ros::Time startTime, nowTime, prevTime;

    // Joint states
    double L, R, x, y, phi, linVel, angVel;
    sensor_msgs::JointState joint_state;
    VectorXd currJointPos;
    tf::TransformListener listener;
    tf::StampedTransform mob_plat_base_transform;

    void calcTransVars(double tf, double blendPerc)
    {
        stepTb1 = tf * blendPerc;
        stepTb2 = tf * (1 - blendPerc);
        stepVarCoeff.clear();
        stepVarCoeff.resize(3);
        stepVarCoeff.at(0) = 10.0 / pow(stepTb1, 3);
        stepVarCoeff.at(1) = -15.0 / pow(stepTb1, 4);
        stepVarCoeff.at(2) = 6.0 / pow(stepTb1, 5);
    }
    double stepSizeTrans(double t)
    {
        double stepSize;
        if (t < stepTb1)
        {
            stepSize = stepVarCoeff.at(0) * pow(t, 3) + stepVarCoeff.at(1) * pow(t, 4) + stepVarCoeff.at(2) * pow(t, 5);
        }
        if (t >= stepTb1 && t <= stepTb2)
        {
            stepSize = 1.0;
        }
        if (t > stepTb2)
        {
            double deltaTime = t - stepTb2;
            stepSize = 1.0 - stepVarCoeff.at(0) * pow(deltaTime, 3) - stepVarCoeff.at(1) * pow(deltaTime, 4) - stepVarCoeff.at(2) * pow(deltaTime, 5);
        }
        return stepSize;
    }

    void plotResults(int k)
    {
        /*
            **********************************************************
            ********** Plot the obtained trajectory data  ************
            **********************************************************
        */
        double NObt = k;
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

        // Figure (1)
        // Manipulability plots
        plt::figure_size(1600, 900);
        plt::subplot(2, 2, 1);
        plt::named_plot("w_MM", time, wMeasure);
        plt::named_plot("w_{p+a}", time, MMmanip);
        plt::named_plot("w_{a}", time, UR5manip);
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();
        // Mob plat trajectory
        plt::subplot(2, 2, 2);
        std::vector<double> mobPlatStart_x(2, mobPlat_x.at(0)),
            mobPlatEnd_x(2, mobPlat_x.at(NObt - 1)),
            mobPlatStart_y(2, mobPlat_y.at(0)),
            mobPlatEnd_y(2, mobPlat_y.at(NObt - 1));
        plt::named_plot("Traj", mobPlat_x, mobPlat_y);
        plt::named_plot("Start Pos", mobPlatStart_x, mobPlatStart_y, "o");
        plt::named_plot("Final Pos", mobPlatEnd_x, mobPlatEnd_y, "s");
        plt::grid(true);
        plt::ylabel("y(m)");
        plt::xlabel("x(m)");
        plt::legend();
        // Position and Orientation errors
        plt::subplot(2, 2, 3);
        plt::named_plot("e_{Px}", time, epos_x);
        plt::named_plot("e_{Py}", time, epos_y);
        plt::named_plot("e_{Pz}", time, epos_z);
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();
        plt::subplot(2, 2, 4);
        plt::named_plot("e_{Ox}", time, eor_x);
        plt::named_plot("e_{Oy}", time, eor_y);
        plt::named_plot("e_{Oz}", time, eor_z);
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();

        // Figure (2)
        // Mob plat velocities
        plt::figure_size(1600, 900);
        plt::subplot(2, 3, 1);
        plt::named_plot("v(m/s)", time, mobPlat_v, "b");
        plt::named_plot("w(rad/s)", time, mobPlat_w, "r");
        plt::named_plot("v-", time, std::vector<double>(NObt, -1 * dqlimits(0)), "--b");
        plt::named_plot("v+", time, std::vector<double>(NObt, dqlimits(0)), "-.b");
        plt::named_plot("w-", time, std::vector<double>(NObt, -1 * dqlimits(1)), "--r");
        plt::named_plot("w+", time, std::vector<double>(NObt, dqlimits(1)), "-.r");
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();
        // Prismatic joint
        plt::subplot(2, 3, 2);
        plt::named_plot("z(m)", time, z_pj, "y");
        plt::named_plot("dz(m/s)", time, dz_pj, "c");
        plt::named_plot("z-", time, std::vector<double>(NObt, qlimits(2, 0)), "--y");
        plt::named_plot("z+", time, std::vector<double>(NObt, qlimits(2, 1)), "-.y");
        plt::named_plot("dz-", time, std::vector<double>(NObt, -1 * dqlimits(2)), "--c");
        plt::named_plot("dz+", time, std::vector<double>(NObt, dqlimits(2)), "-.c");
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();
        // UR5 Joints 1 and 2
        plt::subplot(2, 3, 3);
        plt::named_plot("qa1", time, qa1, "b");
        plt::named_plot("qa2", time, qa2, "r");
        plt::named_plot("qa1-", time, std::vector<double>(NObt, qlimits(3, 0)), "--b");
        plt::named_plot("qa1+", time, std::vector<double>(NObt, qlimits(3, 1)), "-.b");
        plt::named_plot("qa2-", time, std::vector<double>(NObt, qlimits(4, 0)), "--r");
        plt::named_plot("qa2+", time, std::vector<double>(NObt, qlimits(4, 1)), "-.r");
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();
        // UR5 Joint 3 and 4
        plt::subplot(2, 3, 4);
        plt::named_plot("qa3", time, qa3, "y");
        plt::named_plot("qa4", time, qa4, "m");
        plt::named_plot("qa3-", time, std::vector<double>(NObt, qlimits(5, 0)), "--y");
        plt::named_plot("qa3+", time, std::vector<double>(NObt, qlimits(5, 1)), "-.y");
        plt::named_plot("qa4-", time, std::vector<double>(NObt, qlimits(6, 0)), "--m");
        plt::named_plot("qa4+", time, std::vector<double>(NObt, qlimits(6, 1)), "-.m");
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();
        // UR5 Joint 5 and 6
        plt::subplot(2, 3, 5);
        plt::named_plot("qa5", time, qa5, "g");
        plt::named_plot("qa6", time, qa6, "c");
        plt::named_plot("qa5-", time, std::vector<double>(NObt, qlimits(7, 0)), "--g");
        plt::named_plot("qa5+", time, std::vector<double>(NObt, qlimits(7, 1)), "-.g");
        plt::named_plot("qa6-", time, std::vector<double>(NObt, qlimits(8, 0)), "--c");
        plt::named_plot("qa6+", time, std::vector<double>(NObt, qlimits(8, 1)), "-.c");
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();
        // UR5 Joint velocities
        plt::subplot(2, 3, 6);
        plt::named_plot("dqa1", time, dqa1, "b");
        plt::named_plot("dqa2", time, dqa2, "r");
        plt::named_plot("dqa3", time, dqa3, "y");
        plt::named_plot("dqa4", time, dqa4, "m");
        plt::named_plot("dqa5", time, dqa5, "g");
        plt::named_plot("dqa6", time, dqa6, "c");
        plt::grid(true);
        plt::xlabel("time");
        plt::legend();
        plt::show();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mars_mot_plan_sim_server");

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
