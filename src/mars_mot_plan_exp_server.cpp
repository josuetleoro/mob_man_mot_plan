#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/server/simple_action_server.h>
#include <mars_mot_plan/PoseTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

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

        // Initialize mobile platform position and orientation
        L = 0.4225;
        R = 0.0762;
        x = 0; y = 0; phi = 0;
        jointStateCurrTime = ros::Time::now();
        jointStatePrevTime = jointStateCurrTime;
        currJointPos = VectorXd(10);

        //Publishers for command velocities
        mob_plat_vel_pub = nh_.advertise<geometry_msgs::Twist>("/mob_plat/cmd_vel", 10);    
        prism_vel_pub = nh_.advertise<std_msgs::Float32>("/prism_joint/cmd_vel", 10);
        ur_script_pub = nh_.advertise<std_msgs::String>("/ur_driver/URScript", 10);
        ur5_accel = 2.0, ur5_speedj_time = 0.5;

        as.start();
        ROS_INFO_STREAM("Mars motion planning action server started");
        ros::SubscribeOptions ops =
            ros::SubscribeOptions::create<sensor_msgs::JointState>(
                "/joint_states",     // topic name
                10,                   // queue length
                bind(&MarsPoseTrajActionSim::jointStateCB, this, _1),
                ros::VoidPtr(),      // tracked object, we don't need one thus NULL
                &joint_states_queue           // pointer to callback queue object
            );
        // Subscriber for joint states
        joint_state_sub = nh_.subscribe(ops);
         // spawn async spinner with 1 thread, running on the joint_states_queue
        async_spinner = new ros::AsyncSpinner(1, &joint_states_queue);
        // start the spinner
        async_spinner->start();
    }

    ~MarsPoseTrajActionSim(void)
    {
        async_spinner->stop();
    }

    void jointStateCB(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
    {
        jointStateCurrTime = ros::Time::now();
        jointStateTs = (jointStateCurrTime - jointStatePrevTime).toSec();
        jointStatePrevTime = jointStateCurrTime;
        std::vector<double> joint_pos = joint_state_msg->position;
        std::vector<double> joint_vel = joint_state_msg->velocity;

        // Calculate linear and angular velocity of the platform
        double rVel = joint_vel.at(0) * R;
        double lVel = joint_vel.at(1) * R; // Left wheel has opposite sign because of direction of z axis
        linVel = (rVel + lVel) / 2;
        angVel = (rVel - lVel) / L;

        // Store the current joint positions
        mutex.lock();
        // Calculate the mobile platform position based on odometry
        x += jointStateTs * linVel * cos(phi);
        y += jointStateTs * linVel * sin(phi);
        phi += jointStateTs * angVel;
        //cout << "mob_plat_pos: " << x << ", " << y << ", "<< phi << endl;
        currJointPos << x, y, phi, joint_pos.at(2),
            joint_pos.at(3), joint_pos.at(4), joint_pos.at(5),
            joint_pos.at(6), joint_pos.at(7), joint_pos.at(8);
        mutex.unlock();
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
            listener.waitForTransform("/map", "/base_footprint",
                              ros::Time(0), ros::Duration(0.2));
            listener.lookupTransform("/map", "/base_footprint",
                               ros::Time(0), mob_plat_base_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_WARN("Transform from frame map to base_footprint not available yet");
            ROS_ERROR("%s",ex.what());
            return;
        }
        // Set odometry values and copy intial joint positions
        VectorXd q0 = VectorXd(10);
        mutex.lock();
        x = mob_plat_base_transform.getOrigin().x();
        y = mob_plat_base_transform.getOrigin().y();
        phi = tf::getYaw(mob_plat_base_transform.getRotation()); 
        q0 << x, y, phi, currJointPos(3),
            currJointPos(4), currJointPos(5), currJointPos(6),
            currJointPos(7), currJointPos(8), currJointPos(9);
        mutex.unlock();        

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

        posef = Pose(goal->desPose);
        std::cout << "Tf:" << endl
                  << posef.matrixRep() << endl;
        std::cout << "Posef:" << endl
                  << posef << endl
                  << endl;
        std::cout << "Trajectory time: " << tf << endl;
        
        /*std::cout << "Are these parameters OK? (y/n): ";
        char confirm = getchar();
        getchar(); // Get the CR character
        if (confirm != 'y' && confirm != 'Y')
        {
            ROS_INFO("Parameters not confirmed");
            ROS_INFO("%s: Motion planning aborted", action_name.c_str());
            as.setAborted();
            return;
        }*/

        /*cout << "Motion planning started" << endl;
        as.setSucceeded(result);
        return;*/

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

        //Create file to write start and end times
        string timestamps_file_path = ros::package::getPath("mars_mot_plan") + "/data/" + "traj_stamps.csv";
        fstream stamps_fs;
        stamps_fs.open(timestamps_file_path.c_str(), std::fstream::out);

        bool success = true;
        int k = 0;
        trajDuration = 0.0;
        std::cout << "Motion planning started" << endl;
        startTime = ros::Time::now();
        nowTime = startTime;
        prevTime = nowTime;
        // Write the starting timestamp
        stamps_fs  << nowTime.sec << "," << nowTime.nsec << "\n";
        ros::Rate rate(freq);
        // Copy the initial values of the joint positions
        q.push_back(q0);
        while ((tf - trajDuration) >= -1e-2) // Times are not exact so a small error must be allowed
        {
            cout << "Current traj time: " << trajDuration << endl;
            // check that preempt has not been requested by the client
            if (as.isPreemptRequested() || !ros::ok())
            {
                // Stop all the joints
                sendVelCommand(VectorXd::Zero(9));
                ROS_WARN("%s: Preempted", action_name.c_str());                
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
                // Stop all the joints
                sendVelCommand(VectorXd::Zero(9));
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

            cout << "trajDuration: " << trajDuration << endl;
            cout << "WMatrix(3): " << Wmatrix(3,3) << endl;
            cout << "partSol(3): " << partSol(3) << endl;
            cout << "homSol(3): " << homSol(3) << endl;

            // Compute the joint velocities
            eta.push_back(partSol + alphak * homSol);
            dq.push_back(S * eta.at(k));

            // Send velocity commands
            sendVelCommand(eta.at(k));
            // sendVelCommand(VectorXd::Zero(9));

            // Update joint positions for next iteration
            if ((tf - trajDuration) > -1e-2)
            {
                rate.sleep();
                nowTime = ros::Time::now();
                trajDuration = (nowTime - startTime).toSec();
                
                //ts = (nowTime - prevTime).toSec();
                //q.push_back(q.at(k) + dq.at(k) * ts);

                // Store the new joint values
                mutex.lock();
                q.push_back(currJointPos);
                mutex.unlock();

                // Print current joint states
                //cout << "currJointPos: " << currJointPos << endl;

                prevTime = nowTime;
                // Publish the feedback
                feedback.currTime = trajDuration;
                feedback.currPose = Pose::poseToGeomMsgPose(pose.at(k));
                as.publishFeedback(feedback);
            }
            k++;
        }
        // When ending the trajectory make sure zero velocity is send
        sendVelCommand(VectorXd::Zero(9));
        // Write final time stamp
        nowTime = ros::Time::now();
        stamps_fs  << nowTime.sec << "," << nowTime.nsec << "\n";
        stamps_fs.close();

        trajDuration = trajDuration - 1 / freq;
        cout << "Trajectory duration: " << trajDuration << endl;
        cout << "Desired trajectory time: " << tf << endl;

        VectorXd finalPoseError = Pose::pose_diff(desiredTraj.getPose(tf), pose.back());
        double finalPosErrorNorm = finalPoseError.head(3).norm();
        cout << "Final position error norm: " << finalPosErrorNorm << endl;
        cout << "Final orientation error norm: " << finalPoseError.tail(3).norm() << endl;

        if (abs(finalPosErrorNorm) < 0.2)
        {
            ROS_INFO_STREAM("Motion planning completed. Number of iterations: " << k);
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
        /*std::cout << "Desired final pos: " << endl
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
                  << poseError.back().tail(3).norm() << endl;*/

        // Save experimental data to files
        ROS_INFO("Saving data to csv files");
        recordData();
        ROS_INFO("Saving completed");
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<mars_mot_plan::PoseTrajectoryAction> as;
    std::string action_name;
    mars_mot_plan::PoseTrajectoryFeedback feedback;
    mars_mot_plan::PoseTrajectoryResult result;

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

    // Joint states
    double L, R, x, y, phi, linVel, angVel;
    ros::CallbackQueue joint_states_queue;
    ros::Subscriber joint_state_sub;
    ros::AsyncSpinner *async_spinner;
    ros::Time jointStatePrevTime, jointStateCurrTime;
    double jointStateTs;
    boost::mutex mutex;
    VectorXd currJointPos;
    tf::TransformListener listener;
    tf::StampedTransform mob_plat_base_transform;
    geometry_msgs::Vector3 mob_plat_base_trans;
    geometry_msgs::Quaternion mob_plat_base_quat;

    // Velocity comands
    ros::Publisher mob_plat_vel_pub, prism_vel_pub, ur_script_pub;
    geometry_msgs::Twist mob_plat_speed_msg;
    std_msgs::Float32 prism_speed_msg;
    std_msgs::String ur5_speed_msg;
    double ur5_accel, ur5_speedj_time;
   
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

    void sendVelCommand(const VectorXd &vel_cmd)
    {
        // vel_cmd: v, w, dz, qa1, qa2, qa3, qa4, qa5, qa6

        //Mobile platform velocity command
        mob_plat_speed_msg.linear.x = vel_cmd(0); //linear velocity
        mob_plat_speed_msg.angular.z = vel_cmd(1);; //angular velocity

        //Prismatic joint velocity command
        prism_speed_msg.data = vel_cmd(2);

        //UR5 speedj command
		//std::string speedj = "speedj([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]," + std::to_string(ur5_accel) + "," + std::to_string(ur5_speedj_time) + ")";
        std::string speedj = "speedj([" + std::to_string(vel_cmd(3)) + 
                                    "," + std::to_string(vel_cmd(4)) + 
                                    "," + std::to_string(vel_cmd(5)) + 
                                    "," + std::to_string(vel_cmd(6)) + 
                                    "," + std::to_string(vel_cmd(7)) + 
                                    "," + std::to_string(vel_cmd(8)) + "],"
                                    + std::to_string(ur5_accel) + "," + std::to_string(ur5_speedj_time) + ")";
		ur5_speed_msg.data = speedj;

        //Publish the velocities
        mob_plat_vel_pub.publish(mob_plat_speed_msg);
        prism_vel_pub.publish(prism_speed_msg);
		ur_script_pub.publish(ur5_speed_msg);
    }

    void recordData()
    {
        string files_dir = ros::package::getPath("mars_mot_plan") + "/data/";
        string time_file_path = files_dir + "time.csv";
        string manip_file_path = files_dir + "manip.csv";
        string pose_error_file_path = files_dir + "pose_error.csv";
        string joint_pos_file_path = files_dir + "joint_pos.csv";
        string joint_vel_file_path = files_dir + "joint_vel.csv"; // Joint quasi velocities
        string col_dist_file_path = files_dir + "col_dist.csv";

        fstream time_fs, manip_fs, pose_error_fs, joint_pos_fs, joint_vel_fs, col_dist_fs;

        time_fs.open(time_file_path.c_str(), std::fstream::out);
        manip_fs.open(manip_file_path.c_str(), std::fstream::out);
        pose_error_fs.open(pose_error_file_path.c_str(), std::fstream::out);
        joint_pos_fs.open(joint_pos_file_path.c_str(), std::fstream::out);
        joint_vel_fs.open(joint_vel_file_path.c_str(), std::fstream::out); // Joint quasi velocities
        col_dist_fs.open(col_dist_file_path.c_str(), std::fstream::out);   // elbowDist, wristDist, wristHeight

        double N = time.size();
        for (int k = 0; k < N; k++)
        {
            // time
            time_fs << std::to_string(time.at(k)) << "\n";

            // Manipulabilities
            manip_fs << std::to_string(MMmanip.at(k)) << ","
                     << std::to_string(UR5manip.at(k)) << ","
                     << std::to_string(wMeasure.at(k)) << "\n";

            // Pose error
            pose_error_fs << std::to_string(poseError.at(k)(0)) << ","
                          << std::to_string(poseError.at(k)(1)) << ","
                          << std::to_string(poseError.at(k)(2)) << ","
                          << std::to_string(poseError.at(k)(3)) << ","
                          << std::to_string(poseError.at(k)(4)) << ","
                          << std::to_string(poseError.at(k)(5)) << "\n";

            // Joint positions
            joint_pos_fs << std::to_string(q.at(k)(0)) << ","
                         << std::to_string(q.at(k)(1)) << ","
                         << std::to_string(q.at(k)(2)) << ","
                         << std::to_string(q.at(k)(3)) << ","
                         << std::to_string(q.at(k)(4)) << ","
                         << std::to_string(q.at(k)(5)) << ","
                         << std::to_string(q.at(k)(6)) << ","
                         << std::to_string(q.at(k)(7)) << ","
                         << std::to_string(q.at(k)(8)) << ","
                         << std::to_string(q.at(k)(9)) << "\n";

            // Joint velocities
            joint_vel_fs << std::to_string(eta.at(k)(0)) << ","
                         << std::to_string(eta.at(k)(1)) << ","
                         << std::to_string(eta.at(k)(2)) << ","
                         << std::to_string(eta.at(k)(3)) << ","
                         << std::to_string(eta.at(k)(4)) << ","
                         << std::to_string(eta.at(k)(5)) << ","
                         << std::to_string(eta.at(k)(6)) << ","
                         << std::to_string(eta.at(k)(7)) << ","
                         << std::to_string(eta.at(k)(8)) << "\n";

            // Collision distances
            col_dist_fs << std::to_string(elbowDistVector.at(k)) << ","
                        << std::to_string(wristDistVector.at(k)) << ","
                        << std::to_string(wristHeightVector.at(k)) << "\n";
        }
        time_fs.close();
        manip_fs.close();
        pose_error_fs.close();
        joint_pos_fs.close();
        joint_vel_fs.close();
        col_dist_fs.close();

        // Joint constraints
        string joint_constraints_file_path = files_dir + "joint_constraints.csv";
        fstream joint_constraints_fs; // first row: joint neg limit, second row: joint pos limit, third row: joint vel limit
        MatrixXd qlimit = robot.getJointLim().transpose();
        VectorXd dqlimit = robot.getJointVelLim();
        IOFormat matrixFormat(StreamPrecision, 0, ",");
        IOFormat vectorFormat(StreamPrecision, 0, ",", ",");
        joint_constraints_fs.open(joint_constraints_file_path.c_str(), std::fstream::out);
        joint_constraints_fs << qlimit.format(matrixFormat) << endl;
        joint_constraints_fs << dqlimit.format(vectorFormat) << endl;
        joint_constraints_fs.close();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mars_mot_plan_exp_server");

    ros::NodeHandle nh_;

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
