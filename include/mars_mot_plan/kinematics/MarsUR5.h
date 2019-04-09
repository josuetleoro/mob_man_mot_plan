#ifndef MARSUR5_H
#define MARSUR5_H

#include <math.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "mars_mot_plan/kinematics/DualQuat.h"
using namespace std;

class MarsUR5
{
private:
    // lenght of UR5 links
	double l1, l2, l3, l4, l5, l6;

    // Position of UR5 base with respect to the center of the wheels
    double a, b;

    // Directions and points for each joint
    // Points and Directions of each joint
	Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;
	Eigen::Vector3d d1, d2, d3, d4, d5, d6, d7, d8;

	// Moments of the axes
	Eigen::Vector3d m1, m2, m3, m4, m5, m6, m7, m8;

	// Dual-quaternion representation of each axis of the UR5
	DualQuat DQ1, DQ2, DQ3, DQ4, DQ5, DQ6;
    // Dual Quaternion for mobile platform
    DualQuat DQmp;
	// Dual-quaternion representation in plucker coordinates of the tool axes
    DualQuat DQL6;	//approach direction
	DualQuat DQL7;	//orientation direction

	// Current states
	Eigen::VectorXd q;   	//Joints
	Eigen::Matrix4d T0e;	//End effector transformation matrix
	Eigen::MatrixXd JBar;	//Jacobian with non-holonomic constraints
	Eigen::MatrixXd Ja;		//UR5 Jacobian
	bool JBarEvaluated, JaEvaluated;		//Helper to avoid evaluating JBar multiple times
	/**
	 * Evaluate the Jacobian of the complete Mobile manipulator system 
	 * with non-holonomic constraints included
	 */
	void evalJBar();
	/**
	 * Evaluate the Jacobian of the robot arm alone
	 */
	void evalJa();

	// For manipulability gradient
	Eigen::MatrixXd dJdq3, dJdq5, dJdq6, dJdq7, dJdq8, dJdq9;	//Derivative of JacobianBar with respect to the joints
	Eigen::MatrixXd dJadq5, dJadq6, dJadq7, dJadq8, dJadq9;		//Derivative of UR5J with respect to the joints
	double w, wa;			 	//Mars manip and arm manip
	Eigen::VectorXd dP, dPa; 	//gradient of Mars manip and arm manip
	MatrixXd JBart, JJt, inv_JJt, inv_Ja, dpAux; //Auxiliar matrices
	void evalJacobians();
	void evalJacobiansDer();

	// For joint limits avoidance and collision avoidance
	MatrixXd qlimits;
	VectorXd prevGradHJLim, prevGradPElbow, prevGradPWrist;
	VectorXd gradPElbow, gradPWrist;
	double gradH, jLimBeta;
	double elbowPosZ, elbowSafeDist, wristSafeDist;
	VectorXd JElbow, JWrist;
	double deltaP_deltad;
	VectorXd deltad_deltaq, gradPDif;	// Auxiliar variables for gradient calculation

	// For joint velocities limit avoidance
	VectorXd dqlimits;
public:
	MarsUR5();
	void setJointPositions(const Eigen::VectorXd& q);
    Eigen::Matrix4d getEETransform();
	Eigen::MatrixXd getJacobianBar()
	{
		evalJBar();
		return JBar;
	}
	Eigen::MatrixXd getJacobianUR5()
	{
		evalJa();
		return Ja;
	}

	/**
	 * Calculate the manipulabilities of the complete mobile
	 * manipulator system and the robot arm alone and their
	 * corresponding gradients.
	 */
	void getManipGrads(VectorXd &MMdP, double &MMmanip, VectorXd &UR5dP, double &UR5manip);

	/**
	 * Calculates the inverse of the weighting matrix for joint
	 * limits avoidance.
	 */
	void getJLimGrad(MatrixXd &wJLim);

	/**
	 * Calculates the inverse of the elbow collision weighting matrix
	 */
	void getJLimWeight(MatrixXd &wJLim);

	/**
	 * Calculates the inverse of the elbow collision weighting matrix
	 * and also return the distance of the elbow to mobile platform.
	 * The variables rho, alpha and beta are shape paremeters.
	 * rho: controls the amplitude.
	 * alpha and beta: control the decay rate.
	 */
	void getElbowColWeight(double rho, double alpha, double beta, MatrixXd &wColElbow, double &distance);

	/**
	 * Calculates the inverse of the wrist collision weighting matrix,
	 * returns the distance of the wrist to the mobile platform, and
	 * also returns the position of the wrist.
	 * The variables rho, alpha and beta are shape paremeters.
	 * rho: controls the amplitude.
	 * alpha and beta: control the decay rate.
	 */
	void getWristColWeight(double rho, double alpha, double beta, MatrixXd &wColWrist, double &distance, Vector3d &wristPos);

	/**
	 * Calculates the maximum and minimum magnitudes of the self motion.
	 * The inputs are the control input (particular solution) and 
	 * self motion (homogeneous solution) vectors.
	 * The output are the maximum and minimum magnitudes.
	 */
	void getSelfMotionLims(const VectorXd &dqp, const VectorXd &dqh, double &maxMag, double &minMag);

	/**
	 * Calculates the inverse of the velocities normalization matrix using the joint maximum velocities.
	 */
	MatrixXd getVelsNormMatrix();
};

#endif
