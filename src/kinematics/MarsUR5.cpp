#include "mars_mot_plan/kinematics/MarsUR5.h"
#include "mars_mot_plan/kinematics/PadenKahan.h"

#define _USE_MATH_DEFINES // for the PI constant
#include <math.h>
using namespace std;
using namespace Eigen;

MarsUR5::MarsUR5()
{
    //Lenghts of UR5 links
    this->l1 = 0.089159;
    this->l2 = 0.425;
    this->l3 = 0.39225;
    this->l4 = 0.10915;
    this->l5 = 0.09465;
    this->l6 = 0.0823;

    //Position of UR5 base with respect to the center of the wheels
    this->a = 0.011;
    this->b = 0.0762 + 0.48296; //(wheels radius + center of wheels to prism joint)

    //Points and Directions for UR5 joints
    this->p1 << this->a, 0, this->b + this->l1;
    this->p2 << this->p1;
    this->p3 << this->p2 + Vector3d(this->l2, 0, 0);
    this->p4 << this->p3 + Vector3d(this->l3, 0, 0);
    this->p5 << this->p4 + Vector3d(0, this->l4, 0);
    this->p6 << this->p5 + Vector3d(0, this->l6, -1*this->l5);
    this->d1 << 0, 0, 1;
    this->d2 << 0, 1, 0;
    this->d3 << 0, 1, 0;
    this->d4 << 0, 1, 0;
    this->d5 << 0, 0, -1;
    this->d6 << 0, 1, 0;

    //Auxiliar line for tool position and orientation
    this->p7 << this->p6;
    this->d7 << 0, 0, 1;
    
    //Get the moments of the axes for the plucker coordinates
    this->m1 = this->p1.cross(d1);
    this->m2 = this->p2.cross(d2);
    this->m3 = this->p3.cross(d3);
    this->m4 = this->p4.cross(d4);
    this->m5 = this->p5.cross(d5);
    this->m6 = this->p6.cross(d6);
    this->m7 = this->p7.cross(d7);

    //Dual quaternion representation of the needed axes in Plucker Coordinates
    this->DQL6 = DualQuat(Quat(0, this->d6), Quat(0, this->m6));
    this->DQL7 = DualQuat(Quat(0, this->d7), Quat(0, this->m7));

    //Initialize the current states
    this->q = VectorXd::Zero(10);
    this->T0e = Eigen::Matrix4d::Zero();
    this->T0e(3, 3) = 1.0;
    this->JBar = MatrixXd::Zero(6,9);
    this->Ja = MatrixXd::Zero(6,6);
    JBarEvaluated = false;
    JaEvaluated = false;

    // Initialize the varibales for manipulability calculations to zero (To reserve the memory)
    dJdq3 = JBar, dJdq5 = JBar, dJdq6 = JBar, dJdq7 = JBar, dJdq8 = JBar, dJdq9 = JBar;
    dJadq5 = JBar, dJadq6 = JBar, dJadq7 = JBar, dJadq8 = JBar, dJadq9 = JBar;
	dP = dPa = VectorXd::Zero(10);

    // Initialize variables for joint limits avoidance and collision avoidance
    qlimits = MatrixXd::Zero(9,2);
    qlimits(0,0) = -1*Infinity; qlimits(0,1) = Infinity;
    qlimits(1,0) = -1*Infinity; qlimits(1,1) = Infinity;
    qlimits(2,0) = 0.0; qlimits(2,1) = 0.25;
    qlimits(3,0) = -100*M_PI/180; qlimits(3,1) = 1*M_PI/180;
    qlimits(4,0) = -90*M_PI/180; qlimits(4,1) = 15*M_PI/180;
    qlimits(5,0) = 0*M_PI/180; qlimits(5,1) = 180*M_PI/180;
    qlimits(6,0) = -2*M_PI; qlimits(6,1) = 2*M_PI;
    qlimits(7,0) = -2*M_PI; qlimits(7,1) = 2*M_PI;
    qlimits(8,0) = -2*M_PI; qlimits(8,1) = 2*M_PI;
    jLimBeta = 50;
    prevGradHJLim = VectorXd::Zero(9);

    // Initialize the variables for self-collision avoidance
    elbowSafeDist = 0.6; wristSafeDist = 0.37;
    JElbow = VectorXd::Zero(9);
    JWrist = VectorXd::Zero(9);
    prevGradPElbow = VectorXd::Zero(9);
    prevGradPWrist = VectorXd::Zero(9);

    // Initialize the variables for joint velocities limit avoidance
    dqlimits = VectorXd::Zero(9);
    dqlimits(0) = 0.3;      //default 0.6
    dqlimits(1) = M_PI/4;   //default pi/2
    dqlimits(2) = 0.02;
    dqlimits(3) = M_PI;
    dqlimits(4) = M_PI;
    dqlimits(5) = M_PI;
    dqlimits(6) = M_PI;
    dqlimits(7) = M_PI;
    dqlimits(8) = M_PI;
}

void MarsUR5::setJointPositions(const Eigen::VectorXd& q)
{
    if (q.size() != 10) //The size of the
    {
        throw "The angles of the 10 joints are required";
    }
    this->q = q;
    JBarEvaluated = false;
    JaEvaluated = false;
}

Eigen::Matrix4d MarsUR5::getEETransform()
{
    //Dual quaternion rigid body transformmations for mobile platform and prismatic joint
    Vector3d t(q(0), q(1), q(3));
    //Translation and rotation quaternions
    Quat Qrot = Quat::rotationOperator(q(2), Vector3d(0, 0, 1));
    Quat Qt(0, t);
    //Composite operator
    DualQuat DQmp(Qrot, 0.5*Qt*Qrot);
   
    //Dual quaternion rigid body trasnformations for UR5
    DQ1 = DualQuat::rigidTransf(q(4), d1, m1);
    DQ2 = DualQuat::rigidTransf(q(5), d2, m2);
    DQ3 = DualQuat::rigidTransf(q(6), d3, m3);
    DQ4 = DualQuat::rigidTransf(q(7), d4, m4);
    DQ5 = DualQuat::rigidTransf(q(8), d5, m5);
    DQ6 = DualQuat::rigidTransf(q(9), d6, m6);

    //******************** Rigid Transfomation ******************//
    DualQuat DQtotal = DQmp * DQ1 * DQ2 * DQ3 * DQ4 * DQ5 * DQ6;
    DualQuat DQtotalc = DQtotal.conj();

    //Rotate L6 and L7 using the rigid transformation operator
    DualQuat newQL6 = DQtotal * DQL6 * DQtotalc;
    DualQuat newQL7 = DQtotal * DQL7 * DQtotalc;

    //Get the position from intersection of L6 and L7 and the directions from the vectors
    Eigen::Vector3d pos = DualQuat::linesIntPlucker(newQL6, newQL7); //Position
    Eigen::Vector3d approach = newQL6.getPrim().getV();      //Approach direction
    Eigen::Vector3d orientation = newQL7.getPrim().getV();        //Orientation direction
    Eigen::Vector3d normal = orientation.cross(approach);

    //Store the vectors in the T0e matrix
    this->T0e.col(0).head(3) = normal;
    this->T0e.col(1).head(3) = orientation;
    this->T0e.col(2).head(3) = approach;
    this->T0e.col(3).head(3) = pos;
    return this->T0e;
}

void MarsUR5::getManipGrads(VectorXd &MMdP, double &MMmanip, VectorXd &UR5dP, double &UR5manip)
{
    // Make sure the Jacobians and their derivatives have been evaluated
    evalJacobians();
    evalJacobiansDer();

    // Manipulability and gradient of JBar
    JBart = JBar.transpose();
    JJt = JBar*JBart;
    w = sqrt(JJt.determinant());
    double w_2 = w/2.0;
    inv_JJt = JJt.inverse();
    MMmanip = w;
    dpAux = inv_JJt * (dJdq3*JBart + JBar*(dJdq3.transpose()));
    dP(2) = w_2 * dpAux.trace();
    dpAux = inv_JJt * (dJdq5*JBart + JBar*(dJdq5.transpose()));
    dP(4) = w_2 * dpAux.trace();
    dpAux = inv_JJt * (dJdq6*JBart + JBar*(dJdq6.transpose()));
    dP(5) = w_2 * dpAux.trace();
    dpAux = inv_JJt * (dJdq7*JBart + JBar*(dJdq7.transpose()));
    dP(6) = w_2 * dpAux.trace();
    dpAux = inv_JJt * (dJdq8*JBart + JBar*(dJdq8.transpose()));
    dP(7) = w_2 * dpAux.trace();
    dpAux = inv_JJt * (dJdq9*JBart + JBar*(dJdq9.transpose()));
    dP(8) = w_2 * dpAux.trace();
    MMdP = dP;

    // Manipulability and gradient of Ja
    wa = abs(Ja.determinant());
    inv_Ja = Ja.inverse();
    UR5manip = wa;
    dpAux = inv_Ja*dJadq5;
    dPa(4) = wa * dpAux.trace();
    dpAux = inv_Ja*dJadq6;
    dPa(5) = wa * dpAux.trace();
    dpAux = inv_Ja*dJadq7;
    dPa(6) = wa * dpAux.trace();
    dpAux = inv_Ja*dJadq8;
    dPa(7) = wa * dpAux.trace();
    dpAux = inv_Ja*dJadq9;
    dPa(8) = wa * dpAux.trace();
    UR5dP = dPa;
}

void MarsUR5::getJLimWeight(MatrixXd &wJLim)
{
    wJLim = MatrixXd::Identity(9, 9);
    double gradH, gradHDif;
    for (int i = 2; i < 9; i++)
    {
        // q(i + 1) is used because we have 9 joint limits for the reduced coordiantes
        // but 10 joints for the generalized coordinates
        gradH = abs(pow(qlimits(i, 1) - qlimits(i, 0), 2) * (2 * q(i + 1) - qlimits(i, 1) - qlimits(i, 0)) / (4 * pow(qlimits(i, 1) - q(i + 1), 2) * pow(q(i + 1) - qlimits(i, 0), 2))) / jLimBeta;
        gradHDif = gradH - prevGradHJLim(i);

        /*cout << "i: " << i << endl;
        cout << "q: " << q.transpose() << endl;
        cout << "q(i+1): " << q(i+1) << endl;
        cout << "gradH: " << gradH << endl;
        cout << "gradHDif: " << gradHDif << endl;
        cout << "qlimit_low: " << qlimits(i, 0) << endl;
        cout << "qlimit_high: " << qlimits(i, 1) << endl;
        cout << endl;*/

        prevGradHJLim(i) = gradH;
        if (gradHDif >= 0)
            wJLim(i, i) = 1 / sqrt(1 + gradH);
        else
            wJLim(i, i) = 1;
    }
}

void MarsUR5::getElbowColWeight(double rho, double alpha, double beta, MatrixXd &wColElbow, double &distance)
{
    wColElbow = MatrixXd::Identity(9, 9);

    /* Calculate the elbow position and Jacobian
       The Jacobian is with respect to a point on the center of the wheels
       projected to the floor. This frame allows to define the vector pa_pb just
       as the z position of the elbow minus the safe distance
    */
    elbowPosZ = 0.645359 - 0.425 * sin(q(5)) + q(3);
    JElbow << 0.0, 0.0, 1.0, 0.0, -0.425 * cos(q(5)), 0.0, 0.0, 0.0, 0.0;

    //Calculate the collision gradient
    double pa_pb = elbowPosZ - elbowSafeDist;
    distance = pa_pb;
    double dist = abs(pa_pb); //absolute of the distance
    deltad_deltaq = 1.0/dist*(JElbow.transpose()*pa_pb);
    deltaP_deltad=-1*rho*exp(-1*alpha*dist)*pow(dist,-1*beta)*(beta/dist+alpha);
    gradPElbow = (deltaP_deltad*deltad_deltaq).cwiseAbs();
    gradPDif = gradPElbow - prevGradPElbow;
    //cout << "gradPElbow:" << endl << gradPElbow.transpose() << endl;
    for (int i = 2; i < 5; i++)
    {
        if (gradPDif(i) >= 0)
            wColElbow(i,i) = 1/sqrt(1 + gradPElbow(i));
        else
            wColElbow(i,i) = 1;
    }
    prevGradPElbow = gradPElbow;
}

void MarsUR5::getWristColWeight(double rho, double alpha, double beta, MatrixXd &wColWrist, double &distance, Vector3d &wristPos)
{
    wColWrist = MatrixXd::Identity(9, 9);

    /* Calculate the wrist position and Jacobian
       The Jacobian is with respect to a point on the center of the wheels
       projected to the floor. This frame allows to define the vector pa_pb just
       as the x position of the elbow minus the safe distance
    */
    wristPos << 0.39225e0 * cos(q(4)) * cos(q(5)) * cos(q(6)) - 0.39225e0 * cos(q(4)) * sin(q(5)) * sin(q(6)) - 0.49e-1 + 0.425e0 * cos(q(4)) * cos(q(5)), 0.39225e0 * sin(q(4)) * cos(q(5)) * cos(q(6)) - 0.39225e0 * sin(q(4)) * sin(q(5)) * sin(q(6)) + 0.425e0 * sin(q(4)) * cos(q(5)), -0.39225e0 * sin(q(5)) * cos(q(6)) - 0.39225e0 * cos(q(5)) * sin(q(6)) + 0.645359e0 - 0.425e0 * sin(q(5)) + q(3);
    //cout << "wristPos: " << endl << wristPos.transpose() << endl;
    JWrist << 0.0, 0.0, 0.0, -0.1961250000e0 * sin(q(4) + q(5) + q(6)) - 0.1961250000e0 * sin(q(4) - q(5) - q(6)) - 0.2125000000e0 * sin(q(4) + q(5)) - 0.2125000000e0 * sin(q(4) - q(5)), -0.1961250000e0 * sin(q(4) + q(5) + q(6)) + 0.1961250000e0 * sin(q(4) - q(5) - q(6)) - 0.2125000000e0 * sin(q(4) + q(5)) + 0.2125000000e0 * sin(q(4) - q(5)), -0.1961250000e0 * sin(q(4) + q(5) + q(6)) + 0.1961250000e0 * sin(q(4) - q(5) - q(6)), 0.0, 0.0, 0.0;
    //cout << "JWrist: " << endl << JWrist.transpose() << endl;

    // Calculate the collision gradient
    double pa_pb = wristPos(0) - wristSafeDist;
    distance = pa_pb;
    double dist = abs(pa_pb); //absolute of the distance

    // The collision weighting matrix is only evaluated if the wrist height
    // is below a safe distance, otherwise the identity matrix is used
    if (wristPos(2) > 0.5)
        return;

    deltad_deltaq = 1.0/dist*(JWrist.transpose()*pa_pb);
    deltaP_deltad=-1*rho*exp(-1*alpha*dist)*pow(dist,-1*beta)*(beta/dist+alpha);
    gradPWrist = (deltaP_deltad*deltad_deltaq).cwiseAbs();
    gradPDif = gradPWrist - prevGradPWrist;
    //cout << "gradPWrist:" << endl << gradPWrist.transpose() << endl;
    for (int i = 3; i < 6; i++)
    {
        if (gradPDif(i) >= 0)
            wColWrist(i,i) = 1/sqrt(1 + gradPWrist(i));
        else
            wColWrist(i,i) = 1;
    }
    prevGradPWrist = gradPWrist;
}

void MarsUR5::getSelfMotionLims(const VectorXd &dqp, const VectorXd &dqh, double &maxMag, double &minMag)
{
    double kmax = std::numeric_limits<double>::infinity();
    double kmin = -std::numeric_limits<double>::infinity();
    double kmax_aux, kmin_aux;
    for (int i = 0; i < 9; i++)
    {
        double k1 = (dqlimits(i)-dqp(i))/dqh(i);
        double k2 = (-dqlimits(i)-dqp(i))/dqh(i);
        kmax_aux = std::max((dqlimits(i)-dqp(i))/dqh(i),(-dqlimits(i)-dqp(i))/dqh(i));
        kmin_aux = std::min((dqlimits(i)-dqp(i))/dqh(i),(-dqlimits(i)-dqp(i))/dqh(i));
        kmax = std::min(kmax_aux, kmax);
        kmin = std::max(kmin_aux, kmin);
    }
    maxMag = kmax;
    minMag = kmin;
}

MatrixXd MarsUR5::getVelsNormMatrix()
{
    MatrixXd invTq = MatrixXd::Zero(9, 9);
    double max = 0;
    for (int i = 0; i < 9; i++)
    {
        invTq(i, i) = sqrt(dqlimits(i));
        /*if (invTq(i,i) > max)
        {
            max = invTq(i, i);
        }*/
    }
    //invTq = invTq / max;
    return invTq;
}