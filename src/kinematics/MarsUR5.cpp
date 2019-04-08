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

    //Initialize the elements for manipulability calculations to zero (To reserve the memory)
    dJdq3 = JBar, dJdq5 = JBar, dJdq6 = JBar, dJdq7 = JBar, dJdq8 = JBar, dJdq9 = JBar;
    dJadq5 = JBar, dJadq6 = JBar, dJadq7 = JBar, dJadq8 = JBar, dJadq9 = JBar;
	dP = dPa = VectorXd::Zero(10);
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

void MarsUR5::getManipGradients(VectorXd &MM_dP, double &MM_manip, VectorXd &UR5_dP, double &UR5_manip)
{
    // Make sure the Jacobians and their derivatives have been evaluated
    evalJacobians();
    evalJacobiansDer();

    // Manipulability and gradient of JBar
    MatrixXd JBarT = JBar.transpose();
    MatrixXd JJt = JBar*JBarT;
    double w = sqrt(JJt.determinant());
    double w_2 = w/2.0;
    MatrixXd inv_JJt = JJt.inverse();
    MM_manip = w;
    MM_dP = VectorXd::Zero(10);
    MM_dP(2) = w_2 * (inv_JJt * (dJdq3*JBarT + JBar*(dJdq3.transpose()))).trace();
    MM_dP(4) = w_2 * (inv_JJt * (dJdq5*JBarT + JBar*(dJdq5.transpose()))).trace();
    MM_dP(5) = w_2 * (inv_JJt * (dJdq6*JBarT + JBar*(dJdq6.transpose()))).trace();
    MM_dP(6) = w_2 * (inv_JJt * (dJdq7*JBarT + JBar*(dJdq7.transpose()))).trace();
    MM_dP(7) = w_2 * (inv_JJt * (dJdq8*JBarT + JBar*(dJdq8.transpose()))).trace();
    MM_dP(8) = w_2 * (inv_JJt * (dJdq9*JBarT + JBar*(dJdq9.transpose()))).trace();

    // Manipulability and gradient of Ja
    double wa = abs(Ja.determinant());
    MatrixXd inv_Ja = Ja.inverse();
    UR5_manip = wa;
    UR5_dP = VectorXd::Zero(10);
    UR5_dP(4) = wa * (inv_Ja*dJadq5).trace();
    UR5_dP(5) = wa * (inv_Ja*dJadq6).trace();
    UR5_dP(6) = wa * (inv_Ja*dJadq7).trace();
    UR5_dP(7) = wa * (inv_Ja*dJadq8).trace();
    UR5_dP(8) = wa * (inv_Ja*dJadq9).trace();
}





