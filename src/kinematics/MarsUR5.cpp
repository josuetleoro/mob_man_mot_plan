#include "mars_mot_plan/MarsUR5.h"
#include "mars_mot_plan/PadenKahan.h"

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
    this->dqL6 = DualQuat(Quat(0, this->d6), Quat(0, this->m6));
    this->dqL7 = DualQuat(Quat(0, this->d7), Quat(0, this->m7));

    //Initialize the values of the Transformaion matrix
    this->T0e = Eigen::Matrix4d::Zero();
    this->T0e(3, 3) = 1.0;
}

Eigen::Matrix4d MarsUR5::forwardKin(const Eigen::VectorXd &q)
{
    if (q.size() != 10) //The size of the
    {
        throw "The angles of the 10 joints are required";
    }

    //Dual quaternion rigid body transformmations for mobile platform and prismatic joint
    Vector3d t(q(0), q(1), q(3));
    //Translation and rotation quaternions
    Quat qrot = Quat::rotationOperator(q(2), Vector3d(0, 0, 1));
    Quat qt(0, t);
    //Composite operator
    DualQuat dqmp(qrot, 0.5*qt*qrot);

    /*cout << "qrot:" << endl << qrot << endl;
    cout << "qt:" << endl << qt << endl;*/
    cout << "dqmp:" << endl << dqmp << endl;
    
    //Dual quaternion rigid body trasnformations for UR5
    dQ1 = DualQuat::rigidTransf(q(4), d1, m1);
    dQ2 = DualQuat::rigidTransf(q(5), d2, m2);
    dQ3 = DualQuat::rigidTransf(q(6), d3, m3);
    dQ4 = DualQuat::rigidTransf(q(7), d4, m4);
    dQ5 = DualQuat::rigidTransf(q(8), d5, m5);
    dQ6 = DualQuat::rigidTransf(q(9), d6, m6);

    cout << "dq1: " << endl << dQ1 << endl;
    cout << "dq2: " << endl << dQ2 << endl;
    cout << "dq3: " << endl << dQ3 << endl;
    cout << "dq4: " << endl << dQ4 << endl;
    cout << "dq5: " << endl << dQ5 << endl;
    cout << "dq6: " << endl << dQ6 << endl;

    //******************** Rigid Transfomation ******************//
    DualQuat Qtotal = dqmp * dQ1 * dQ2 * dQ3 * dQ4 * dQ5 * dQ6;
    DualQuat Qtotalc = Qtotal.conj();

    cout << "Qtotal: " << endl << Qtotal << endl;

    //Rotate L6 and L7 using the rigid transformation operator
    DualQuat newQL6 = Qtotal * dqL6 * Qtotalc;
    DualQuat newQL7 = Qtotal * dqL7 * Qtotalc;


    /*cout << "p6: " << endl << p6 << endl;
    cout << "d6: " << endl << d6 << endl;
    cout << "p7: " << endl << p7 << endl;
    cout << "d7: " << endl << d7 << endl;*/
    cout << "dqL6: " << endl << dqL6 << endl;
    cout << "newQL6: " << endl << newQL6 << endl;
    cout << "dqL7: " << endl << dqL7 << endl;
    cout << "newQL7: " << endl << newQL6 << endl;

    cout << "p1: " << endl << p1 << endl;
    cout << "d1: " << endl << d1 << endl;
    cout << "p2: " << endl << p2 << endl;
    cout << "d2: " << endl << d2 << endl;
    cout << "p3: " << endl << p3 << endl;
    cout << "d3: " << endl << d3 << endl;
    cout << "p4: " << endl << p4 << endl;
    cout << "d4: " << endl << d4 << endl;
    cout << "p5: " << endl << p5 << endl;
    cout << "d5: " << endl << d5 << endl;
    cout << "p6: " << endl << p6 << endl;
    cout << "d6: " << endl << d6 << endl;
    cout << "p7: " << endl << p6 << endl;
    cout << "d7: " << endl << d6 << endl;

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
