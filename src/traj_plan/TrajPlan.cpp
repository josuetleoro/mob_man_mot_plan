#include "mars_mot_plan/traj_plan/TrajPlan.h"
//#include <iostream>

using namespace std;
using namespace Eigen;

void TrajPlan::fithOrderInterp(double qi, double dqi, double ddqi, double qf, double dqf, double ddqf, double ti, double tf, double ts, std::vector<double> &q, std::vector<double> &dq, std::vector<double> &ddq, std::vector<double> &time)
{
    //Calculate the coefficients
    std::vector<double> coef = polynomCoef(qi, dqi, ddqi, qf, dqf, ddqf, ti, tf);

    // Perform the interpolation
    double m = (tf-ti)/ts;
    q.resize(m+1);
    dq.resize(m+1);
    ddq.resize(m+1);
    time.resize(m+1);
    q[0]=qi;
    dq[0]=dqi;
    ddq[0]=ddqi;
    time[0]=ti;
    double tk_plus_1;   
    for (int k=0; k < m; k++)
    {
        //cout << "k+1: " << k+1 << endl;
        // Compute q(k+1), dq(k+1) and ddq(k+1)
        tk_plus_1 = ti + (k+1)*ts;
        time[k+1] = tk_plus_1;        
        q[k+1]=qPol(tk_plus_1,coef);
        dq[k+1]=dqPol(tk_plus_1,coef);
        ddq[k+1]=ddqPol(tk_plus_1,coef);
    }
    /*cout << "size(q): " << q.size() << endl;
    cout << "m: " << m << endl;*/
}

std::vector<double> TrajPlan::polynomCoef(double qi, double dqi, double ddqi, double qf, double dqf, double ddqf, double ti, double tf)
{
    std::vector<double> coef;
    coef.resize(6);

    double T = tf - ti;
    coef[0] = qi;
    coef[1] = dqi;
    coef[2] = 0.5*ddqi;
    coef[3] = (1/(2*pow(T,3)))*(20*(qf-qi)-(8*dqf+12*dqi)*T-(3*ddqf-ddqi)*pow(T,2));
    coef[4] = (1/(2*pow(T,4)))*(30*(qi-qf)+(14*dqf+16*dqi)*T+(3*ddqf-2*ddqi)*pow(T,2));
    coef[5] = (1/(2*pow(T,5)))*(12*(qf-qi)-(6*dqf+dqi)*T-(ddqf-ddqi)*pow(T,2));

    return coef;
}

double TrajPlan::qPol(double t, const std::vector<double> &coef)
{
    return coef[0]+coef[1]*t+coef[2]*t*t+coef[3]*t*t*t+coef[4]*t*t*t*t+coef[5]*t*t*t*t*t;    
}
    
double TrajPlan::dqPol(double t, const std::vector<double> &coef)
{
    return coef[1]+2*coef[2]*t+3*coef[3]*t*t+4*coef[4]*t*t*t+5*coef[5]*t*t*t*t;
}
    
double TrajPlan::ddqPol(double t, const std::vector<double> &coef)
{
    return 2*coef[2]+6*coef[3]*t+12*coef[4]*t*t+20*coef[5]*t*t*t;
}

void TrajPlan::quatPolynomInterp(Quat qi, Vector3d wi, Vector3d dwi, Quat qf, Vector3d wf, Vector3d dwf, double ti, double tf, double ts, std::vector<Quat> &quat, std::vector<Vector3d> &w, std::vector<Vector3d> &dw, std::vector<double> &time)
{
    std::vector<Quat> coef = quatPolynomCoef(qi, wi, dwi, qf, wf, dwf, ti, tf);

    // Perform the interpolation
    double m = (tf - ti)/ts;
    quat.resize(m+1);
    w.resize(m+1);
    dw.resize(m+1);
    time.resize(m+1);
    quat[0]=qi;
    w[0]=wi;
    dw[0]=dwi;
    time[0]=ti;
    //cout << "quat size: " << quat.size() << endl;
    Quat q_k_plus_1, dq_k_plus_1, ddq_k_plus_1, qwk_plus_1, dqwk_plus_1;
    double tk_plus_1, tau, ti_hat = ti;
    for (int k=0; k < m; k++)
    {
        // Compute q(k+1), dq(k+1) and ddq(k+1)
        tk_plus_1 = ti + (k+1)*ts;
        time[k+1] = tk_plus_1;
        q_k_plus_1=quatPol(tk_plus_1,ti,tf,coef);
        dq_k_plus_1=dquatPol(tk_plus_1,ti,tf,coef);
        ddq_k_plus_1=ddquatPol(tk_plus_1,ti,tf,coef);
        
        // Compute w(k+1) and dw(k+1)
        qwk_plus_1=2*dq_k_plus_1*q_k_plus_1.inv();
        dqwk_plus_1=2*ddq_k_plus_1*q_k_plus_1.inv()-2*(qwk_plus_1/4);
        w[k+1]=qwk_plus_1.getV();
        dw[k+1]=dqwk_plus_1.getV();

        // Compute quat(k+1)
        quat[k+1]=q_k_plus_1;
        // Since by convention the quaternion scalar part must be kept positive,
        // multiply the quaternion by -1 when the scalar part is negative.
        if (quat[k+1].getS() < 0)
            quat[k+1]=-1*quat[k+1];
        quat[k+1].normalize();
        /*cout << "norm = " << quat[k+1].norm() << endl;        
        cout << "k = " << k << " quat: " << quat[k+1] << endl;
        getchar();*/
    }
}

Quat TrajPlan::quatDerNorm(Quat w, double dN, Quat q)
{
    return 0.5*w*q + dN*q;
}

Quat TrajPlan::quatSecDerNorm(Quat w, Quat dw, double dN, double ddN, Quat q)
{
    return 0.5*dw*q + dN*w*q - 0.25*w.getV().norm()*q + ddN*q;
}

std::vector<Quat> TrajPlan::quatPolynomCoef(Quat qi, Vector3d wi, Vector3d dwi, Quat qf, Vector3d wf, Vector3d dwf, double ti, double tf)
{
    // Initialization
    Quat qwi = Quat(0, wi);
    Quat dqwi = Quat(0, dwi);
    Quat qwf = Quat(0, wf);
    Quat dqwf = Quat(0, dwf);

    // To find the shortest path change the sign on one of the quaternions
    // if the dot product of its vectors is less than zero
    /*cout << "Qi: " << qi << endl;
    cout << "Qf: " << qf << endl;
    cout << "Qi.Qf: " << qi.getV().dot(qf.getV()) << endl;*/
    double qidotqf = qi.getV().dot(qf.getV());
    if (abs(qidotqf) > 1e-6)
    {
        if (qidotqf < 0)
        {
            cout << "\033[1;33mFinal quaternion changed sign to find shortest path\033[0m" << endl;
            qf = -1*qf;
        }
    }
    // Assign the derivatives of the norm
    double dNf = 0, ddNf = 0;

    // Calculate the quaternion polynomial coefficients
    Quat dqf = quatDerNorm(qwf, dNf, qf);
    Quat ddqf = quatSecDerNorm(qwf, dqwf, dNf, ddNf, qf);

    std::vector<Quat> coef;
    coef.resize(6);
    double T = tf - ti;
    Quat dQi = quatDerNorm(qwi, 0, qi);
    Quat ddQi = quatSecDerNorm(qwi, dqwi, 0, 0, qi);
    coef[0] = qi;
    coef[1] = 3*qi + dQi*T;
    coef[2] = 0.5*ddQi*T*T+3*dQi*T+6*qi;
    coef[3] = qf;
    coef[4] = 3*qf-dqf*T;
    coef[5] = 0.5*ddqf*T*T-3*dqf*T+6*qf;
    return coef;
}

Quat TrajPlan::quatPol(double t, double ti, double tf, const std::vector<Quat> &coef)
{
    double tau=(t-ti)/(tf-ti);
    return pow(1-tau,3)*(coef[0]+coef[1]*tau+coef[2]*tau*tau)+tau*tau*tau*(coef[3]+coef[4]*(1-tau)+coef[5]*(1-tau)*(1-tau));
}

Quat TrajPlan::dquatPol(double t, double ti, double tf, const std::vector<Quat> &coef)
{
    double t_ti = t-ti;
    double tf_ti = tf-ti;
    double one_t_ti_by_tf_ti = 1-t_ti/tf_ti;
    return  -3*pow((one_t_ti_by_tf_ti),2)*(coef[0]+coef[1]*t_ti/tf_ti+coef[2]*pow(t_ti,2)/pow(tf_ti,2))/tf_ti
            +pow(one_t_ti_by_tf_ti,3)*(coef[1]/tf_ti+2*coef[2]*t_ti/pow(tf_ti,2))
            +3*pow(t_ti,2)*(coef[3]+coef[4]*(one_t_ti_by_tf_ti)+coef[5]*pow(one_t_ti_by_tf_ti,2))/pow(tf_ti,3)
            +pow(t_ti,3)*(-1*coef[4]/tf_ti-2*coef[5]*(one_t_ti_by_tf_ti)/tf_ti)/pow(tf_ti,3);

    //From matlab
    /*dq = -3*(1-(t-ti)/(tf-ti))^2*(p0+p1*(t-ti)/(tf-ti)+p2*(t-ti)^2/(tf-ti)^2)/(tf-ti)
    +(1-(t-ti)/(tf-ti))^3*(p1/(tf-ti)+2*p2*(t-ti)/(tf-ti)^2)
    +3*(t-ti)^2*(p3+p4*(1-(t-ti)/(tf-ti))+p5*(1-(t-ti)/(tf-ti))^2)/(tf-ti)^3+
    (t-ti)^3*(-1*p4/(tf-ti)-2*p5*(1-(t-ti)/(tf-ti))/(tf-ti))/(tf-ti)^3;*/
}

Quat TrajPlan::ddquatPol(double t, double ti, double tf, const std::vector<Quat> &coef)
{
    double t_ti = t-ti;
    double tf_ti = tf-ti;
    double one_t_ti_by_tf_ti = 1-t_ti/tf_ti;
    return  6*one_t_ti_by_tf_ti*(coef[0]+coef[1]*t_ti/tf_ti+coef[2]*pow(t_ti,2)/pow(tf_ti,2))/pow(tf_ti,2)
            -6*pow(one_t_ti_by_tf_ti,2)*(coef[1]/tf_ti+2*coef[2]*t_ti/pow(tf_ti,2))/tf_ti
            +2*pow(one_t_ti_by_tf_ti,3)*coef[2]/pow(tf_ti,2)
            +6*t_ti*(coef[3]+coef[4]*(one_t_ti_by_tf_ti)+coef[5]*pow(one_t_ti_by_tf_ti,2))/pow(tf_ti,3)
            +6*pow(t_ti,2)*(-1*coef[4]/tf_ti-2*coef[5]*(one_t_ti_by_tf_ti)/tf_ti)/pow(tf_ti,3);
            +2*pow(t_ti,3)*coef[5]/pow(tf_ti,5);
    
    //From matlab
    /*ddq=(6*(1-(t-ti)/(tf-ti)))*(p0+p1*(t-ti)/(tf-ti)+p2*(t-ti)^2/(tf-ti)^2)/(tf-ti)^2
    -6*(1-(t-ti)/(tf-ti))^2*(p1/(tf-ti)+2*p2*(t-ti)/(tf-ti)^2)/(tf-ti)
    +2*(1-(t-ti)/(tf-ti))^3*p2/(tf-ti)^2
    +(6*(t-ti))*(p3+p4*(1-(t-ti)/(tf-ti))+p5*(1-(t-ti)/(tf-ti))^2)/(tf-ti)^3
    +6*(t-ti)^2*(-1*p4/(tf-ti)-2*p5*(1-(t-ti)/(tf-ti))/(tf-ti))/(tf-ti)^3
    +2*(t-ti)^3*p5/(tf-ti)^5;    */    
}

Vector3d TrajPlan::wPol(double t, double ti, double tf, const std::vector<Quat> &coef)
{
    Quat Q = quatPol(t,ti,tf,coef);
    Quat dQ = dquatPol(t,ti,tf,coef);
    Vector3d w = (2*dQ*Q.inv()).getV();
    return w;
}

Vector3d TrajPlan::dwPol(double t, double ti, double tf, const std::vector<Quat> &coef)
{
    Quat Q = quatPol(t,ti,tf,coef);
    Quat dQ = dquatPol(t,ti,tf,coef);
    Quat ddQ = ddquatPol(t,ti,tf,coef);
    Vector3d dw = (2*ddQ*Q.inv()-2*(dQ*Q.inv()/2)).getV();
    return dw;
}