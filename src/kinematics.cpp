#include "kinematics.h"

using namespace std;
using namespace ur_rtde;


Kinematics::Kinematics()
{

}


Eigen::MatrixXd Kinematics::calc_xp_k(Eigen::MatrixXd xk, Eigen::MatrixXd xm, double angle)
{
    Throw kast;

    Eigen::MatrixXd tmp = kast.calculate3DSpeed(xk, xm, angle);
    Eigen::MatrixXd xp_k(6, 1);
    xp_k(0) = tmp(0);
    xp_k(1) = tmp(1);
    xp_k(2) = tmp(2);
    xp_k(3) = 0;
    xp_k(4) = 0;
    xp_k(5) = 0;
    _xp_k = xp_k;
    return xp_k;
}


Eigen::MatrixXd Kinematics::calc_qp_k(Eigen::MatrixXd q_k, Eigen::MatrixXd xp_k)
{
    // Calculate joint-speed from Jacobian and throw-speed
    jacobian Jac(q_k);
    Eigen::MatrixXd J = Jac.get();
    Eigen::MatrixXd qp_k = J.inverse() * xp_k;
    cout << "qp_k = Jac(q_k).inverse() * xp_k" << endl;
    cout << "q_k = \n" << q_k << endl;
    cout << "qp_k = " << qp_k << "\n = \n" << J.inverse() << "\n * \n" << xp_k << endl;
    //qp_k(0) = 0;

    // Opstil grafer for q og qp for hvert led
    // qp(t) = f(t) = at + b                ->      b = 0       a = (qp(t) - b) / t
    //  q(t) = F(t) = a/2*t^2 + bt + c      ->      c = q_s     a = 2(q(t) - bt - c) / t^2

    _qp_k = qp_k;
    return qp_k;
}


Eigen::MatrixXd Kinematics::calc_acc(Eigen::MatrixXd qp_k, double t)
{
    // Calculate acceleration matrix
    Eigen::MatrixXd a(6, 1);
    for (int i = 0; i < 6; i++)
    {
        a(i) = (qp_k(i) - 0) / t;
    }

    _acc = a;
    return a;
}


double Kinematics::calc_max_acc(Eigen::MatrixXd acc)
{
    double acceleration = 0;
    for (int i = 0; i < 6; i++)
    {
        if (acc(i) > acceleration)
        {
            acceleration = acc(i);
        }
        else if (acc(i) < acceleration)
        {
            double temp = acc(i) * -1;
            if (temp > acceleration)
                acceleration = temp;
        }
    }

    return acceleration;
}
