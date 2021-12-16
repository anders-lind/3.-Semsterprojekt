#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "jacobian.h"
#include "throw.h"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <iomanip>

class Kinematics
{
public:
    // Constructor
    Kinematics();

    void connect();
    void printValues();

    // ---------- Methods for calculations ----------
    Eigen::MatrixXd calc_xp_k(Eigen::MatrixXd xk, Eigen::MatrixXd xm, double angle = 0);
    Eigen::MatrixXd calc_qp_k(Eigen::MatrixXd q_k, Eigen::MatrixXd xp_k);

    Eigen::MatrixXd calc_acc(Eigen::MatrixXd qp_k, double t);

             double calc_max_acc(Eigen::MatrixXd acc);


    // ---------- Member variables ----------
    std::string _ip;

    std::vector<double> _kastpositionDeg;
    std::vector<double> _maalpositionDeg;

    Eigen::Vector3d _xk;
    Eigen::Vector3d _xm;
    Eigen::MatrixXd _xp_k;
    Eigen::MatrixXd _qp_k;
    Eigen::MatrixXd _acc;

    double _t;
};

#endif // KINEMATICS_H
