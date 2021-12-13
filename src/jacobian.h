#ifndef JACOBIAN_H
#define JACOBIAN_H

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>


class jacobian
{
public:
    jacobian();
    jacobian(Eigen::MatrixXd jointValues);

    Eigen::MatrixXd get(){return J;};
    Eigen::MatrixXd getInverse(){return J.inverse();};

    static Eigen::MatrixXd vec2Eig(std::vector<double> coordinates);
    static std::vector<double> eig2Vec(Eigen::MatrixXd mat);

    Eigen::MatrixXd forward(Eigen::MatrixXd jointValues);
    void update(Eigen::MatrixXd jointValues);

private:
    Eigen::MatrixXd J{6,6};
};

#endif // JACOBIAN_H
