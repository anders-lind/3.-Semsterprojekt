#include "jacobian.h"

jacobian::jacobian()
{

}

jacobian::jacobian(Eigen::MatrixXd jointValues)
{
    double q1 = jointValues(0,0), q2 = jointValues(1,0), q3 = jointValues(2,0), q4 = jointValues(3,0), q5 = jointValues(4,0), q6 = jointValues(5,0);

    Eigen::MatrixXd j {
        { (273*cos(q1))/2500 + (3223*cos(q1)*cos(q5))/10000 + (17*cos(q2)*sin(q1))/40 - (947*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 + (947*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 - (3223*sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))))/10000 - (49*sin(q1)*sin(q2)*sin(q3))/125 + (49*cos(q2)*cos(q3)*sin(q1))/125, (17*cos(q1)*sin(q2))/40 + (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (3223*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000 + (49*cos(q1)*cos(q2)*sin(q3))/125 + (49*cos(q1)*cos(q3)*sin(q2))/125, (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (3223*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000 + (49*cos(q1)*cos(q2)*sin(q3))/125 + (49*cos(q1)*cos(q3)*sin(q2))/125, (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (3223*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000, - (3223*sin(q1)*sin(q5))/10000 - (3223*cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))))/10000,                                                                                                                                                     0},
        { (273*sin(q1))/2500 - (17*cos(q1)*cos(q2))/40 + (3223*cos(q5)*sin(q1))/10000 + (947*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (3223*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))))/10000 - (49*cos(q1)*cos(q2)*cos(q3))/125 + (49*cos(q1)*sin(q2)*sin(q3))/125, (17*sin(q1)*sin(q2))/40 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 + (3223*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 + (49*cos(q2)*sin(q1)*sin(q3))/125 + (49*cos(q3)*sin(q1)*sin(q2))/125, (3223*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 + (49*cos(q2)*sin(q1)*sin(q3))/125 + (49*cos(q3)*sin(q1)*sin(q2))/125, (3223*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000,   (3223*cos(q1)*sin(q5))/10000 + (3223*cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))))/10000,                                                                                                                                                     0},
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                         (49*sin(q2)*sin(q3))/125 - (49*cos(q2)*cos(q3))/125 - (17*cos(q2))/40 - (3223*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                                 (49*sin(q2)*sin(q3))/125 - (49*cos(q2)*cos(q3))/125 - (3223*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                 (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 - (3223*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                 -(3223*cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))))/10000,                                                                                                                                                     0},
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                              sin(q1),                                                                                                                                                                                                                                                                                                                                                                    sin(q1),                                                                                                                                                                                                                                                                                              sin(q1),                                                         cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)), cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))},
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                             -cos(q1),                                                                                                                                                                                                                                                                                                                                                                   -cos(q1),                                                                                                                                                                                                                                                                                             -cos(q1),                                                         cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)), sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5)},
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                        1,                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                                    0,                                                                                         sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)),                                                  -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))},
    };

    J = j;
}

Eigen::MatrixXd jacobian::vec2Eig(std::vector<double> coordinates)
{
    Eigen::MatrixXd mat(6,1);
    for (int i = 0; i < 6; i++)
        mat(i,0) = coordinates[i];

    return mat;
}

std::vector<double> jacobian::eig2Vec(Eigen::MatrixXd mat)
{
    std::vector<double> vec(6);
    for (int i = 0; i < 6; i++)
        vec[i] = mat(i,0);

    return vec;
}

void jacobian::update(Eigen::MatrixXd jointValues)
{
    double q1 = jointValues(0,0), q2 = jointValues(1,0), q3 = jointValues(2,0), q4 = jointValues(3,0), q5 = jointValues(4,0), q6 = jointValues(5,0);

    Eigen::MatrixXd j{
        { (273*cos(q1))/2500 + (3223*cos(q1)*cos(q5))/10000 + (17*cos(q2)*sin(q1))/40 - (947*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 + (947*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 - (3223*sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))))/10000 - (49*sin(q1)*sin(q2)*sin(q3))/125 + (49*cos(q2)*cos(q3)*sin(q1))/125, (17*cos(q1)*sin(q2))/40 + (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (3223*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000 + (49*cos(q1)*cos(q2)*sin(q3))/125 + (49*cos(q1)*cos(q3)*sin(q2))/125, (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (3223*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000 + (49*cos(q1)*cos(q2)*sin(q3))/125 + (49*cos(q1)*cos(q3)*sin(q2))/125, (947*cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (947*sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (3223*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))))/10000, - (3223*sin(q1)*sin(q5))/10000 - (3223*cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))))/10000,                                                                                                                                                     0},
        { (273*sin(q1))/2500 - (17*cos(q1)*cos(q2))/40 + (3223*cos(q5)*sin(q1))/10000 + (947*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (3223*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))))/10000 - (49*cos(q1)*cos(q2)*cos(q3))/125 + (49*cos(q1)*sin(q2)*sin(q3))/125, (17*sin(q1)*sin(q2))/40 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 + (3223*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 + (49*cos(q2)*sin(q1)*sin(q3))/125 + (49*cos(q3)*sin(q1)*sin(q2))/125, (3223*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 + (49*cos(q2)*sin(q1)*sin(q3))/125 + (49*cos(q3)*sin(q1)*sin(q2))/125, (3223*sin(q5)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))))/10000 - (947*sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 - (947*cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000,   (3223*cos(q1)*sin(q5))/10000 + (3223*cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))))/10000,                                                                                                                                                     0},
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                         (49*sin(q2)*sin(q3))/125 - (49*cos(q2)*cos(q3))/125 - (17*cos(q2))/40 - (3223*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                                 (49*sin(q2)*sin(q3))/125 - (49*cos(q2)*cos(q3))/125 - (3223*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                 (947*cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 - (3223*sin(q5)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))))/10000 + (947*sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000,                                                                 -(3223*cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))))/10000,                                                                                                                                                     0},
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                              sin(q1),                                                                                                                                                                                                                                                                                                                                                                    sin(q1),                                                                                                                                                                                                                                                                                              sin(q1),                                                         cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)), cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))},
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                                                                                                                                                                                                                             -cos(q1),                                                                                                                                                                                                                                                                                                                                                                   -cos(q1),                                                                                                                                                                                                                                                                                             -cos(q1),                                                         cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)), sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5)},
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                        1,                                                                                                                                                                                                                                                                                                                                                                                                    0,                                                                                                                                                                                                                                                                                                                                                                          0,                                                                                                                                                                                                                                                                                                    0,                                                                                         sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)),                                                  -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))},
    };

    J = j;
}

Eigen::MatrixXd jacobian::forward(Eigen::MatrixXd jointValues)
{
    double q1 = jointValues(0,0), q2 = jointValues(1,0), q3 = jointValues(2,0), q4 = jointValues(3,0), q5 = jointValues(4,0), q6 = jointValues(5,0);

    // Forward kinematics for each link
    Eigen::MatrixXd A1 {
        {cos(q1),   0,          sin(q1),    0},
        {sin(q1),   0,          -cos(q1),   0},
        {0,         1,          0,          0.08916},
        {0,         0,          0,          1}
    };
    Eigen::MatrixXd A2 {
        {cos(q2),   -sin(q2),   0,          -0.425*cos(q2)},
        {sin(q2),   cos(q2),    0,          -0.425*sin(q2)},
        {0,         0,          1,          0},
        {0,         0,          0,          1}};
    Eigen::MatrixXd A3 {
        {cos(q3),    -sin(q3),  0,          -0.392*cos(q3)},
        {sin(q3),    cos(q3),   0,          -0.392*sin(q3)},
        {0,         0,          1,          0},
        {0,         0,          0,          1}
    };
    Eigen::MatrixXd A4 {
        {cos(q4),   0,          sin(q4),    0},
        {sin(q4),   0,          -cos(q4),   0},
        {0,         1,          0,          0.1092},
        {0,         0,          0,          1}};
    Eigen::MatrixXd A5 {
        {cos(q5),   0,          -sin(q5),   0},
        {sin(q5),   0,          cos(q5),    0},
        {0,         -1,         0,          0.0947},
        {0,         0,          0,          1}};
    Eigen::MatrixXd A6 {
        {cos(q6),   -sin(q6),   0,          0},
        {sin(q6),   cos(q6),    0,          0},
        {0,         0,          1,          0.0823},
        {0,         0,          0,          1}};


    // T = A1*A2*A3*A4*A5*A6;
    Eigen::MatrixXd T {
        {  cos(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))), - sin(q6)*(sin(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))), cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))), (273*sin(q1))/2500 - (17*cos(q1)*cos(q2))/40 + (823*cos(q5)*sin(q1))/10000 + (947*cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/10000 + (947*sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/10000 - (823*sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))))/10000 - (49*cos(q1)*cos(q2)*cos(q3))/125 + (49*cos(q1)*sin(q2)*sin(q3))/125},
        {- cos(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - sin(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))),   sin(q6)*(cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))) - cos(q6)*(cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))), sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5), (947*cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/10000 - (823*cos(q1)*cos(q5))/10000 - (17*cos(q2)*sin(q1))/40 - (273*cos(q1))/2500 - (947*sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/10000 + (823*sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))))/10000 + (49*sin(q1)*sin(q2)*sin(q3))/125 - (49*cos(q2)*cos(q3)*sin(q1))/125},
        {                                                                                      sin(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + cos(q5)*cos(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                       cos(q6)*(cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) - sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) - cos(q5)*sin(q6)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                  -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))),                                                                                                                               (947*sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/10000 - (49*cos(q2)*sin(q3))/125 - (49*cos(q3)*sin(q2))/125 - (823*sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3))))/10000 - (947*cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)))/10000 - (17*sin(q2))/40 + 2229.0/25000.0},
        {                                                                                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                                                                                                                                                      1}
    };

    return T;
}