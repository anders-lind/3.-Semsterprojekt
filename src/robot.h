#ifndef ROBOT_H
#define ROBOT_H
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "jacobian.h"
#include "gripper.h"

class robot
{
public:
    robot();
    robot(std::string robotIP, std::string gripperIP);
    void startingPosition();
    void pickUpBall(std::vector<double> coordinates);
    void goToThrowPos();
    void throwBall(std::vector<double> goalCoordinates, double t = 0.5);
    void radConversion(std::vector<double> &jPose);

private:
    //gripper m_gripper;
    ur_rtde::RTDEControlInterface *m_control;
    ur_rtde::RTDEReceiveInterface *m_recieve;
};

#endif // ROBOT_H
