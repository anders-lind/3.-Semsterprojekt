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
#include "kinematics.h"

class robot
{
public:
    //Basic constructor skal ikke bruges
    robot();
    //Constructor til sim
    robot(std::string robotIP);
    //Constructor til program
    robot(std::string robotIP, std::string gripperIP);

    //Robotcommandoer
    void moveJ(std::vector<double> jointValues);
    void moveL(std::vector<double> coordinates);
    void speedL(std::vector<double>dx, double a, double t);
    std::vector<double> getActualTCPPose();
    //Program kommandoer
    void startingPosition();
    void pickUpBall(std::vector<double> coordinates, bool isSimulation = false);
    void goToThrowPos(std::vector<double> qThrowPosDegrees = {99, -90, 106, -124, - 85, -101});
    void throwBall(double &max_acc, double &speed, std::vector<double> x_m_goalCoordinates, double angle = 0, double time = 0.25, double qpp_max = 0, bool isSimulation = false);
    void closeConnections(bool isSimulation = false);
    void radConversion(std::vector<double> &jPose);

private:
    gripper *m_gripper;
    ur_rtde::RTDEControlInterface *m_control;
    ur_rtde::RTDEReceiveInterface *m_recieve;
};

#endif // ROBOT_H
