#include "gripper.h"
#include<unistd.h>
#include<iostream>

Gripper::Gripper()
{
    m_gripper = new rl::hal::WeissWsg50("192.168.100.11", 1000, 3.00f, 40.0f, 10);
    m_gripper->open();
    m_gripper->start();
    if (!m_gripper->isConnected()) {
        std::cout << "Failed to connect to gripper!" << std::endl;
    }
    std::cout << "The connection is: " << m_gripper->isConnected() << std::endl;
}

Gripper::Gripper(std::string ip)
{
    m_gripper = new rl::hal::WeissWsg50(ip, 1000, 3.00f, 40.0f, 10);
    m_gripper->open();
    m_gripper->start();
    if (!m_gripper->isConnected()) {
        std::cout << "Failed to connect to gripper!" << std::endl;
    }
    std::cout << "The connection is: " << m_gripper->isConnected() << std::endl;
}

void Gripper::graspObject()
{
    m_gripper->doPrePositionFingers(0.04f, 0.4, false, false);
    sleep(2);
    std::cout << "Gripper has grasped the pingpong ball!" << std::endl;
}

void Gripper::releaseObject()
{
    m_gripper->doPrePositionFingers(0.11f, 0.4, false, false);
    sleep(2);
    std::cout << "Gripper has released the object" << std::endl;
}

void Gripper::graspObjectDia(float Diameter)
{
    m_gripper->doPrePositionFingers(0.04f, 0.4, false, false);
    sleep(2);
    std::cout << "Gripper has grasped the object with diameter: " << Diameter << std::endl;
}

void Gripper::closeConnection()
{
    m_gripper->stop();
    m_gripper->close();
    std::cout << "Connection closed." << std::endl;
}
