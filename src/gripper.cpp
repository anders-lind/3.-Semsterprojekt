#include "gripper.h"

gripper::gripper()
{
    m_gripper = new rl::hal::WeissWsg50("192.168.100.11", 1000, 3.00f, 40.0f, 10);
    std::cout << "Opening connection" << std::endl;
    m_gripper->open();
    std::cout << "Starting connection" << std::endl;
    m_gripper->start();
    if (!m_gripper->isConnected()) {
        std::cout << "Failed to connect to gripper!" << std::endl;
    }
    std::cout << "The connection is: " << m_gripper->isConnected() << std::endl;
}

gripper::gripper(std::string ip)
{
    m_gripper = new rl::hal::WeissWsg50(ip, 1000, 3.00f, 40.0f, 10);
    std::cout << "Opening connecttion" << std::endl;
    m_gripper->open();
    std::cout << "Starting connection" << std::endl;
    m_gripper->start();
    if (!m_gripper->isConnected()) {
        std::cout << "Failed to connect to gripper!" << std::endl;
    }
    std::cout << "The connection is: " << m_gripper->isConnected() << std::endl;
}

void gripper::graspObject()
{
    m_gripper->doPrePositionFingers(0.04f, 0.4, false, false);
    sleep(2);
    std::cout << "Gripper has grasped the pingpong ball!" << std::endl;
}

void gripper::releaseObject(double time)
{
    std::this_thread::sleep_for(std::chrono::duration<double>(time - 0.06));
    m_gripper->doPrePositionFingers(0.06f, 0.4, false, false);
    std::cout << "Gripper has released the object" << std::endl;
    m_gripper->doPrePositionFingers(0.09f, 0.4, false, false);
}

void gripper::graspObjectDia(float Diameter)
{
    m_gripper->doPrePositionFingers(0.04f, 0.4, false, false);
    sleep(2);
    std::cout << "Gripper has grasped the object with diameter: " << Diameter << std::endl;
}

void gripper::closeConnection()
{
    m_gripper->stop();
    m_gripper->close();
    std::cout << "Connection closed." << std::endl;
}
