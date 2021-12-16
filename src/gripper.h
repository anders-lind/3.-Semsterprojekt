#ifndef GRIPPER_H
#define GRIPPER_H

#include <rl/hal/WeissWsg50.h>
#include<unistd.h>
#include<iostream>
#include<chrono>
#include<thread>

class gripper
{
public:
    gripper();
    gripper(std::string ip);

    void graspObject();
    void releaseObject(double time);
    void graspObjectDia(float Diameter);
    void closeConnection();

private:
    rl::hal::WeissWsg50* m_gripper;
};

#endif // GRIPPER_H
