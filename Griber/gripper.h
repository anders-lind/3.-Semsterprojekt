#ifndef GRIPPER_H
#define GRIPPER_H
#include <rl/hal/WeissWsg50.h>

class Gripper
{
public:
    Gripper();
    Gripper(std::string ip);
    void graspObject();
    void releaseObject();
    void graspObjectDia(float Diameter);
    void closeConnection();
private:
    rl::hal::WeissWsg50* m_gripper;
};

#endif // GRIPPER_H
