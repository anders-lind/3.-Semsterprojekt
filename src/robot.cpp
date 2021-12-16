#include "robot.h"

using namespace std;


robot::robot()
{
    //m_gripper = new gripper("192.168.100.11");
    try{
        std::cout << std::boolalpha;
        std::cout << "attempting to connect to robot socket at 127.0.0.1" << std::endl;
        m_control = new ur_rtde::RTDEControlInterface("127.0.0.1");
        m_recieve = new ur_rtde::RTDEReceiveInterface("127.0.0.1");
        std::cout << "Succesfull connection to robot." << std::endl;
    } catch(const std::runtime_error& error){
        std::cout << "UR_RTDE: Failed to connect to robot!!!" << std::endl;
    }
}

robot::robot(std::string robotIP)
{
    try{

        std::cout << std::boolalpha;

        std::cout << "attempting to connect to robot socket at " << robotIP << std::endl;

        m_control = new ur_rtde::RTDEControlInterface(robotIP);
        m_recieve = new ur_rtde::RTDEReceiveInterface(robotIP);
        std::cout << "Succesfull connection to robot." << std::endl;
    } catch(const std::runtime_error& error){
        std::cout << "UR_RTDE: Failed to connect to robot!!!" << std::endl;
    }
}

robot::robot(std::string robotIP, std::string gripperIP)
{
    m_gripper = new gripper(gripperIP);
    try{
        std::cout << std::boolalpha;
        std::cout << "attempting to connect to robot socket at " << robotIP << std::endl;
        m_control = new ur_rtde::RTDEControlInterface(robotIP);
        m_recieve = new ur_rtde::RTDEReceiveInterface(robotIP);
        std::cout << "Succesfull connection to robot." << std::endl;
    } catch(const std::runtime_error& error){
        std::cout << "UR_RTDE: Failed to connect to robot!!!" << std::endl;
    }
}

void robot::startingPosition()
{
    //Sets the coordinates to our zero position and converts to rad
    //Første vi havde
    //std::vector<double> coordinates = {130, -110, 125, -130, -62, -72};
    //Tests
    std::vector<double> coordinates = {98, -99, 119, -158, -85, -74};
    //std::vector<double> coordinates = {66, -97, 133, -158, -84, -82};
    radConversion(coordinates);
    //Sets speed, acceleration, move syncronous or not
    double speed = 3;
    double acc = 1;
    bool async = false;
    //Moves the robot
    m_control->moveJ(coordinates, speed, acc, async);
}

void robot::pickUpBall(std::vector<double> coordinates, bool isSimulation)
{
    //Sets speed, acceleration, move syncronous or not
    double speed = 3;
    double acc = 1;
    bool async = false;
    //Moves the robot
    //m_control->moveL(coordinates, speed, acc, async);
    std::vector<double> tmp(6);
    tmp = coordinates;
    tmp.at(2) = 0.200;
    m_control->moveL(tmp);
    m_control->moveL(coordinates);
    if (!isSimulation)
        m_gripper->graspObject();
    m_control->moveL(tmp);
}

void robot::goToThrowPos(vector<double> qThrowPosDegrees){
    radConversion(qThrowPosDegrees);
    m_control->moveJ(qThrowPosDegrees);
}

void robot::throwBall(double &max_acc, double &speed, std::vector<double> goalCoordinates, double angle, double time, double qpp_max, bool isSimulation)
{
    // Joint values for throw position
    std::vector<double> q_kVec = m_recieve->getTargetQ();      // Default value = {99, -90, 106, -124, -85, -101}

    // Cartisian coordinates for throw position
    std::vector<double> x_kVec = m_recieve->getActualTCPPose();

    // Convert from std::vector to Eigen Matrix
    Eigen::MatrixXd x_m = jacobian::vec2Eig(goalCoordinates);
    Eigen::MatrixXd x_k = jacobian::vec2Eig(x_kVec);
    Eigen::MatrixXd q_k = jacobian::vec2Eig(q_kVec);

    cout << "x_m = \n" << x_m << endl;
    cout << "x_k = \n" << x_k << endl;
    cout << "q_k = \n" << q_k << endl;

    Kinematics kin;


    // Calculate cartisian throw speed
    Eigen::MatrixXd xp_k(6,1);
    xp_k = kin.calc_xp_k(x_k, x_m, angle);
    cout << "xp_k = \n" << xp_k << endl << endl;
    speed = xp_k.norm();

    // Calculate joint throw speed
    Eigen::MatrixXd qp_k = kin.calc_qp_k(q_k, xp_k);


    // Calculate time of throw if no time is given
    double qp_max = -0;
    if (time <= 0){
        // first we find the highest joint speed
        for (int i = 0; i < 6; i++){
            if (abs(qp_k(i)) > abs(qp_max))
                qp_max = qp_k(i);
        }
        cout << "hastighed qp_k = " << qp_k << " max qp_k = " << qp_max << endl;

        // Find time from highest velocity and acc
        time = abs(qp_max) / qpp_max;
        cout << "tiden er udregnet til t = " << time << " sekunder" << endl;
    }

    // Calculate joint acceleration of throw
    Eigen::MatrixXd acc = kin.calc_acc(qp_k, time);
    max_acc = kin.calc_max_acc(acc);
    cout << "acc = \n" << acc << " max acc = " << max_acc << endl << endl;


    // Calculate start position
    Eigen::MatrixXd q_s(6,1);
    for (int i = 0; i < 6; i++){
        q_s(i) = q_k(i) - 0.5*acc(i)*time*time;
    }


    // Go to x_s
    cout << "Go to throw start position" << endl;
    m_control->moveJ(jacobian::eig2Vec(q_s),1,3,false);
    std::this_thread::sleep_for(std::chrono::duration<double>(1));


    // Make throw
    cout << "Throw!" << endl;
    if (!isSimulation){
        std::thread t1(&gripper::releaseObject, *m_gripper, time);
        m_control->speedJ(jacobian::eig2Vec(qp_k), max_acc, time);
        std::this_thread::sleep_for(std::chrono::duration<double>(time));
        m_control->speedStop(30);
        t1.join();
    }
    if (isSimulation){
        m_control->speedJ(jacobian::eig2Vec(qp_k), max_acc, time);
        std::this_thread::sleep_for(std::chrono::duration<double>(time/2));
        m_control->speedStop();
    }

}

void robot::moveJ(std::vector<double> jointValues)
{
    m_control->moveJ(jointValues, 1, 3, false);
}


void robot::moveL(std::vector<double> coordinates)
{
    m_control->moveL(coordinates, 1, 3, false);
}


void robot::speedL(vector<double> dx, double a, double t)
{
    m_control->speedL(dx, a, t);
    sleep(t);
}

vector<double> robot::getActualTCPPose()
{
    return m_recieve->getActualTCPPose();
}

void robot::closeConnections(bool isSimulation){
    if (!isSimulation)
        m_gripper->closeConnection();
    m_control->disconnect();
    m_recieve->disconnect();
}

void robot::radConversion(std::vector<double> &jPose)
{
    //Degrees to rad conversion
    double rad = M_PI/180;
    for(int i = 0; i < jPose.size(); ++i){
        jPose[i] = jPose.at(i) * rad;
    }
}
