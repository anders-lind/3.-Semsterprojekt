#include "robot.h"
#include "kinematics.h"
#include "jacobian.h"
#include<chrono>
#include<thread>

robot::robot()
{
    m_gripper = new gripper("192.168.100.11");
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
    std::vector<double> coordinates = {130, -110, 125, -130, -62, -72};
    radConversion(coordinates);
    //Sets speed, acceleration, move syncronous or not
    double speed = 3;
    double acc = 1;
    bool async = false;
    //Moves the robot
    m_control->moveJ(coordinates, speed, acc, async);
}

void robot::pickUpBall(std::vector<double> coordinates)
{
    //Sets speed, acceleration, move syncronous or not
    double speed = 3;
    double acc = 1;
    bool async = false;
    //Moves the robot
    //m_control->moveL(coordinates, speed, acc, async);
    m_control->moveL(coordinates);
    m_gripper->graspObject();
}

void robot::goToThrowPos(){
    std::vector<double> qThrowPos = {99, -90, 106, -124, - 85, -101};
    radConversion(qThrowPos);
    m_control->moveJ(qThrowPos);
}

void robot::throwBall(std::vector<double> goalCoordinates, double t)
{
    // Joint values for throw position
    std::vector<double> q_kV = {99, -90, 106, -124, -85, -101};
    radConversion(q_kV);

    // Cartisian coordinates for throw position
    std::vector<double> x_kV = {0.4674, -0.1943, -0.043, 0, 0, 0};
    Eigen::MatrixXd x_mE = jacobian::vec2Eig(goalCoordinates);


    Kinematics kin;

    // COnvert from std::vector to Eigen Matrix
    Eigen::MatrixXd x_kE = jacobian::vec2Eig(x_kV);

    // Calculate cartisian throw speed
    Eigen::MatrixXd xp_k = kin.calc_xp_k(x_kE, x_mE);

    // Calculate joint throw speed
    Eigen::MatrixXd qp_k = kin.calc_qp_k(jacobian::vec2Eig(q_kV), xp_k);

    // Calculate joint acceleration of throw
    Eigen::MatrixXd acc = kin.calc_acc(qp_k, t);

    double max_acc = kin.calc_max_acc(acc);

    // Calculate start position
    std::vector<double> qp_k_Neg(6);
    for(int i = 0; i < jacobian::eig2Vec(qp_k).size(); ++i){
        qp_k_Neg.at(i) = jacobian::eig2Vec(qp_k).at(i) * (-1);
    }

    // Go to x_s
    m_control->speedJ(qp_k_Neg, max_acc, t);
    std::this_thread::sleep_for(std::chrono::duration<double>(t)); // Skal msåke kun bruges når man bruger den rigtige robot
    m_control->speedStop();
    std::this_thread::sleep_for(std::chrono::duration<double>(2));

    // Make throw
    m_control->speedJ(jacobian::eig2Vec(qp_k), max_acc, t);
    std::thread t1(&gripper::releaseObject, *m_gripper);
    std::this_thread::sleep_for(std::chrono::duration<double>(t)); // Skal msåke kun bruges når man bruger den rigtige robot
    t1.join();
    m_control->speedStop();

}

void robot::closeConnections(){
    m_gripper->closeConnection();
    m_control->disconnect();
    m_recieve->disconnect();
}
/*
void robot::throwBall(std::vector<double> goalCoordinates)
{
    std::vector<double> q_k = {99, -90, 106, -124, -85, -101};
    radConversion(q_k);
    std::vector<double> x_k = {0.4674, -0.1943, -0.043, 0, 0, 0};
    Eigen::MatrixXd x_m = jacobian::vec2Eig(goalCoordinates);
    Kinematics kin;
    Eigen::MatrixXd xp_k = kin.calc_xp_k(jacobian::vec2Eig(x_k), x_m);
    Eigen::MatrixXd qp_k = kin.calc_qp_k(jacobian::vec2Eig(q_k), xp_k);
    Eigen::MatrixXd acc = kin.calc_acc(qp_k, 0.5);
    double max_acc = kin.calc_max_acc(acc);

    std::vector<double> test(6);
    for(int i = 0; i < jacobian::eig2Vec(qp_k).size(); ++i){
        test.at(i) = jacobian::eig2Vec(qp_k).at(i) * (-1);
    }

    m_control->speedJ(test, max_acc, 0.5);
    sleep(double(0.5));
    m_control->speedStop();

    m_control->speedJ(jacobian::eig2Vec(qp_k), max_acc, 0.5);
    sleep(double(0.5));
    m_control->speedStop();
}
*/
void robot::radConversion(std::vector<double> &jPose)
{
    //Degrees to rad conversion
    double rad = M_PI/180;
    for(int i = 0; i < jPose.size(); ++i){
        jPose[i] = jPose.at(i) * rad;
    }
}
