#include "robot.h"
#include "kinematics.h"
#include "jacobian.h"
#include<chrono>
#include<thread>

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
    std::vector<double> coordinates = {130, -110, 125, -130, -62, -72};
    //std::vector<double> coordinates = {130, -110, 125, -120, -89, -159};
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

void robot::throwBall(std::vector<double> goalCoordinates, double angle, double time)
{
    // // Used to control timing
    // double stepTime = m_control->getStepTime();
    // chrono::time_point<chrono::system_clock> start, end;
    // chrono::duration<double> currentDuration;
    // start = chrono::system_clock::now();


    // do{

    //     sleep(stepTime);
    //     currentDuration = chrono::system_clock::now() - start;
    // }
    // while (currentDuration.count() < time)


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
    Eigen::MatrixXd xp_k = kin.calc_xp_k(x_k, x_m, angle);
    cout << "xp_k = \n" << xp_k << endl << endl;

    // Calculate joint throw speed
    Eigen::MatrixXd qp_k = kin.calc_qp_k(q_k, xp_k);

    // Calculate joint acceleration of throw
    Eigen::MatrixXd acc = kin.calc_acc(qp_k, time);
    double max_acc = kin.calc_max_acc(acc);
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
    std::thread t1(&gripper::releaseObject, *m_gripper, time);
    m_control->speedJ(jacobian::eig2Vec(qp_k), max_acc, time);
    std::this_thread::sleep_for(std::chrono::duration<double>(time));
    m_control->speedStop();
    t1.join();
}

/*
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
*/

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

void robot::closeConnections(){
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
