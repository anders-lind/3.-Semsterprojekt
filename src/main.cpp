//opencv includes
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/videoio.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/calib3d.hpp>
#include<thread>
#include <chrono>

//My program includes
#include "colourdetection.h"
#include "objectdetection.h"
#include "camera.h"
#include "pylon.h"
#include "gripper.h"
#include "robot.h"
#include "database.h"

int main(int argc, char* argv[])
{

    //Opretter forbindelse til databasen:
    Database db;
    //db.drop_tables();
    // db.connect();
    db.create_tables();

    //Machine vision
    //Loads image
    cv::Mat image;
    pylon cP;
    cP.getimage(image);
    image = cv::imread("billede.png");
    cv::imshow("billede", image);
    cv::waitKey(0);

    //Creation of colour masks from corrected image:
    cv::Mat blue, yellow, green, orange;
    colourDetection a;

    //Calibrate colours
    //a.CalibrateColours(image);

    //Detect colours
    a.DetectBlue(image, blue);
    a.DetectYellow(image, yellow);
    a.DetectGreen(image, green);
    a.DetectOrange(image, orange);

    //Get coordinates
    objectDetection o;
    //std::vector<cv::Point2f> real er realworld coordinaterne der skal laves om til robot coordianter
    std::vector<cv::Point2f> ballCoor;
    std::vector<cv::Point2f> cupCoor;
    cv::Point2f A, B;
    //o.getColouredCupCoordinates(yellow, cupCoor);
    //o.getColouredBallCoordinates(yellow, ballCoor);
    o.getSingleBallCoordinates(image, ballCoor);
    o.getSingleCupCoordinates(image, cupCoor);

    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    std::cout << ballCoor << std::endl;

    std::vector<double> robotBallCoor(6);
    std::vector<double> robotCupCoor(6);

    robotBallCoor = o.convertCoordinates(B, 0);
    robotCupCoor = o.convertCoordinates(A, 0.075);

    //Robot
    //Is it a simulation or not:
    bool isSimulation = false;

    //Opretter forbindelse til robotten og griberen:
    robot r("192.168.100.49", "192.168.100.10");
    //Til Sim
    //robot r("127.0.0.1");

    //Program robotten kører
    std::cout << "Start position" << std::endl;
    r.startingPosition();
    sleep(1);

    std::cout << "Pick up ball" << std::endl;
    //std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    std::cout << "Throw position" << std::endl;
    //ThrowPosition sættes
    //Standard position
    std::vector<double> throwJointValuesDegrees = {99, -90, 106, -124, -85, -101};
    r.goToThrowPos(throwJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    std::cout << "Throw!" << std::endl;
    double angle = 0;       /*45 * (M_PI/180)*/
    double time = 0.13;
    double max_acc = 0;
    double speed = 0;
    //std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(max_acc, speed, robotCupCoor, angle, time, isSimulation);
    sleep(1);

    r.startingPosition();

    //Ligger data ind i databasen efter kast
    bool hit;
    std::cout << "Is the ball in the cup? " << std::endl;
    std::cin >> hit;
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    std::vector<double> throwPos = throwJointValuesDegrees;
    r.radConversion(throwPos);
    db.add_joint_throw_values(throwPos);
    db.add_kast_data(robotCupCoor, throwPos, time, hit, angle, max_acc, speed);

/*
    //Finder ny farve og kaster
    o.getColouredCupCoordinates(orange, cupCoor);
    o.getColouredBallCoordinates(orange, ballCoor);
    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    std::cout << ballCoor << std::endl;
    robotBallCoor = o.convertCoordinates(B, 0);
    robotCupCoor = o.convertCoordinates(A, 0.075);

    //Program robotten kører
    std::cout << "Start position" << std::endl;
    r.startingPosition();
    sleep(1);

    std::cout << "Pick up ball" << std::endl;
    //std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    std::cout << "Throw position" << std::endl;
    r.goToThrowPos(throwJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    std::cout << "Throw!" << std::endl;
    angle = 0;
    time = 0.13;
    max_acc = 0;
    speed = 0;
    //std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(max_acc, speed, robotCupCoor, angle, time, isSimulation);
    sleep(1);

    r.startingPosition();

    //Ligger data ind i databasen efter kast
    std::cout << "Is the ball in the cup? " << std::endl;
    std::cin >> hit;
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    throwPos = throwJointValuesDegrees;
    r.radConversion(throwPos);
    db.add_joint_throw_values(throwPos);
    db.add_kast_data(robotCupCoor, throwPos, time, hit, angle, max_acc, speed);

    //Finder ny farve og kaster
    //Finder ny farve og kaster
    o.getColouredCupCoordinates(blue, cupCoor);
    o.getColouredBallCoordinates(blue, ballCoor);
    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    std::cout << ballCoor << std::endl;
    robotBallCoor = o.convertCoordinates(B, 0);
    robotCupCoor = o.convertCoordinates(A, 0.075);

    //Program robotten kører
    std::cout << "Start position" << std::endl;
    r.startingPosition();
    sleep(1);

    std::cout << "Pick up ball" << std::endl;
    //std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    std::cout << "Throw position" << std::endl;
    r.goToThrowPos(throwJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    std::cout << "Throw!" << std::endl;
    angle = 0;
    time = 0.13;
    max_acc = 0;
    speed = 0;
    //std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(max_acc, speed, robotCupCoor, angle, time, isSimulation);
    sleep(1);

    r.startingPosition();

    //Ligger data ind i databasen efter kast
    std::cout << "Is the ball in the cup? " << std::endl;
    std::cin >> hit;
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    throwPos = throwJointValuesDegrees;
    r.radConversion(throwPos);
    db.add_joint_throw_values(throwPos);
    db.add_kast_data(robotCupCoor, throwPos, time, hit, angle, max_acc, speed);

    //Finder ny farve og kaster
    cP.getimage(image);
    image = cv::imread("billede.png");
    cv::imshow("billede", image);
    cv::waitKey(0);
    o.getColouredCupCoordinates(green, cupCoor);
    o.getSingleBallCoordinates(image, ballCoor);
    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    std::cout << ballCoor << std::endl;
    robotBallCoor = o.convertCoordinates(B, 0);
    robotCupCoor = o.convertCoordinates(A, 0.075);

    //Program robotten kører
    std::cout << "Start position" << std::endl;
    r.startingPosition();
    sleep(1);

    std::cout << "Pick up ball" << std::endl;
    //std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    std::cout << "Throw position" << std::endl;
    r.goToThrowPos(throwJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    std::cout << "Throw!" << std::endl;
    angle = 0;
    time = 0.13;
    max_acc = 0;
    speed = 0;
    //std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(max_acc, speed, robotCupCoor, angle, time, isSimulation);
    sleep(1);

    r.startingPosition();

    //Ligger data ind i databasen efter kast
    std::cout << "Is the ball in the cup? " << std::endl;
    std::cin >> hit;
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    throwPos = throwJointValuesDegrees;
    r.radConversion(throwPos);
    db.add_joint_throw_values(throwPos);
    db.add_kast_data(robotCupCoor, throwPos, time, hit, angle, max_acc, speed);
*/
    //Disconnecter fra robot og griber
  //  r.closeConnections(isSimulation);

    //Disconnecter fra databasen
   // db.disconnect();

    return 0;
}
