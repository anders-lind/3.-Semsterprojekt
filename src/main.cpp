//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

//Pylon includes
#include <pylon/PylonIncludes.h>

//My program includes
#include "colourdetection.h"
#include "objectdetection.h"
#include "camera.h"
#include "pylon.h"
#include "gripper.h"
#include "robot.h"
#include "database.h"

#include <thread>
#include <chrono>

using namespace std;


int main(int argc, char* argv[])
{

    // //Machine vision
    // //Loads image
    // cv::Mat image;
    // pylon cP;
    // cP.getimage(image);
    // image = cv::imread("billede.png");
    // cv::imshow("billede", image);
    // cv::waitKey(0);

    // //Creation of colour masks from corrected image:
    // cv::Mat blue, yellow;
    // colourDetection a;
    // //a.CalibrateColours(image);

    // a.DetectBlue(image, blue);
    // a.DetectYellow(image, yellow);

    // //Get coordinates
    // objectDetection o;
    // //std::vector<cv::Point2f> real er realworld coordinaterne der skal laves om til robot coordianter
    // std::vector<cv::Point2f> ballCoor;
    // std::vector<cv::Point2f> cupCoor;
    // cv::Point2f A, B;
    // o.getColouredCupCoordinates(yellow, cupCoor);
    // o.getColouredBallCoordinates(yellow, ballCoor);
    // //o.getSingleBallCoordinates(blue, ballCoor);
    // //o.getSingleCupCoordinates(blue, cupCoor);
    // A.x = cupCoor.at(0).x;
    // A.y = cupCoor.at(0).y;
    // B.x = ballCoor.at(0).x;
    // B.y = ballCoor.at(0).y;
    // std::cout << ballCoor << std::endl;

    // std::vector<double> robotBallCoor(6);
    // std::vector<double> robotCupCoor(6);

    // robotBallCoor = o.convertCoordinates(B, 0);
    // robotCupCoor = o.convertCoordinates(A, 0.075);

    //Robot
    //Robot test in cell:

    //bool isSimulation = false;
    // robot r("192.168.100.49", "192.168.100.10");
    bool isSimulation = true;
    robot r("127.0.0.1");

    cout << "Start position" << endl;
    r.startingPosition();
    sleep(1);

    cout << "Pick up ball" << endl;
    std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    cout << "Throw position" << endl;
    std::vector<double> TrowJointValuesDegrees = {99, -90, 106, -124, - 85, -101};
    r.goToThrowPos(TrowJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    cout << "Throw!" << endl;
    double angle = 0;
    double time = 0.13;
    std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(robotCupCoor, angle, time, isSimulation);
    sleep(1);

    r.startingPosition();
/*
    o.getColouredCupCoordinates(yellow, cupCoor);
    o.getColouredBallCoordinates(yellow, ballCoor);
    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    robotBallCoor = o.convertCoordinates(B);
    robotCupCoor = o.convertCoordinates(A);

    r.pickUpBall(robotBallCoor);
    sleep(1);

    r.goToThrowPos();
    sleep(1);

    r.throwBall(robotCupCoor);
    sleep(1);

    r.startingPosition();
*/
    r.closeConnections(isSimulation);

    //For sim
/*
    robot r;
    r.startingPosition();
    std::cout << "start" << std::endl;
    sleep(1);

    //std::vector<double> coor = {0.24468073, -0.074169411, 0.174, 3.14 , 0, 0};
    //r.pickUpBall(robotBallCoor);
    //std::cout << "Pick up ball" << std::endl;

    //r.goToThrowPos();
    //std::cout << "Throw" << std::endl;
    //sleep(1);

    //std::vector<double> maalPos = {0.218,-0.566, 0.228, 0, 0, 0};
    //r.throwBall(robotCupCoor);
    //std::cout << "Throw Done" << std::endl;
    //sleep(1);

    //r.startingPosition();
    //std::cout << "Back to start" << std::endl;
    //sleep(1);



*/
    //Database
    /*
    Database db;
    db.drop_tables();
    // db.disconnect();
    // db.connect();
    db.create_tables();
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    //db.add_joint_nulpunkt();
    //db.add_joint_slut();
    //db.add_kast_data();
*/
    return 0;
}
