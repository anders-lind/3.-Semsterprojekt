//opencv includes
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
//Pylon includes
//#include <pylon/PylonIncludes.h>
//My program includes
#include "colourdetection.h"
#include "objectdetection.h"
#include "camera.h"
#include "pylon.h"
#include "gripper.h"
#include "robot.h"
#include<thread>
#include <chrono>
#include "Database.h"

int main(int argc, char* argv[])
{

    //Machine vision
    //Loads image
    cv::Mat image;
    pylon cP;
    cP.getimage(image);
    image = cv::imread("billede.png");
    cv::imshow("billede", image);
    cv::waitKey(0);

    //Creation of colour masks from corrected image:
    cv::Mat blue;
    //colourDetection a;
    //a.DetectBlue(image, blue);

    //Get coordinates
    objectDetection o;
    //std::vector<cv::Point2f> real er realworld coordinaterne der skal laves om til robot coordianter
    std::vector<cv::Point2f> ballCoor;
    std::vector<cv::Point2f> cupCoor;
    cv::Point2f A, B;
    //o.getColouredCupCoordinates(blue, cupCoor);
    //o.getColouredBallCoordinates(blue, ballCoor);
    o.getSingleBallCoordinates(image, ballCoor);
    o.getSingleCupCoordinates(image, cupCoor);
    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;

    //Loads robot 2 table calibration matrix and converts table coordinates to robot coordinates
    cv::Mat R2T;
    cv::FileStorage file("../../../build-RobotToTableCalib-Desktop-Debug/R2T2.xml", cv::FileStorage::READ);
    file["R2T"] >> R2T;
    file.release();
    std::cout << "Tjek: " << R2T << std::endl;
    cv::Mat cupCoorMat = cv::Mat_<float>(1,4);
    cv::Mat ballCoorMat = cv::Mat_<float>(1,4);
    cupCoorMat.at<float>(0,0) = A.x;
    cupCoorMat.at<float>(0,1) = A.y;
    cupCoorMat.at<float>(0,2) = 0;
    cupCoorMat.at<float>(0,3) = 1;
    ballCoorMat.at<float>(0,0) = B.x;
    ballCoorMat.at<float>(0,1) = B.y;
    ballCoorMat.at<float>(0,2) = 0;
    ballCoorMat.at<float>(0,3) = 1;
    cupCoorMat = cupCoorMat.reshape(1).t() * R2T;
    ballCoorMat = ballCoorMat.reshape(1).t() * R2T;
    cupCoorMat.pop_back();
    ballCoorMat.pop_back();
    std::cout << "Tjek: " << cupCoorMat << std::endl;
    std::cout << "Tjek: " << ballCoorMat << std::endl;
    std::vector<double> robotBallCoor = {ballCoorMat.at<float>(0,0), ballCoorMat.at<float>(0,1), ballCoorMat.at<float>(0,2), 3.14, 0, 0};
    std::vector<double> robotCupCoor = {cupCoorMat.at<float>(0,0), cupCoorMat.at<float>(0,1), cupCoorMat.at<float>(0,2), 0, 0, 0};

    //Robot
    //Robot test in cell:
    robot r("robotIp", "gripperIp");
    r.startingPosition();
    sleep(3);

    r.pickUpBall(robotBallCoor);
    sleep(3);

    r.goToThrowPos();
    sleep(3);

    /* For sim
    robot r;
    r.startingPosition();
    std::cout << "start" << std::endl;
    sleep(3);

    std::vector<double> coor = {0.24468073, -0.074169411, 0.174, 3.14 , 0, 0};
    r.pickUpBall(coor);
    std::cout << "Pick up ball" << std::endl;

    r.goToThrowPos();
    std::cout << "Throw" << std::endl;
    sleep(1);

    std::vector<double> maalPos = {0.218,-0.566, 0.228, 0, 0, 0};
    r.throwBall(maalPos);
    std::cout << "Throw Done" << std::endl;
    sleep(1);

    r.startingPosition();
    std::cout << "Back to start" << std::endl;
    sleep(1);
  */

    //Database

    //EDIT: Opdater kode og navn til MySQL. Opdater alle variabler med de ønskede værdier

//    Database db; //Opretter database objekt
//    db.drop_tables(); //Sletter alle tables i MySQL
//    db.disconnect(); //Afbryd forbindelse til database
//    db.connect(); //Forbind til database
//    db.create_tables(); //Skab alle tables
//    db.add_boldposition(); //Indsæt 1 af hver data
//    db.add_kopposition(); //Indsæt 1 af hver data
//    db.add_joint_nulpunkt(); //Indsæt 1 af hver data
//    db.add_joint_slut(); //Indsæt 1 af hver data
//    db.add_kast_data(); //Indsæt 1 af hver data



    return 0;
}
