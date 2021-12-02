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


int main(int argc, char* argv[])
{
    //Machine vision
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
    std::vector<double> robotCoor= {ballCoorMat.at<float>(0,0), ballCoorMat.at<float>(0,1), ballCoorMat.at<float>(0,2)};
    //Robot
    //robot r;
    //r.startingPosition();
    //r.pickUpBall(robotCoor);
    //r.throwBall();

    //Database


    return 0;
}
