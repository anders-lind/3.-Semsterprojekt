//opencv includes
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/videoio.hpp>
// #include <opencv2/imgproc.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/calib3d.hpp>
//Pylon includes
//#include <pylon/PylonIncludes.h>
//My program includes
// #include "colourdetection.h"
// #include "objectdetection.h"
// #include "camera.h"
// #include "pylon.h"
// #include "gripper.h"
#include "robot.h"
#include <thread>
#include <chrono>
#include <string>

// kun brugt til debugging, kan fjernes
#include "throw.h"
#include "jacobian.h"
#include <vector>
#include <eigen3/Eigen/Dense>

using namespace std;


int main(int argc, char* argv[])
{
    /*
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
*/

    /* --- Robot --- */
    // // Connect to robot
    string IP = "127.0.0.1";
    robot r(IP);
    
    // // // Go to start pos
    // // cout << "Go to start position" << endl;
    // // r.startingPosition();
    // // sleep(1);

    // // // Go to ball pos
    // // vector<double> coordBall = {0.24468073, -0.074169411, 0.174, 3.14 , 0, 0};
    // // cout << "pick up ball" << endl;
    // // r.pickUpBall(coordBall);
    // // sleep(1);

    // Go to maal pos
    //vector<double> coordMaal = {0.218,-0.566, 0.228, 0, 0, 0};
    vector<double> coordMaal = {0, -0.7, 0, M_PI, 0, 0};
    cout << "Go to maal position" << endl;
    r.moveL(coordMaal);
    sleep(1);

    // Go to throw pos
    std::cout << "Go to throw position" << std::endl;
    vector<double> throwPos = {0, -0.2, 0.5, 0, 0, 0};
    //r.goToThrowPos();
    r.moveL(throwPos);
    sleep(1);

    //Trow ball
    cout << "Begin throw preperations" << endl;
    double angle = 0;
    double time = 0.2;
    r.throwBall(coordMaal,angle,time);       // throwPos = {0.4674, -0.1943, -0.043, 0, 0, 0};
    std::cout << "Throw Done" << std::endl;
    sleep(1);

    // // Go to start
    // std::cout << "Back to start" << std::endl;
    // r.startingPosition();
    // sleep(1);

    // cout << "Test throw direction" << endl;
    // Eigen::MatrixXd xk(6,1);
    // Eigen::MatrixXd xm(6,1);

    // xk << 0.19292, -0.46815, 0.35731, 0, 0, 0;
    // xm << coordMaal.at(0), coordMaal.at(1), coordMaal.at(2), 0, 0, 0;
    // Throw t;
    // Eigen::MatrixXd speed3DEig = t.calculate3DSpeed(xk,xm,M_PI/4);
    // cout << "Speed3DEig = " << speed3DEig << endl;

    // vector<double> speed3D(6);
    // double k = 4;
    // speed3D.at(0) = speed3DEig(0)*k; speed3D.at(1) = speed3DEig(1)*k; speed3D.at(2) = speed3DEig(2)*k; speed3D.at(3) = 0; speed3D.at(4) = 0; speed3D.at(5) = 0;
    // cout << "Speed3D =  [" << speed3D.at(0) << ", " << speed3D.at(1) << ", " << speed3D.at(2) << ", " << speed3D.at(3) << ", " << speed3D.at(4) << ", " << speed3D.at(5) << "]" << endl;
    // r.moveL(speed3D);
    // sleep(2);



    

    // // /* --- Test af Throw klasse --- */
    // Eigen::MatrixXd xk(6,1);
    // vector<double> tmp = {0, 0, 0, 0, 0, 0};
    // xk = jacobian::vec2Eig(tmp);

    // Eigen::MatrixXd xm(6,1); 
    // tmp = {0, 0, -100, 0, 0, 0};
    // xm = jacobian::vec2Eig(tmp);

    // Eigen::MatrixXd distance(6,1);
    // distance = xm - xk;

    // Throw T;
    // double angle = 0;
    // Eigen::MatrixXd speed3D = T.calculate3DSpeed(xk, xm, angle);
    // cout << "Total speed to reach distance = " << speed3D << "size = " << speed3D.norm() << endl; 

    //Database


    return 0;
}
