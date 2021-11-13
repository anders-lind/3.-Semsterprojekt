#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H
//Includes
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class objectDetection
{
public:
    objectDetection();
    void getSingleCupCoordinates(cv::Mat image);
    void getSingleBallCoordinates(cv::Mat image);
    void getColouredCupCoordinates(cv::Mat mask);
    void getColouredBallCoordinates(cv::Mat mask);
private:
    cv::Mat homography_matrix;  //For computing the homography
};

#endif // OBJECTDETECTION_H
