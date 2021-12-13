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
    void getSingleCupCoordinates(cv::Mat image, std::vector<cv::Point2f> &cupCoor);
    void getSingleBallCoordinates(cv::Mat image, std::vector<cv::Point2f> &ballCoor);
    void getColouredCupCoordinates(cv::Mat mask, std::vector<cv::Point2f> &cupCoor);
    void getColouredBallCoordinates(cv::Mat mask, std::vector<cv::Point2f> &ballCoor);
    std::vector<double> convertCoordinates(cv::Point2f coordinate, double heightOfObjectm);

private:
    cv::Mat homography_matrix;  //For computing the homography
};

#endif // OBJECTDETECTION_H
