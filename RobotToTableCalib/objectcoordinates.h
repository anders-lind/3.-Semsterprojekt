#ifndef OBJECTCOORDINATES_H
#define OBJECTCOORDINATES_H
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class ObjectCoordinates
{
public:
    ObjectCoordinates();
    void getSingleBallCoordinates(cv::Mat image, std::vector<cv::Point2f> coordinates);
private:
    cv::Mat homography_matrix;  //For computing the homography
};

#endif // OBJECTCOORDINATES_H
