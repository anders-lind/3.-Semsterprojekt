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
    void findColouredCup(cv::Mat colourMask, cv::Mat &image);
    void findColouredBall(cv::Mat colourMask, cv::Mat &image);
    void findCup(cv::Mat image, cv::Mat &outputImage);
    void findBall(cv::Mat image, cv::Mat &outputImage);
};

#endif // OBJECTDETECTION_H
