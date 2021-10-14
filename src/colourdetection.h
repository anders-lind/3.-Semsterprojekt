#ifndef COLOURDETECTION_H
#define COLOURDETECTION_H

//includes
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

class colourDetection
{
public:
    colourDetection();
    void DetectGreen(cv::Mat input, cv::Mat &output);
    void DetectBlue(cv::Mat input, cv::Mat &output);
    void DetectRed(cv::Mat input, cv::Mat &output);
    void DetectOrange(cv::Mat input, cv::Mat &output);
    void DetectYellow(cv::Mat input, cv::Mat &output);
    void CalibrateColours(cv::Mat input);
};

#endif // COLOURDETECTION_H
