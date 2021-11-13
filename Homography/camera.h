#ifndef CAMERA_H
#define CAMERA_H
//includes
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

class camera
{
public:
    camera();
    void showImage(cv::Mat image);
private:
    cv::Mat mapX, mapY;

};

#endif // CAMERA_H
