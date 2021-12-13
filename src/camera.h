#ifndef CAMERA_H
#define CAMERA_H

//includes
#include <opencv2/opencv.hpp>

class cameraCali
{
public:
    cameraCali();
    void showImage(cv::Mat image);

private:
    cv::Mat mapX, mapY;     //For correction of image
};

#endif // CAMERA_H
