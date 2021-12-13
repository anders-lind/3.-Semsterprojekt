#ifndef CAMERACALI_H
#define CAMERACALI_H
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

#endif // CAMERACALI_H
