#ifndef PYLON_H
#define PYLON_H

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include "camera.h"

class pylon
{
public:
    pylon();
    void getimage(cv::Mat &output);
};

#endif // PYLON_H
