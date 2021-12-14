#include "colourdetection.h"

colourDetection::colourDetection()
{

}

void colourDetection::DetectGreen(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the green colour wanted:
    int hmin = 49, smin = 236, vmin = 59;
    int hmax = 80, smax = 255, vmax = 124;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Green", output);
    cv::waitKey(0);
}

void colourDetection::DetectBlue(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the blue colour wanted:
    int hmin = 90, smin = 107, vmin = 28;
    int hmax = 101, smax = 255, vmax = 100;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Blue", output);
    cv::waitKey(0);
}

void colourDetection::DetectRed(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the red colour wanted:
    int hmin = 0, smin = 225, vmin = 45;
    int hmax = 3, smax = 255, vmax = 103;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Red", output);
    cv::waitKey(0);
}

void colourDetection::DetectOrange(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the orange colour wanted:
    int hmin = 1, smin = 255, vmin = 115;
    int hmax = 20, smax = 255, vmax = 219;
    //Convertion to HSV-colourspace from RGB-colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Orange", output);
    cv::waitKey(0);
}

void colourDetection::DetectYellow(cv::Mat input, cv::Mat &output)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV;
    //HSV values for the yellow colour wanted:
    int hmin = 20, smin = 255, vmin = 68;
    int hmax = 30, smax = 255, vmax = 204;
    //Convertere til HSV-colourspace fra RGB-colourspace
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sets the HSV colour values:
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);
    //Applying colourfilter to the image:
    cv::inRange(imgHSV, lower, upper, output);
    //Shows the colour mask:
    cv::imshow("Image Yellow", output);
    cv::waitKey(0);
}

void colourDetection::CalibrateColours(cv::Mat input)
{
    //Initializes temporary images for computation:
    cv::Mat imgHSV, mask;
    //initialize min og max HSV values for colourspace:
    int hmin = 0, smin = 0, vmin = 0;
    int hmax = 179, smax = 255, vmax = 255;
    //Converters to HSV Colourspace:
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);
    //Creates trackbars:
    cv::namedWindow("Trackbars", (640, 200));
    cv::createTrackbar("Hue min", "Trackbars", &hmin, 179);
    cv::createTrackbar("Hue max", "Trackbars", &hmax, 179);
    cv::createTrackbar("Sat min", "Trackbars", &smin, 255);
    cv::createTrackbar("Sat max", "Trackbars", &smax, 255);
    cv::createTrackbar("Val min", "Trackbars", &vmin, 255);
    cv::createTrackbar("Val max", "Trackbars", &vmax, 255);

    //Runs the program as a video while the values get updated from the trackbars:
    while (true){
        //Sets upper and lower limits:
        cv::Scalar lower(hmin, smin, vmin);
        cv::Scalar upper(hmax, smax, vmax);
        //Applying colourfilter to the image:
        cv::inRange(imgHSV, lower, upper, mask);
        //Show the colour mask:
        cv::imshow("Image mask", mask);
        cv::waitKey(1);
    }
}
