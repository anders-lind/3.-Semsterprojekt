#include "colourdetection.h"

colourDetection::colourDetection()
{

}
void colourDetection::DetectGreen(cv::Mat input, cv::Mat &output)
{
    //Opretter nødvændige midlertidige billeder til behandling
    cv::Mat imgHSV, mask;
    //Værdier for farven der skal findes
    int hmin = 60, smin = 76, vmin = 96;
    int hmax = 84, smax = 200, vmax = 236;
    //Convertere til HSV-colourspace fra RGB-colourspace
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sætter værdierne for farven der skal detekteres
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applyer farvefilter til billedet
    cv::inRange(imgHSV, lower, upper, output);
    //Viser de isolerede farver i billedet
    cv::imshow("Image Green", output);
    cv::waitKey(0);
}

void colourDetection::DetectBlue(cv::Mat input, cv::Mat &output)
{
    //Opretter nødvændige midlertidige billeder til behandling
    cv::Mat imgHSV, mask;
    //Værdier for farven der skal findes
    int hmin = 88, smin = 88, vmin = 42;
    int hmax = 102, smax = 255, vmax = 255;
    //Convertere til HSV-colourspace fra RGB-colourspace
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sætter værdierne for farven der skal detekteres
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    //Applyer farvefilter til billedet
    cv::inRange(imgHSV, lower, upper, output);
    //Viser de isolerede farver i billedet
    cv::imshow("Image Blue", output);
    cv::waitKey(0);
}

void colourDetection::DetectRed(cv::Mat input, cv::Mat &output)
{
    //Opretter nødvændige midlertidige billeder til behandling
    cv::Mat imgHSV, mask;
    //Værdier for farven der skal findes
    int hmin = 174, smin = 87, vmin = 110;
    int hmax = 179, smax = 234, vmax = 255;
    //Convertere til HSV-colourspace fra RGB-colourspace
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sætter værdierne for farven der skal detekteres
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);
    //Applyer farvefilter til billedet
    cv::inRange(imgHSV, lower, upper, output);
    //Viser de isolerede farver i billedet
    cv::imshow("Image Red", output);
    cv::waitKey(0);
}

void colourDetection::DetectOrange(cv::Mat input, cv::Mat &output)
{
    //Opretter nødvændige midlertidige billeder til behandling
    cv::Mat imgHSV, mask;
    //Værdier for farven der skal findes
    int hmin = 0, smin = 161, vmin = 145;
    int hmax = 21, smax = 201, vmax = 255;
    //Convertere til HSV-colourspace fra RGB-colourspace
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sætter værdierne for farven der skal detekteres
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);
    //Applyer farvefilter til billedet
    cv::inRange(imgHSV, lower, upper, output);
    //Viser de isolerede farver i billedet
    cv::imshow("Image Orange", output);
    cv::waitKey(0);
}

void colourDetection::DetectYellow(cv::Mat input, cv::Mat &output)
{
    //Opretter nødvændige midlertidige billeder til behandling
    cv::Mat imgHSV, mask;
    //Værdier for farven der skal findes
    int hmin = 21, smin = 109, vmin = 177;
    int hmax = 31, smax = 227, vmax = 255;
    //Convertere til HSV-colourspace fra RGB-colourspace
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);

    //Sætter værdierne for farven der skal detekteres
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);
    //Applyer farvefilter til billedet
    cv::inRange(imgHSV, lower, upper, output);
    //Viser de isolerede farver i billedet
    cv::imshow("Image Yellow", output);
    cv::waitKey(0);
}

void colourDetection::CalibrateColours(cv::Mat input)
{
    //opretter midlertidige containere
    cv::Mat imgHSV, mask;
    //initialisere min og max værdier for colourspace
    int hmin = 0, smin = 0, vmin = 0;
    int hmax = 179, smax = 255, vmax = 255;
    //Convertere Colourspace
    cv::cvtColor(input, imgHSV, cv::COLOR_BGR2HSV);
    //Laver trackbars
    cv::namedWindow("Trackbars", (640, 200));
    cv::createTrackbar("Hue min", "Trackbars", &hmin, 179);
    cv::createTrackbar("Hue max", "Trackbars", &hmax, 179);
    cv::createTrackbar("Sat min", "Trackbars", &smin, 255);
    cv::createTrackbar("Sat max", "Trackbars", &smax, 255);
    cv::createTrackbar("Val min", "Trackbars", &vmin, 255);
    cv::createTrackbar("Val max", "Trackbars", &vmax, 255);

    //Køre programmet som en vidoe mens værdier opdateres
    while (true){
    cv::Scalar lower(hmin, smin, vmin);
    cv::Scalar upper(hmax, smax, vmax);

    cv::inRange(imgHSV, lower, upper, mask);

    cv::imshow("Image mask", mask);
    cv::waitKey(1);
}
}
