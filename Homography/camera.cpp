#include "camera.h"

camera::camera()
{

    //initialize variables:
    cv::Size frameSize(1440, 1080);
    //Creates temporary variables to contain intrinsic camera matrix and distortion coefficients:
    cv::Mat tmp;                //intrinsic camera matrix
    cv::Vec<float, 5> tmp1;     //distortion coefficients

    //Reads the K file and creates the K variable:
    cv::FileStorage fileK("../build-Calibreringsprojekt-Desktop-Debug/K3.xml", cv::FileStorage::READ);
    fileK["K"] >> tmp;
    fileK.release();
    cv::Matx33f K(tmp);

    //Reads the k file and creates the k variable:
    cv::FileStorage filek("../build-Calibreringsprojekt-Desktop-Debug/k3.xml", cv::FileStorage::READ);
    filek["k"] >> tmp1;
    filek.release();
    cv::Vec<float, 5> k(tmp1);

    //Prints our varaibles to check if they have the right values
    std::cout << K << std::endl;
    std::cout << k << std::endl;

    // Precompute lens correction interpolation
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);
}

void camera::showImage(cv::Mat image)
{
    cv::Mat imgUndistorted;
    // 5. Remap the image using the precomputed interpolation maps.
    cv::remap(image, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
    cv::resize(imgUndistorted, imgUndistorted, cv::Size(imgUndistorted.cols * 0.8, imgUndistorted.rows * 0.8));
    // Display
    cv::namedWindow("Undistorted Image", cv::WINDOW_AUTOSIZE );
    cv::imshow("Undistorted Image", imgUndistorted);
    cv::waitKey(0);
}
