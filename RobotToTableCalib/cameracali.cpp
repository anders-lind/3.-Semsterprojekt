#include "cameracali.h"

cameraCali::cameraCali()
{
    //initializeses variable
    cv::Size frameSize(1440, 1080);
    //Variables to contain intrinsic camera matrix and distortion coefficients
    cv::Mat tmp;                //Intrinsic camera matrix
    cv::Vec<float, 5> tmp1;     //Distortion coefficients

    //Input for switch case depending on which robot cell you are working in:
    int x = 0;
    std::cout << "Input robot cell number between 1 and 4:" << std::endl;
    std::cin >> x;
    switch (x) {
    case 1: {
        //Reads the K file to the K variable
        cv::FileStorage fileK("../build-Calibreringsprojekt-Desktop-Debug/K1.xml", cv::FileStorage::READ);
        fileK["K"] >> tmp;
        fileK.release();

        //Reads the k file to the k variable
        cv::FileStorage filek("../build-Calibreringsprojekt-Desktop-Debug/k1.xml", cv::FileStorage::READ);
        filek["k"] >> tmp1;
        filek.release();
        break;
    }
    case 2:{
        //Reads the K file to the K variable
        cv::FileStorage fileK("../build-Calibreringsprojekt-Desktop-Debug/K2.xml", cv::FileStorage::READ);
        fileK["K"] >> tmp;
        fileK.release();

        //Reads the k file to the k variable
        cv::FileStorage filek("../build-Calibreringsprojekt-Desktop-Debug/k2.xml", cv::FileStorage::READ);
        filek["k"] >> tmp1;
        filek.release();
        break;
    }
    case 3: {
        //Reads the K file to the K variable
        cv::FileStorage fileK("../build-Calibreringsprojekt-Desktop-Debug/K3.xml", cv::FileStorage::READ);
        fileK["K"] >> tmp;
        fileK.release();

        //Reads the k file to the k variable
        cv::FileStorage filek("../build-Calibreringsprojekt-Desktop-Debug/k3.xml", cv::FileStorage::READ);
        filek["k"] >> tmp1;
        filek.release();
        break;
    }
    case 4: {
        //Reads the K file to the K variable
        cv::FileStorage fileK("../build-Calibreringsprojekt-Desktop-Debug/K4.xml", cv::FileStorage::READ);
        fileK["K"] >> tmp;
        fileK.release();

        //Reads the k file to the k variable
        cv::FileStorage filek("../build-Calibreringsprojekt-Desktop-Debug/k4.xml", cv::FileStorage::READ);
        filek["k"] >> tmp1;
        filek.release();
        break;
    }
    default: {
        std::cout << "Wrong input. Insert Restart and enter robot cell number between 1 and 4!" << std::endl;
        break;
    }
    }
    cv::Matx33f K(tmp);
    cv::Vec<float, 5> k(tmp1);
    //Prints the variables
    std::cout << K << std::endl;
    std::cout << k << std::endl;
    // Precompute lens correction interpolation:
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1, mapX, mapY);
}

void cameraCali::showImage(cv::Mat image)
{
    // Remap the image using the precomputed interpolation maps:
    cv::remap(image, image, mapX, mapY, cv::INTER_LINEAR);
    //Resizes the image:
    cv::resize(image, image, cv::Size(image.cols * 0.8, image.rows * 0.8));
    // Display:
    cv::imshow("undistorted image", image);
    cv::waitKey(0);
}
