#include "objectdetection.h"

objectDetection::objectDetection()
{
    //Input for switch case depending on which robot cell you are working in:
    int i = 0;
    std::cout << "Input robot cell number between 1 and 4:" << std::endl;
    std::cin >> i;
    switch (i) {
    case 1: {
        //Reads the homography matrix from a file:
        cv::FileStorage fileH("../../../build-Homography-Desktop-Debug/H1.xml", cv::FileStorage::READ);
        fileH["H"] >> homography_matrix;
        fileH.release();
        //Prints homography matrix:
        std::cout << homography_matrix << std::endl;
        break;
    }
    case 2: {
        //Reads the homography matrix from a file:
        cv::FileStorage fileH("../../../build-Homography-Desktop-Debug/H2.xml", cv::FileStorage::READ);
        fileH["H"] >> homography_matrix;
        fileH.release();
        //Prints homography matrix:
        std::cout << homography_matrix << std::endl;
        break;
    }
    case 3: {
        //Reads the homography matrix from a file:
        cv::FileStorage fileH("../../../build-Homography-Desktop-Debug/H3.xml", cv::FileStorage::READ);
        fileH["H"] >> homography_matrix;
        fileH.release();
        //Prints homography matrix:
        std::cout << homography_matrix << std::endl;
        break;
    }
    case 4: {
        //Reads the homography matrix from a file:
        cv::FileStorage fileH("../../../build-Homography-Desktop-Debug/H4.xml", cv::FileStorage::READ);
        fileH["H"] >> homography_matrix;
        fileH.release();
        //Prints homography matrix:
        std::cout << homography_matrix << std::endl;
        break;
    }
    default: {
        std::cout << "Wrong input. Insert Restart and enter robot cell number between 1 and 4!" << std::endl;
        break;
    }
    }
}

void objectDetection::getSingleCupCoordinates(cv::Mat image, std::vector<cv::Point2f> &cupCoor)
{
    //Resize image and create a grayscale image for computation:
    cv::Mat gray;
    cv::resize(image, image, cv::Size(image.cols * 0.8, image.rows * 0.8));
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    //Applies blur to the grayscale image:
    cv::medianBlur(gray, gray, 5);
    //Creates a circles vector and fills it with the possible hough circles:
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                     gray.rows / 16,  // change this value to detect circles with different distances to each other
                     100, 30, 28, 36 // change the last two parameters
                     // (min_radius & max_radius) to detect larger circles
                     );
    //Finds the circle and circle center pixel of the hough circle and prints it:
    cv::Point2f center;
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        center = cv::Point2f(c[0], c[1]);
        // circle center:
        cv::circle(image, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
        // Prints circle center coordinates:
        std::cout << center << std::endl;
        // circle outline:
        int radius = c[2];
        cv::circle(image, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    }
    //Show detected hough circles:
    cv::imshow("Single Cup found", image);
    cv::waitKey();

    //converts to real world coordinates:
    //Needed variables:
    std::vector<cv::Point2f> real;
    std::vector<cv::Point2f> centers;

    //Put the circle center coordinate into a vector and converts to real world coordinates:
    centers.push_back(center);
    cv::perspectiveTransform(centers, real, homography_matrix);
    std::cout << real << std::endl;
    cupCoor = real;
}

void objectDetection::getSingleBallCoordinates(cv::Mat image, std::vector<cv::Point2f> &ballCoor)
{
    //Resize image and create a grayscale image for computation:
    cv::Mat gray;
    cv::resize(image, image, cv::Size(image.cols * 0.8, image.rows * 0.8));
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    //Applies blur to the grayscale image:
    cv::medianBlur(gray, gray, 5);
    //Creates a circles vector and fills it with the possible hough circles:
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                     gray.rows / 16,  // change this value to detect circles with different distances to each other
                     100, 30, 12, 16 // change the last two parameters
                     // (min_radius & max_radius) to detect larger circles
                     );
    //Finds the circle and circle center pixel of the hough circle and prints it:
    cv::Point2f center;
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        center = cv::Point2f(c[0], c[1]);
        // circle center:
        cv::circle(image, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
        // Prints circle center coordinates:
        std::cout << center << std::endl;
        // circle outline:
        int radius = c[2];
        cv::circle(image, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    }
    //Show detected hough circles:
    cv::imshow("Single ball found", image);
    cv::waitKey();

    //converts to real world coordinates:
    //Needed variables:
    std::vector<cv::Point2f> real;
    std::vector<cv::Point2f> centers;

    //Put the circle center coordinate into a vector and converts to real world coordinates:
    centers.push_back(center);
    cv::perspectiveTransform(centers, real, homography_matrix);
    std::cout << real << std::endl;
    ballCoor = real;
}

void objectDetection::getColouredCupCoordinates(cv::Mat mask, std::vector<cv::Point2f> &cupCoor)
{
    //Resizes the picture
    cv::resize(mask, mask, cv::Size(mask.cols * 0.8, mask.rows * 0.8));
    //Blures the picture (if nessesary)
    cv::GaussianBlur(mask, mask, cv::Size(3,3), 3, 0);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(mask, mask, kernel);
    //Creates a circles vector and fills it with the possible hough circles:
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1,
                     mask.rows / 8,  // change this value to detect circles with different distances to each other
                     30, 15, 28, 36 // change the last two parameters
                     // (min_radius & max_radius) to detect larger circles
                     );
    //Finds the circle and circle center pixel of the hough circle and prints it:
    cv::Point2f center;
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        center = cv::Point2f(c[0], c[1]);
        // circle center:
        cv::circle(mask, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
        // Prints circle center coordinates:
        std::cout << center << std::endl;
        // circle outline:
        int radius = c[2];
        cv::circle(mask, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    }
    //Show detected hough circles:
    cv::imshow("Cup found", mask);
    cv::waitKey();

    //converts to real world coordinates:
    //Needed variables:
    std::vector<cv::Point2f> real;
    std::vector<cv::Point2f> centers;

    //Put the circle center coordinate into a vector and converts to real world coordinates:
    centers.push_back(center);
    cv::perspectiveTransform(centers, real, homography_matrix);
    std::cout << real << std::endl;
    cupCoor = real;
}

void objectDetection::getColouredBallCoordinates(cv::Mat mask, std::vector<cv::Point2f> &ballCoor)
{
    //Resizes the picture
    cv::resize(mask, mask, cv::Size(mask.cols * 0.8, mask.rows * 0.8));
    //Blures the picture (if nessesary)
    cv::GaussianBlur(mask, mask, cv::Size(3,3), 3, 0);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(mask, mask, kernel);
    //Creates a circles vector and fills it with the possible hough circles:
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1,
                     mask.rows / 8,  // change this value to detect circles with different distances to each other
                     30, 15, 12, 16 // change the last two parameters
                     // (min_radius & max_radius) to detect larger circles
                     );
    //Finds the circle and circle center pixel of the hough circle and prints it:
    cv::Point2f center;
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        center = cv::Point2f(c[0], c[1]);
        // circle center:
        cv::circle(mask, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
        // Prints circle center coordinates:
        std::cout << center << std::endl;
        // circle outline:
        int radius = c[2];
        cv::circle(mask, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    }
    //Show detected hough circles:
    cv::imshow("Ball found", mask);
    cv::waitKey();

    //converts to real world coordinates:
    //Needed variables:
    std::vector<cv::Point2f> real;
    std::vector<cv::Point2f> centers;

    //Put the circle center coordinate into a vector and converts to real world coordinates:
    centers.push_back(center);
    cv::perspectiveTransform(centers, real, homography_matrix);
    std::cout << real << std::endl;
    ballCoor = real;
}

std::vector<double> objectDetection::convertCoordinates(cv::Point2f coordinate, double heightOfObjectm)
{
    //Loads robot 2 table calibration matrix and converts table coordinates to robot coordinates
    cv::Mat R2T;
    cv::FileStorage file("../../../build-RobotToTableCalib-Desktop-Debug/R2T2.xml", cv::FileStorage::READ);
    file["R2T"] >> R2T;
    file.release();
    std::cout << "Tjek R2T: " << R2T << std::endl;

    //Laver en matrixe til robot 2 table udregninger
    cv::Mat coorMat = cv::Mat_<float>(1,4);
    coorMat.at<float>(0,0) = coordinate.x/100;
    coorMat.at<float>(0,1) = coordinate.y/100;
    coorMat.at<float>(0,2) = heightOfObjectm;
    coorMat.at<float>(0,3) = 1;
    //Robot to table calibration
    coorMat = R2T * coorMat.reshape(1).t();
    //Lav til en vektor
    std::vector<double> robotCoor;
    if(coordinate.y < 35){
        robotCoor = {coorMat.at<float>(0,0), coorMat.at<float>(0,1), coorMat.at<float>(0,2), 3.14, 0, 0};
    }else if(coordinate.y >= 35){
        robotCoor = {coorMat.at<float>(0,0), coorMat.at<float>(0,1), coorMat.at<float>(0,2), 0, 3.14, 0};
    }else{
        std::cerr << "No cup or ball found!!!" << std::endl;
    }
    return robotCoor;
}
