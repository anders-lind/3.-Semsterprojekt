#include "objectdetection.h"

objectDetection::objectDetection()
{

}

void objectDetection::findColouredCup(cv::Mat colourMask, cv::Mat &image)
{
    //Oprettelse af midlertidige containers
    cv::Mat imgBlured, imgCanny, imgDilation;
    //Preprossesing af billedet
    cv::GaussianBlur(colourMask, imgBlured, cv::Size(3,3), 3, 0);
    cv::Canny(imgBlured, imgCanny, 25, 75);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(imgCanny, imgDilation, kernel);

    //Oprettelse af nødvændige vectorer og find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(imgDilation, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> conPoly(contours.size());

    //filter for contours
    for(int i = 0; i < contours.size(); ++i){
        int area = cv::contourArea(contours[i]);
        std::cout << area << std::endl;

        if(area > 3000){
            float peri = cv::arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
            cv::drawContours(image, conPoly, i, cv::Scalar(255, 0, 255), 2);
            std::cout << conPoly[i].size() << std::endl;
        }
    }
    //Viser billedet med koppen markeret
    cv::imshow("Image with coloured cup found", image);
    cv::waitKey(0);
}

void objectDetection::findColouredBall(cv::Mat colourMask, cv::Mat &image)
{
    //Oprettelse af midlertidige containers
    cv::Mat imgBlured, imgCanny, imgDilation;
    //Preprossesing af billedet
    cv::GaussianBlur(colourMask, imgBlured, cv::Size(3,3), 3, 0);
    cv::Canny(imgBlured, imgCanny, 25, 75);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(imgCanny, imgDilation, kernel);

    //Oprettelse af nødvændige vectorer og find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(imgDilation, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> conPoly(contours.size());

    //filter for contours
    for(int i = 0; i < contours.size(); ++i){
        int area = cv::contourArea(contours[i]);
        std::cout << area << std::endl;

        if(area < 3000){
            float peri = cv::arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
            cv::drawContours(image, conPoly, i, cv::Scalar(255, 0, 255), 2);
            std::cout << conPoly[i].size() << std::endl;
        }
    }
    //Viser billedet med bolden markeret
    cv::imshow("Image with coloured ball found", image);
    cv::waitKey(0);
}

void objectDetection::findCup(cv::Mat image, cv::Mat &outputImage)
{
    cv::Mat imgGray, imgblured, imgCanny, imgDilation;
    //preprossesing
    cv::cvtColor(image, imgGray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(imgGray, imgblured, cv::Size(3,3), 3, 0);
    cv::Canny(imgblured,imgCanny, 25, 75);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(imgCanny, imgDilation, kernel);

    //Oprettelse af nødvændige vectorer og find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(imgDilation, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> conPoly(contours.size());

    //filter for contours
    for(int i = 0; i < contours.size(); ++i){
        int area = cv::contourArea(contours[i]);
        std::cout << area << std::endl;

        if(area > 3000){
            float peri = cv::arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
            cv::drawContours(image, conPoly, i, cv::Scalar(255, 0, 255), 2);
            std::cout << conPoly[i].size() << std::endl;
        }
    }
    //Viser billedet med bolden markeret
    cv::imshow("Image with cup found", image);
    cv::waitKey(0);
}

void objectDetection::findBall(cv::Mat image, cv::Mat &outputImage)
{
    cv::Mat imgGray, imgblured, imgCanny, imgDilation;
    //preprossesing
    cv::cvtColor(image, imgGray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(imgGray, imgblured, cv::Size(3,3), 3, 0);
    cv::Canny(imgblured,imgCanny, 25, 75);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::dilate(imgCanny, imgDilation, kernel);

    //Oprettelse af nødvændige vectorer og find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierachy;
    cv::findContours(imgDilation, contours, hierachy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> conPoly(contours.size());

    //filter for contours
    for(int i = 0; i < contours.size(); ++i){
        int area = cv::contourArea(contours[i]);
        std::cout << area << std::endl;

        if(area < 3000){
            float peri = cv::arcLength(contours[i], true);
            cv::approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
            cv::drawContours(image, conPoly, i, cv::Scalar(255, 0, 255), 2);
            std::cout << conPoly[i].size() << std::endl;
        }
    }
    //Viser billedet med bolden markeret
    cv::imshow("Image with ball found", image);
    cv::waitKey(0);
}
