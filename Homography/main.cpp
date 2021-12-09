#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "camera.h"
#include "mouse.h"

//Initializes two vectors that is going to contain the points for computation of the homography:
std::vector<cv::Point2f> obj;
std::vector<cv::Point2f> scene;

//Method to calculate the homography:
void get_homographyMatrix()
{
    //Initialization of the vector that is going to contain the calculated points:
    std::vector<cv::Point2f> obj_projection;

    //Creates 4 2D coordinate sets that contains the real distance in cm between the four image coordinates:
    cv::Point2f A,B,C,D;
    A.x=0;
    A.y=0;

    B.x=75;
    B.y=0;

    C.x=75;
    C.y=75;

    D.x=0;
    D.y=75;
    //Inserts the 4 2D coordinates into the obj vector for computation:
    obj.push_back(A);
    obj.push_back(B);
    obj.push_back(C);
    obj.push_back(D);

    //Calculates the homography matrix from obj and scene via getPerspectiveTransform and prints out the matrix:
    cv::Mat homography_matrix = cv::getPerspectiveTransform(scene, obj);
    std::cout<<"Estimated Homography Matrix is:" <<std::endl;
    std::cout<< homography_matrix <<std::endl;

    //Saves the homogrphy matrix to a file:
    cv::FileStorage fileH("H2.xml", cv::FileStorage::WRITE);
    fileH << "H" << cv::Mat(homography_matrix);

    //Test that we get the obj coordinates from the scene vector vi the homography matrix:
    cv::perspectiveTransform( scene, obj_projection, homography_matrix);
    for(std::size_t i=0;i<obj_projection.size();i++)
    {
        std::cout<<obj_projection.at(i).x <<"," <<obj_projection.at(i).y<<std::endl;
    }
}

//Method to get the 4 scene coordinates in from the picture by clicking with the mouse:
void mouse(int event, int x, int y, int flag, void *param){
    //An if statement that allows 4 clicks for the 4 points and then prints out the coordinates and saves them into the scene vector:
    if (event == cv::EVENT_LBUTTONDOWN and scene.size() < 4)
    {
        std::cout << "Coordinate - position (" << x << ", " << y << ")" << std::endl;
        scene.push_back(cv::Point2f(x, y));
    }
}

int main(int argc, char** argv)
{
    //importer billede
    if ( argc != 2 )
    {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }
    cv::Mat img;
    img = cv::imread( argv[1], cv::IMREAD_COLOR);
    if ( !img.data )
    {
        printf("No image data \n");
        return -1;
    }

    //Shows image:
    camera a;
    a.showImage(img);
    //Sets which window the program should listen for mouse inputs:
    cv::setMouseCallback("Undistorted Image", mouse, NULL);
    cv::waitKey(0);
    //Calculates the homography after input:
    get_homographyMatrix();

    return 0;
}
