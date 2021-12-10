#include "throw.h"

#include <iostream>
#include <vector>
#include <cmath>

using namespace std;


// Constructor
Throw::Throw()
{
    _startCoordinates = {0,0,0};
    _targetCoordinates = {0,0,0};
    _distanceVector = {0,0,0};
}


// // Constructor
// Throw::Throw(std::vector<double> startCoordinates, std::vector<double> targetCoordinates)
// {
//     /*////////// Null-initialize //////////*/
//     _startCoordinates = {0,0,0};
//     _targetCoordinates = {0,0,0};
//     _distanceVector = {0,0,0};


//     /*////////// Set start- and target-coordinates //////////*/
//     // Check if input vectors are 3 dimensions
//     if (startCoordinates.size() == 3){
//         for (unsigned long i = 0; i < 3; i++)
//             _startCoordinates[i] = startCoordinates[i];
//     }
//     else
//         std::cout << "ERROR: start-position vector bigger than 3 dimensioens. Start-position set to {0,0,0}" << std::endl;

//     // Check if input vectors are 3 dimensions
//     if (targetCoordinates.size() == 3){
//         for (unsigned long i = 0; i < 3; i++)
//             _targetCoordinates[i] = targetCoordinates[i];
//     }
//     else
//         std::cout << "ERROR: target-position vector bigger than 3 dimensioens. Target-position set to {0,0,0}" << std::endl;


//     /*////////// Calculate and set distance and speed //////////*/
//     calculateDistance();
//     calculateSpeed();
// }


// Calculates and sets the _distanceVector member variable
std::vector<double> Throw::calculateDistance()
{
    std::vector<double> distanceVector(3);
    for (int i = 0; i < 3; i++){
        _distanceVector[i] = _targetCoordinates[i] - _startCoordinates[i];
    }
    return _distanceVector;
}


// Calculates and sets the _speed member variable   ---- This methods returns correct values (certified by Anders 07/12)
double Throw::calculateSpeed(Eigen::MatrixXd distance, double angle)
{
    double x = distance(0);
    double y = distance(1);

    double h = 0;            // Height over floor (can be ignored, as it is equivelent to -y)
    double a = angle;        // Default = PI/4 = 45 degrees
    double g = 9.82;

    //cout << "x = " << x << " y = " << y << " a = " << a << " g = " << g << " h = " << h << endl;

    //      (2^(1/2) * x * (g/       (h - y + x*tan(a)))^(1/2))/(2*cos(a))
    //cout << "sqrt(2) * x * sqrt(g / (h - y + x*tan(a))) / 2*cos(a) " << endl;
    //cout << sqrt(2) << " * " << x <<" * " << "sqrt(" <<g << "/"<< "(" << h << " - " << y << " + " << x << " * " << tan(a)<<")) /" << 2 << " * "<< cos(a) << endl;
    _speed = (sqrt(2) * x * sqrt(g / (h - y + x*tan(a)))) / (2*cos(a));
    return _speed;
}


Eigen::MatrixXd Throw::calculate3DSpeed(Eigen::MatrixXd xk, Eigen::MatrixXd xm, double angle)
{
    Eigen::MatrixXd distance = xm-xk;
    cout << "Distance = \n" << distance << " norm() = " <<distance.norm() << endl;

    // Kastet deles op i en vertikal og horisontal del
    Eigen::MatrixXd distanceH(2,1), distanceV(1,1);
    distanceH << distance(0), distance(1);
    distanceV << distance(2);

    double horizontal = distanceH.norm();
    double vertical = distanceV(0,0);
    cout << "Horizontal and vertical distance = [" << horizontal << ", " << vertical << "]" << endl;

    // En 2D distance laves
    Eigen::MatrixXd distance2D(2,1);
    distance2D << horizontal,vertical;

    // Calculate speed in 2D space
    double speed = calculateSpeed(distance2D, angle);


    // // Calculate a set of spherical coordinates
    // double theta = M_PI/2 - angle ;
    // double r = speed;
    // double phi = acos((distanceH(0)*1 + distanceH(1)*0) / (distanceH.norm() * 1));  // Calculate phi (angle to throw direction from x in xy-plane)

    // double x = r * sin(theta) * cos(phi);
    // double y = r * sin(theta) * sin(phi);
    // double z = r * cos(theta);

    // Eigen::MatrixXd speed3D(3,1);
    // speed3D << x,y,z;

    // cout << "Speed3D = \n" << speed3D << endl;
    
    // return speed3D;


    double speedXYnorm = speed * cos(angle);
    
    Eigen::MatrixXd speedXY(2,1);
    speedXY << speedXYnorm * distanceH / distanceH.norm();

    double speedZ = speed*sin(angle);

    Eigen::MatrixXd speed3D(3,1);
    speed3D << speedXY, speedZ;

    return speed3D;

    
    
    // Eigen::MatrixXd xkLevel(3,1);
    // xkLevel << 0.207, -0.251, 0;

    // // Lav en vinkel på 0 grader til vandret
    // Eigen::MatrixXd distanceXY = distance;
    // distanceXY(2) = 0;
    // double heightDiff = xk(2)-xm(2);
    // cout << "height diff " << heightDiff << endl;

    // // Calculate 2D speed
    // _distanceVector.at(0) = distanceXY.norm();
    // _distanceVector.at(1) = heightDiff;
    // double angle = 0;
    // double speed = calculateSpeed(distance, angle);

    // // Calculate 3D speed
    // Eigen::MatrixXd speed3D;
    // Eigen::MatrixXd unitVector = distanceXY / distanceXY.norm();
    // speed3D = unitVector * speed;

    // return speed3D;
}
    // Calculate intermidiate values
