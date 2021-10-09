#include "throw.h"

#include <iostream>
#include <vector>
#include <cmath>


Throw::Throw()
{
    _startCoordinates = {0,0,0};
    _targetCoordinates = {0,0,0};
    _distanceVector = {0,0,0};
}


Throw::Throw(std::vector<double> startCoordinates, std::vector<double> targetCoordinates)
{
    /*////////// Null-initialize //////////*/
    _startCoordinates = {0,0,0};
    _targetCoordinates = {0,0,0};
    _distanceVector = {0,0,0};


    /*////////// Set start- and target-coordinates //////////*/
    // Check if input vectors are 3 dimensions
    if (startCoordinates.size() == 3){
        for (unsigned long i = 0; i < 3; i++)
            _startCoordinates[i] = startCoordinates[i];
    }
    else
        std::cout << "ERROR: start-position vector bigger than 3 dimensioens. Start-position set to {0,0,0}" << std::endl;

    // Check if input vectors are 3 dimensions
    if (targetCoordinates.size() == 3){
        for (unsigned long i = 0; i < 3; i++)
            _targetCoordinates[i] = targetCoordinates[i];
    }
    else
        std::cout << "ERROR: target-position vector bigger than 3 dimensioens. Target-position set to {0,0,0}" << std::endl;


    /*////////// Calculate and set distance and speed //////////*/
    calculateDistance();
    calculateSpeed();
}


// Calculates and sets the _distanceVector member variable
std::vector<double> Throw::calculateDistance()
{
    std::vector<double> distanceVector(3);
    for (int i = 0; i < 3; i++){
        _distanceVector[i] = _targetCoordinates[i] - _startCoordinates[i];
    }
    return _distanceVector;
}


// Calculates and sets the _speed member variable
double Throw::calculateSpeed()
{
    std::cout << "Calculating speed" << std::endl;
    double x = _distanceVector[0];
    double y = _distanceVector[1];

    double b = 1;
    double c = 1;
    double a = PI/2;

    //std::cout << "sqrt( -" << g << " * " << x <<"^2" << " + 2*" << b << " * " << g << " * " << x << " - " << b << "^2" << " * " << g << " )/( 2 *  cos( " << a << ")*(- " << c << "+" << y << "+" << b << "*tan("<<a<<")-" <<x<<"*tan("<<a<<"))))";
    _speed = sqrt((-g * pow(x,2) + 2*b*g*x - pow(b,2)*g)/(2 * cos(a)*(-c+y+b*tan(a)-x*tan(a))));
    return _speed;
}
