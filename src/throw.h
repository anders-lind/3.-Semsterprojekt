#ifndef THROW_H
#define THROW_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>

class Throw
{
public:
    // Constructors
    Throw();
    Throw(std::vector<double> startCoordinates, std::vector<double> targetCoordinates);

    std::vector<double> calculateDistance();

    // Calculates and sets the speed member variable speed
    double calculateSpeed(Eigen::MatrixXd distance, double angle = 0);
    Eigen::MatrixXd calculate3DSpeed(Eigen::MatrixXd xk, Eigen::MatrixXd xm, double angle = 3.1415/4.0);



    // Setters and getters
    void setStartCoordinates(std::vector<double> startCoordinates) {_distanceVector = startCoordinates;};
    std::vector<double> getStartCoordinates() {return _startCoordinates;};

    void setTargetCoordinates(std::vector<double> targetCoordinates) {_targetCoordinates= targetCoordinates;};
    std::vector<double> getTargetCoordinates() {return _targetCoordinates;};

    std::vector<double> getDistance() {return _distanceVector;};
    double getSpeed() {return _speed;};

private:
    std::vector<double> _startCoordinates;
    std::vector<double> _targetCoordinates;
    std::vector<double> _distanceVector;

    double _throwAngle;
    double _speed;

    double g = 9.82;
    double PI = 3.1415;
};

#endif // THROW_H
