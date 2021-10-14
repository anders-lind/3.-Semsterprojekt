#include <iostream>
#include <vector>
#include<cmath>

#include "throw.h"


using namespace std;

int main()
{
    cout << "Hello World!" << endl;

    std::vector<double> start {10,0,0};
    std::vector<double> target {20,0,0};

    Throw throw1(start, target);
    std::vector<double> distance = throw1.getDistance();
    double speed = throw1.getSpeed();

    std::cout << "Speed = " << speed << std::endl;



    cout << "goodbye World!" << endl;
    return 0;
}
