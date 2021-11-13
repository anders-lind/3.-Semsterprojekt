#include <iostream>
#include "gripper.h"
#include<unistd.h>

using namespace std;

int main()
{
    //Opret forbindelse til griberen:
    std::string ip = "192.168.100.10";
    int port = 1000;
    // Datatype for griberen
    rl::hal::WeissWsg50 a(ip, port, 3.00f, 40.0f, 10);
    //rl::hal::WeissWsg50 a(ip);


    //Start griberen:
    a.open();
    a.start();
    if (!a.isConnected()) {
        cout << "Failed to connect!" << endl;
        return 1;
    }
    std::cout << "The connection is: " << a.isConnected() << std::endl;

    //doPreposition test
    a.doPrePositionFingers(0.04f, 0.4, false, false);
    sleep(3);
    std::cout << "Gripper has gripped" << std::endl;
    a.doPrePositionFingers(0.11f, 0.4, false, false);
    sleep(3);
    std::cout << "Gripper has released" << std::endl;

    //Stop griberen:
    a.stop();
    a.close();
    return 0;
}
