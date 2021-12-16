#include "colourdetection.h"
#include "objectdetection.h"
#include "pylon.h"
#include "robot.h"
#include "database.h"

int main(int argc, char* argv[])
{

    //----------------------------------Opretter forbindelse til databasen:---------------------------------------------
    Database db;
    db.create_tables();

    //-----------------------------------Kast 1 bold i en kop------------------------------------------------------------
    //----------------Machine vision
    //Loads image
    cv::Mat image;
    pylon cP;
    cP.getimage(image);
    image = cv::imread("billede.png");
    cv::imshow("billede", image);
    cv::waitKey(0);

    //------------------------------------Udkommenter dette hvis der skal udføres mere end 1 kast-------------------------
    //Creation of colour masks from corrected image:
    //cv::Mat blue, yellow, green, orange;
    //colourDetection a;

    //Calibrate colours
    //a.CalibrateColours(image);

    //Detect colours
    //a.DetectBlue(image, blue);
    //a.DetectYellow(image, yellow);
    //a.DetectGreen(image, green);
    //a.DetectOrange(image, orange);

    //Get coordinates
    objectDetection o;
    //std::vector<cv::Point2f> real er realworld coordinaterne der skal laves om til robot coordianter
    std::vector<cv::Point2f> ballCoor;
    std::vector<cv::Point2f> cupCoor;
    cv::Point2f A, B;

    //----------------------------------kommentaren er til mere end 1 kast, den ikke kommenterede del er til 1 kast------------------
    //o.getColouredCupCoordinates(yellow, cupCoor);
    //o.getColouredBallCoordinates(yellow, ballCoor);
    o.getSingleBallCoordinates(image, ballCoor);
    o.getSingleCupCoordinates(image, cupCoor);

    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    std::cout << ballCoor << std::endl;

    std::vector<double> robotBallCoor(6);
    std::vector<double> robotCupCoor(6);

    robotBallCoor = o.convertCoordinates(B, 0);
    robotCupCoor = o.convertCoordinates(A, 0.115);

    //-------------Robot
    //Is it a simulation or not:
    bool isSimulation = false;

    //Opretter forbindelse til robotten og griberen:
    robot r("192.168.100.49", "192.168.100.10");

    //---------------Til Sim---------------
    //robot r("127.0.0.1");

    //Program robotten kører
    std::cout << "Start position" << std::endl;
    r.startingPosition();
    sleep(1);

    std::cout << "Pick up ball" << std::endl;
    //Til sim
    //std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    std::cout << "Throw position" << std::endl;
    //ThrowPosition sættes
    //Vælger TCP-vinkel alt efter koppens position
    std::vector<double> throwJointValuesDegrees(6);
    std::cout << A.x << std::endl;
    if(A.x <= 30){
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -150};
    }else if(A.x >= 70){
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -47};
    }else{
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -101};
    }

    r.goToThrowPos(throwJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    std::cout << "Throw!" << std::endl;
    double angle = 0 * (M_PI/180);       /*45 * (M_PI/180)*/
    double time = -1;        // standard 1.3
    double qpp_max = 38;
    double max_acc = 0;
    double speed = 0;
    //Til sim
    //std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(max_acc, speed, robotCupCoor, angle, time, qpp_max, isSimulation);
    sleep(1);

    r.startingPosition();

    //Ligger data ind i databasen efter kast
    bool hit;
    std::cout << "Is the ball in the cup? " << std::endl;
    std::cin >> hit;
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    std::vector<double> throwPos = throwJointValuesDegrees;
    r.radConversion(throwPos);
    db.add_joint_throw_values(throwPos);
    db.add_kast_data(robotCupCoor, throwPos, time, hit, angle, max_acc, speed);


    //-------------------------------Til at udfører mere end 1 kast:-------------------------------------
/*
    //--------------Finder ny farve og kaster----------------
    o.getColouredCupCoordinates(orange, cupCoor);
    o.getColouredBallCoordinates(orange, ballCoor);
    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    std::cout << ballCoor << std::endl;
    robotBallCoor = o.convertCoordinates(B, 0);
    robotCupCoor = o.convertCoordinates(A, 0.09);

    //Program robotten kører
    std::cout << "Start position" << std::endl;
    r.startingPosition();
    sleep(1);

    std::cout << "Pick up ball" << std::endl;
    //Til sim
    //std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    std::cout << "Throw position" << std::endl;
    if(A.x <= 30){
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -150};
    }else if(A.x >= 70){
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -47};
    }else{
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -101};
    }
    r.goToThrowPos(throwJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    std::cout << "Throw!" << std::endl;
    angle = 0;
    time = -1;        // standard 1.3
    qpp_max = 38;
    max_acc = 0;
    speed = 0;
    //Til sim
    //std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(max_acc, speed, robotCupCoor, angle, time, qpp_max, isSimulation);
    sleep(1);

    r.startingPosition();

    //Ligger data ind i databasen efter kast
    std::cout << "Is the ball in the cup? " << std::endl;
    std::cin >> hit;
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    throwPos = throwJointValuesDegrees;
    r.radConversion(throwPos);
    db.add_joint_throw_values(throwPos);
    db.add_kast_data(robotCupCoor, throwPos, time, hit, angle, max_acc, speed);

    //-----------------------Finder ny farve og kaster--------------------------------
    o.getColouredCupCoordinates(blue, cupCoor);
    o.getColouredBallCoordinates(blue, ballCoor);
    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    std::cout << ballCoor << std::endl;
    robotBallCoor = o.convertCoordinates(B, 0);
    robotCupCoor = o.convertCoordinates(A, 0.09);

    //Program robotten kører
    std::cout << "Start position" << std::endl;
    r.startingPosition();
    sleep(1);

    std::cout << "Pick up ball" << std::endl;
    //Til sim
    //std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    std::cout << "Throw position" << std::endl;
    if(A.x <= 30){
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -150};
    }else if(A.x >= 70){
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -47};
    }else{
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -101};
    }
    r.goToThrowPos(throwJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    std::cout << "Throw!" << std::endl;
    angle = 0;
    time = -1;        // standard 1.3
    qpp_max = 38;
    max_acc = 0;
    speed = 0;
    //Til sim
    //std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(max_acc, speed, robotCupCoor, angle, time, qpp_max, isSimulation);
    sleep(1);

    r.startingPosition();

    //Ligger data ind i databasen efter kast
    std::cout << "Is the ball in the cup? " << std::endl;
    std::cin >> hit;
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    throwPos = throwJointValuesDegrees;
    r.radConversion(throwPos);
    db.add_joint_throw_values(throwPos);
    db.add_kast_data(robotCupCoor, throwPos, time, hit, angle, max_acc, speed);


    //-------------------------Finder ny farve og kaster-------------------------
    cP.getimage(image);
    image = cv::imread("billede.png");
    cv::imshow("billede", image);
    cv::waitKey(0);
    o.getColouredCupCoordinates(green, cupCoor);
    o.getSingleBallCoordinates(image, ballCoor);
    A.x = cupCoor.at(0).x;
    A.y = cupCoor.at(0).y;
    B.x = ballCoor.at(0).x;
    B.y = ballCoor.at(0).y;
    std::cout << ballCoor << std::endl;
    robotBallCoor = o.convertCoordinates(B, 0);
    robotCupCoor = o.convertCoordinates(A, 0.09);

    //Program robotten kører
    std::cout << "Start position" << std::endl;
    r.startingPosition();
    sleep(1);

    std::cout << "Pick up ball" << std::endl;
    //Til sim
    //std::vector<double> robotBallCoor = {0.108, -0.385, 0.221, 0, M_PI, 0};
    r.pickUpBall(robotBallCoor, isSimulation);
    sleep(1);

    std::cout << "Throw position" << std::endl;
    if(A.x <= 30){
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -150};
    }else if(A.x >= 70){
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -47};
    }else{
        throwJointValuesDegrees = {99, -90, 106, -124, -85, -101};
    }
    r.goToThrowPos(throwJointValuesDegrees);             // Default = {99, -90, 106, -124, - 85, -101};
    sleep(1);

    std::cout << "Throw!" << std::endl;
    angle = 0;
    time = -1;        // standard 1.3
    qpp_max = 38;
    max_acc = 0;
    speed = 0;
    //Til sim
    //std::vector<double> robotCupCoor = {0.220, -0.600, 0.001, 0, 0, 0};
    r.throwBall(max_acc, speed, robotCupCoor, angle, time, qpp_max, isSimulation);
    sleep(1);

    r.startingPosition();

    //Ligger data ind i databasen efter kast
    std::cout << "Is the ball in the cup? " << std::endl;
    std::cin >> hit;
    db.add_boldposition(robotBallCoor);
    db.add_kopposition(robotCupCoor);
    throwPos = throwJointValuesDegrees;
    r.radConversion(throwPos);
    db.add_joint_throw_values(throwPos);
    db.add_kast_data(robotCupCoor, throwPos, time, hit, angle, max_acc, speed);
*/

    //------------------Disconnecter fra robot og griber--------------
    r.closeConnections(isSimulation);

    //------------------Disconnecter fra databasen---------------------
    db.disconnect();

    return 0;
}
