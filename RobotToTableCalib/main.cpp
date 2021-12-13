#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

int main(int argc, char** argv)
{
    //Vectors for table and robot coordinates
    std::vector<cv::Point3f> table;
    std::vector<cv::Point3f> robot;
    //Table coordinates and robot coordinates:
    cv::Point3f A, B, C, D, rA, rB, rC, rD;
    A.x = 0.025;
    A.y = 0.125;
    A.z = 0;

    B.x = 0.275;
    B.y = 0.125;
    B.z = 0;

    C.x = 0.275;
    C.y = 0.575;
    C.z = 0;

    D.x = 0.025;
    D.y = 0.575;
    D.z = 0;

    table.push_back(A);
    table.push_back(B);
    table.push_back(C);
    table.push_back(D);

    rA.x = 254.25/1000;
    rA.y = -114.37/1000;
    rA.z = 0.174;

    rB.x = 341.15/1000;
    rB.y = -354.66/1000;
    rB.z = 0.174;

    rC.x = -66.00/1000;
    rC.y = -506.26/1000;
    rC.z = 0.174;

    rD.x = -163.20/1000;
    rD.y = -266.08/1000;
    rD.z = 0.174;

    robot.push_back(rA);
    robot.push_back
            (rB);
    robot.push_back(rC);
    robot.push_back(rD);

    //Creates a cv::Mat from the vectors containing coordinates and transposes it
    cv::Mat table1 = cv::Mat(table);
    cv::Mat robot1 = cv::Mat(robot);
    cv::Mat tableTrans = cv::Mat(table).reshape(1).t();
    cv::Mat robotTrans = cv::Mat(robot).reshape(1).t();

    std::cout << "table" << tableTrans << std::endl;
    std::cout << "robot" << robotTrans << std::endl;

    //Calculates the mean value og the coordinates
    cv::Scalar meanTable = cv::mean(table1);
    cv::Scalar meanRobot = cv::mean(robot1);
    cv::Point3f tmp;
    cv::Point3f tmp1;
    tmp.x = meanTable[0];
    tmp.y = meanTable[1];
    tmp.z = meanTable[2];
    tmp1.x = meanRobot[0];
    tmp1.y = meanRobot[1];
    tmp1.z = meanRobot[2];
    cv::Mat tableMean(tmp);
    cv::Mat robotMean(tmp1);

    std::cout << "meanTable" << meanTable << std::endl;
    std::cout << tableMean << std::endl;
    std::cout << "meanRobot" << meanRobot << std::endl;
    std::cout << robotMean << std::endl;

    //Calculates the zero centroids
    cv::Mat Q1 = table1 - meanTable;
    cv::Mat Q2 = robot1 - meanRobot;
    cv::Mat Q1T = Q1.reshape(1).t();
    cv::Mat Q2T = Q2.reshape(1).t();
    cv::Mat Q2TT = Q2T.reshape(1).t();
    std::cout << "Q1" << Q1 << std::endl;
    std::cout << "Q2" << Q2 << std::endl;
    std::cout << "Q1T" << Q1T << std::endl;
    std::cout << "Q2T" << Q2T << std::endl;
    std::cout << "Q2TT" << Q2TT << std::endl;

    //Calculates homography matrix
    cv::Mat H = Q1T * Q2TT;
    std::cout << H << std::endl;

    //Calculates SVD:
    cv::Mat W, U, VT;
    cv::SVD::compute(H, W, U, VT);
    /*
    std::cout << "W" << W << std::endl;
    std::cout << "U" << U << std::endl;
    std::cout << "VT" << VT << std::endl;
*/

    //Calculates Rotation matrix and Translation vector:
    cv::Mat R, T;
    R = U * VT;
    std::cout << "R" << R << std::endl;
    T = robotMean - (R * tableMean);
    std::cout << "T" << T << std::endl;

    //Generates the homogenious transformation matrix:
    cv::Mat finished = cv::Mat_<float>(4,4);
    finished.at<float>(0,0) = R.at<float>(0,0);
    finished.at<float>(0,1) = R.at<float>(0,1);
    finished.at<float>(0,2) = R.at<float>(0,2);
    finished.at<float>(0,3) = T.at<float>(0,0);
    finished.at<float>(1,0) = R.at<float>(1,0);
    finished.at<float>(1,1) = R.at<float>(1,1);
    finished.at<float>(1,2) = R.at<float>(1,2);
    finished.at<float>(1,3) = T.at<float>(1,0);
    finished.at<float>(2,0) = R.at<float>(2,0);
    finished.at<float>(2,1) = R.at<float>(2,1);
    finished.at<float>(2,2) = R.at<float>(2,2);
    finished.at<float>(2,3) = T.at<float>(2,0);
    finished.at<float>(3,0) = 0;
    finished.at<float>(3,1) = 0;
    finished.at<float>(3,2) = 0;
    finished.at<float>(3,3) = 1;
    std::cout << finished << std::endl;

    cv::Mat test = cv::Mat_<float>(1,4);
    test.at<float>(0,0) = A.x;
    test.at<float>(0,1) = A.y;
    test.at<float>(0,2) = A.z;
    test.at<float>(0,3) = 1;

    cv::Mat test2 = cv::Mat_<float>(1,4);
    test2.at<float>(0,0) = D.x;
    test2.at<float>(0,1) = D.y;
    test2.at<float>(0,2) = D.z;
    test2.at<float>(0,3) = 1;
    cv::FileStorage file("R2T2.xml", cv::FileStorage::WRITE);
    file << "R2T" << finished;

    std::cout << "Test: " << test << std::endl;
    std::cout << "Tjek: " <<  finished * test.reshape(1).t() << std::endl;
    std::cout << "Test: " << test2 << std::endl;
    std::cout << "Tjek: " <<  finished * test2.reshape(1).t() << std::endl;
    return 0;
}
