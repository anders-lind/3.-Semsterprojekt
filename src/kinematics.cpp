#include "kinematics.h"
#include "throw.h"

using namespace std;
using namespace ur_rtde;

Kinematics::Kinematics()
{
        // vector<double> kastpositionDeg = {99, -90, 106, -124, -85, -101};
        // vector<double> maalpositionDeg = {100, -70, 101, -136, -87, -100};

        // // Omregn til radianer
        // vector<double> kastpositionRad = kastpositionDeg;
        // for (int i = 0; i < 6; i++)
        //     kastpositionRad.at(i) = kastpositionDeg.at(i) * (2*3.14159)/360;
        // vector<double> maalpositionRad = maalpositionDeg;
        // for (int i = 0; i < 6; i++)
        //     maalpositionRad.at(i) = maalpositionDeg.at(i) * (2*3.14159)/360;


        // Eigen::Vector3d xk(0.4674, -0.1943, -0.043);
        // Eigen::Vector3d xm(0.218, -0.566, 0.228);

        // // Get joint values for throw position
        // Eigen::MatrixXd q_k(6, 1);


        // q_k << kastpositionRad.at(0), kastpositionRad.at(1),
        //     kastpositionRad.at(2), kastpositionRad.at(3),
        //     kastpositionRad.at(4), kastpositionRad.at(5);


        // Eigen::MatrixXd xp_k = calc_xp_k(xk, xm);
        // Eigen::MatrixXd qp_k = calc_qp_k(q_k, xp_k);
        // Eigen::MatrixXd acc = calc_acc(qp_k, _t);
        // double maxAcc = calc_max_acc(acc);
}


Eigen::MatrixXd Kinematics::calc_xp_k(Eigen::MatrixXd xk, Eigen::MatrixXd xm, double angle)
{
    Throw kast;

    Eigen::MatrixXd tmp = kast.calculate3DSpeed(xk, xm, angle);
    Eigen::MatrixXd xp_k(6, 1);
    xp_k(0) = tmp(0);
    xp_k(1) = tmp(1);
    xp_k(2) = tmp(2);
    xp_k(3) = 0;
    xp_k(4) = 0;
    xp_k(5) = 0;
    _xp_k = xp_k;
    return xp_k;
}


Eigen::MatrixXd Kinematics::calc_qp_k(Eigen::MatrixXd q_k, Eigen::MatrixXd xp_k)
{
    // Calculate joint-speed from Jacobian and throw-speed
    jacobian Jac(q_k);
    Eigen::MatrixXd J = Jac.get();
    Eigen::MatrixXd qp_k = J.inverse() * xp_k;
    cout << "qp_k = Jac(q_k).inverse() * xp_k" << endl;
    cout << "q_k = \n" << q_k << endl;
    cout << "qp_k = " << qp_k << "\n = \n" << J.inverse() << "\n * \n" << xp_k << endl;
    //qp_k(0) = 0;

    // Opstil grafer for q og qp for hvert led
    // qp(t) = f(t) = at + b                ->      b = 0       a = (qp(t) - b) / t
    //  q(t) = F(t) = a/2*t^2 + bt + c      ->      c = q_s     a = 2(q(t) - bt - c) / t^2

    _qp_k = qp_k;
    return qp_k;
}


Eigen::MatrixXd Kinematics::calc_acc(Eigen::MatrixXd qp_k, double t)
{
    // Calculate acceleration matrix
    Eigen::MatrixXd a(6, 1);
    for (int i = 0; i < 6; i++)
    {
        a(i) = (qp_k(i) - 0) / t;
    }

    _acc = a;
    return a;
}


double Kinematics::calc_max_acc(Eigen::MatrixXd acc)
{
    double acceleration = 0;
    for (int i = 0; i < 6; i++)
    {
        if (acc(i) > acceleration)
        {
            acceleration = acc(i);
        }
        else if (acc(i) < acceleration)
        {
            double temp = acc(i) * -1;
            if (temp > acceleration)
                acceleration = temp;
        }
    }

    return acceleration;
}



// void Kinematics::printValues()
// {
//     // Print calculation values
//     cout << "---------- Values used in calculations ----------" << endl;
//     // xm (Maalposition)
//     cout << left << setw(25) << "Maalposition: " << "xm = [";
//     for (int i = 0; i < 3; i++)
//         cout << _xm(i) << ", ";
//     cout << "]" << endl;

//         // xk (Kasteposition)
//     cout << left << setw(25) << "Kasteposition: " << "xk = [";
//     for (int i = 0; i < 3; i++)
//         cout << _xk(i) << ", ";
//     cout << "]" << endl;

//     // xp (Throws speed)
//     cout << left << setw(25) << "Throw speed: " << "xp_k = [";
//     for (int i = 0; i < 6; i++)
//         cout << _xp_k(i) << ", ";
//     cout << "]" << endl;

//     // q_s (start position)
//     // cout << left << setw(25) << "Start position: " << "q_s = [";
//     // for (int i = 0; i < 6; i++){
//     //     cout << q_s(i) << ", ";
//     // } cout << "]" << endl;
//     // cout << left << setw(25) << "Start position: " << "q_sDEG = [";
//     // for (int i = 0; i < 6; i++){
//     //     cout << q_sDEG.at(i) << ", ";
//     // } cout << "]" << endl;

//     // q_k (throw position)
//     cout << left << setw(25) << "Throw position: " << "q_k = [";
//     for (int i = 0; i < 6; i++){
//         cout << _q_k(i) << ", ";
//     } cout << "]" << endl;


//     // qp_k (JointSpeed kast)
//     cout << left << setw(25) << "JointSpeed kast: " << "qp_k = [";
//     for (int i = 0; i < 6; i++){
//         cout << _qp_k(i) << ", ";
//         } cout << "]" << endl;

//     // qpp (Joint acceleration)
//     cout << left << setw(25) << "Joint acceleration: " << "qpp = [";
//     for (int i = 0; i < 6; i++){
//         cout << _acc(i) << ", ";
//     } cout << "] " << "Highest acceleration = " << acceleration << endl;

//     // t Time
//     cout << left << setw(25) << "Time : " << "t = " << t << endl;

// }



/*
int main()
{
    // For local host: "127.0.0.1"
    string ip = "127.0.0.1";
    Movement m;
    vector<double> kastpositionDeg = {99, -90, 106, -124, -85, -101};
    vector<double> udgangspositionDeg = {130, -110, 125, -130, -62, -72};
    vector<double> maalpositionDeg = {100, -70, 101, -136, -87, -100};
    try
    {
        cout << boolalpha;
        cout << "UR_RTDE: Attempting connection to robot socket at " << ip << " " << endl;
        // Connecting to robot socket
        RTDEControlInterface rtde_control(ip);
        RTDEReceiveInterface rtde_receive(ip);
        cout << "UR_RTDE: Connected to robot socket at " << ip << "\n"
             << endl;
        // --------------- Kinematik udregninger ---------------
        // Get throw position xk and target position xm
        Eigen::Vector3d xk(0.4674, -0.1943, -0.043);
        Eigen::Vector3d xm(0.218, -0.566, 0.228);
        // Calculate throw speed
        Throw kast;
        Eigen::Vector3d tmp = kast.calculate3DSpeed(xk, xm);
        Eigen::MatrixXd xp_k(6, 1);
        xp_k(0) = tmp(0);
        xp_k(1) = tmp(1);
        xp_k(2) = tmp(2);
        xp_k(3) = 0;
        xp_k(4) = 0;
        xp_k(5) = 0;
        // Get joint values for start positionen
        // Eigen::MatrixXd q_s(6, 1);
        // vector<double> udgangspositionRad = udgangspositionDeg;
        // m.radConversion(udgangspositionRad);
        // q_s << udgangspositionRad.at(0), udgangspositionRad.at(1),
        //     udgangspositionRad.at(2), udgangspositionRad.at(3),
        //     udgangspositionRad.at(4), udgangspositionRad.at(5);
        // Get joint values for throw positionen
        Eigen::MatrixXd q_k(6, 1);
        vector<double> kastpositionRad = kastpositionDeg;
        m.radConversion(kastpositionRad);
        q_k << kastpositionRad.at(0), kastpositionRad.at(1),
            kastpositionRad.at(2), kastpositionRad.at(3),
            kastpositionRad.at(4), kastpositionRad.at(5);
        // Calculate joint-speed from Jacobian and throw-speed
        Jacobian Jac(q_k);
        Eigen::MatrixXd J = Jac.get();
        Eigen::MatrixXd qp_k = J.inverse() * xp_k;
        qp_k(0) = 0;
        // Opstil grafer for q og qp for hvert led
        // qp(t) = f(t) = at + b                ->      b = 0       a = (qp(t) - b) / t
        //  q(t) = F(t) = a/2*t^2 + bt + c      ->      c = q_s     a = 2(q(t) - bt - c) / t^2
        // Vælg kast-tiden
        double t = 0.1;
        // Calculate acceleration matrix
        Eigen::MatrixXd a(6, 1);
        for (int i = 0; i < 6; i++)
        {
            a(i) = (qp_k(i) - 0) / t;
        }
        double acceleration = 0;
        for (int i = 0; i < 6; i++)
        {
            if (a(i) > acceleration)
            {
                acceleration = a(i);
            }
            else if (a(i) < acceleration)
            {
                double temp = a(i) * -1;
                if (temp > acceleration)
                    acceleration = temp;
            }
        }
        // Calculate start position from throw joint speed
        // Eigen::MatrixXd q_s(6, 1);
        // for (int i = 0; i < 6; i++){
        //     q_s(i) = q_k(i) - qp_k(i)*t/2;                 // startpos = kastpos - throwJSPeed*t   =>  q_s = q_k-qp_k*t/2           <-- /s er lidt sus
        // }
        // vector<double> q_sDEG(6);
        // for (int i = 0; i < 6; i++){
        //     q_sDEG.at(i) = q_s(i) * 360/(2*3.1415);
        // }
        std::vector<double> qpp = Jacobian::eig2Vec(a);
        std::vector<double> joint_speed = Jacobian::eig2Vec(qp_k);
        // Print calculation values
        cout << "---------- Values used in calculations ----------" << endl;
        // xm (Maalposition)
        cout << left << setw(25) << "Maalposition: " << "xm = [";
        for (int i = 0; i < 3; i++)
            cout << xm(i) << ", ";
        cout << "]" << endl;
         // xk (Kasteposition)
        cout << left << setw(25) << "Kasteposition: " << "xk = [";
        for (int i = 0; i < 3; i++)
            cout << xk(i) << ", ";
        cout << "]" << endl;
        // xp (Throws speed)
        cout << left << setw(25) << "Throw speed: " << "xp_k = [";
        for (int i = 0; i < 6; i++)
            cout << xp_k(i) << ", ";
        cout << "]" << endl;
        // q_s (start position)
        // cout << left << setw(25) << "Start position: " << "q_s = [";
        // for (int i = 0; i < 6; i++){
        //     cout << q_s(i) << ", ";
        // } cout << "]" << endl;
        // cout << left << setw(25) << "Start position: " << "q_sDEG = [";
        // for (int i = 0; i < 6; i++){
        //     cout << q_sDEG.at(i) << ", ";
        // } cout << "]" << endl;
        // q_k (throw position)
        cout << left << setw(25) << "Throw position: " << "q_k = [";
        for (int i = 0; i < 6; i++){
            cout << q_k(i) << ", ";
        } cout << "]" << endl;
        // qp_k (JointSpeed kast)
        cout << left << setw(25) << "JointSpeed kast: " << "qp_k = [";
        for (int i = 0; i < 6; i++){
            cout << qp_k(i) << ", ";
            } cout << "]" << endl;
        // qpp (Joint acceleration)
        cout << left << setw(25) << "Joint acceleration: " << "qpp = [";
        for (int i = 0; i < 6; i++){
            cout << a(i) << ", ";
        } cout << "] " << "Highest acceleration = " << acceleration << endl;
        // Time
        cout << left << setw(25) << "Time : " << "t = " << t << endl;
        // Print robot values
        cout << "---------- Values of robot ----------" << endl;
        cout << left << setw(25) << "Start-position: " << "x_s = [";
        for (int i = 0; i < 6; i++){
            cout << rtde_receive.getActualTCPPose().at(i) << ", ";
        } cout << "]" << endl;
        cout << left << setw(25) << "Start-position: " << "q_s = [";
        for (int i = 0; i < 6; i++){
            cout << rtde_receive.getTargetQ().at(i) << ", ";
        } cout << endl;
        // --------------- Robot bevægelse ---------------
        // Maalposition
        m.moveJoints(rtde_control, maalpositionDeg);
        cout << "Maalpositionen!" << endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(2));
        // Kastposition
        m.moveJoints(rtde_control, kastpositionDeg);
        cout << "Kastepositionen!" << endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(2));
        // Udgangsposition
        vector<double> negSpeed(6);
        for (int i = 0; i < 6; i++)
            negSpeed.at(i) = joint_speed.at(i)*(-1);
        rtde_control.speedJ(negSpeed, acceleration, t);
        std::this_thread::sleep_for(std::chrono::duration<double>(t));
        rtde_control.speedStop();
        cout << "Udgangsposition!" << endl;
        vector<double> ud(6);
        ud = rtde_receive.getTargetQ();
        cout << "udgangspos = ";
        for (int i = 0; i < 6; i++){
            cout << ud.at(i) << ", ";
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(2));
        //rtde_control.stopScript();
        // m.moveJoints(rtde_control, q_sDEG);
        // cout << "Udgangsposition!" << endl;
        // std::this_thread::sleep_for(std::chrono::duration<double>(2));
        // Kast
        // m.kast(rtde_control, rtde_receive);
        // Start speedJ
        std::cout << "Throw!" << std::endl;
        rtde_control.speedJ(joint_speed, acceleration, t);
        // Timing
        chrono::duration<double> dur;
        chrono::time_point<chrono::system_clock> startTime;
        startTime = chrono::system_clock::now();
        do  {
            dur = chrono::system_clock::now() - startTime;
        }
        while(dur.count() < t);
        // Timing
        cout << left << setw(25) << "Throw position: " << "q_k = [";
        for (int i = 0; i < 6; i++){
            cout << rtde_receive.getTargetQ().at(i) << ", ";
        } cout << "]" << endl;
        // Stop speedJ
        rtde_control.speedStop();
        //rtde_control.stopScript();
        // --------------- Print robot values ---------------
        cout << left << setw(25) << "Throw position: " << "q_k = [";
        for (int i = 0; i < 6; i++){
            cout << rtde_receive.getTargetQ().at(i) << ", ";
        } cout << "]" << endl;
        // q_k (Throw position)
        cout << left << setw(25) << "Expected end position: " << "q_k = [";
        for (int i = 0; i < 6; i++){
            cout << q_k(i) << ", ";
            } cout << "]" << endl;
    }   catch (const runtime_error &error)
    {
        cout << "UR_RTDE: Failed connecting to robot socket at " << ip << "\n"
             << endl;
        cout << "System : Exiting" << endl;
    }
    return 0;
}
*/

/*
#include "kinematics.h"
#include "throw.h"
using namespace std;
using namespace ur_rtde;
Kinematics::Kinematics()
{
        vector<double> kastpositionDeg = {99, -90, 106, -124, -85, -101};
        vector<double> maalpositionDeg = {100, -70, 101, -136, -87, -100};
        // Omregn til radianer
        vector<double> kastpositionRad = kastpositionDeg;
        for (int i = 0; i < 6; i++)
            kastpositionRad.at(i) = kastpositionDeg.at(i) * (2*3.14159)/360;
        vector<double> maalpositionRad = maalpositionDeg;
        for (int i = 0; i < 6; i++)
            maalpositionRad.at(i) = maalpositionDeg.at(i) * (2*3.14159)/360;
        Eigen::Vector3d xk(0.4674, -0.1943, -0.043);
        Eigen::Vector3d xm(0.218, -0.566, 0.228);
        // Get joint values for throw position
        Eigen::MatrixXd q_k(6, 1);
        q_k << kastpositionRad.at(0), kastpositionRad.at(1),
            kastpositionRad.at(2), kastpositionRad.at(3),
            kastpositionRad.at(4), kastpositionRad.at(5);
        Eigen::MatrixXd xp_k = calc_xp_k(xk, xm);
        Eigen::MatrixXd qp_k = calc_qp_k(q_k, xp_k);
        Eigen::MatrixXd acc = calc_acc(qp_k, _t);
        double maxAcc = calc_max_acc(acc);
}
Eigen::MatrixXd Kinematics::calc_xp_k(Eigen::MatrixXd xk, Eigen::MatrixXd xm)
{
    Throw kast;
    Eigen::Vector3d tmp = kast.calculate3DSpeed(xk, xm);
    Eigen::MatrixXd xp_k(6, 1);
    xp_k(0) = tmp(0);
    xp_k(1) = tmp(1);
    xp_k(2) = tmp(2);
    xp_k(3) = 0;
    xp_k(4) = 0;
    xp_k(5) = 0;
    _xp_k = xp_k;
    return xp_k;
}
Eigen::MatrixXd Kinematics::calc_qp_k(Eigen::MatrixXd q_k, Eigen::MatrixXd xp_k)
{
    // Calculate joint-speed from Jacobian and throw-speed
    jacobian Jac(q_k);
    Eigen::MatrixXd J = Jac.get();
    Eigen::MatrixXd qp_k = J.inverse() * xp_k;
    qp_k(0) = 0;
    // Opstil grafer for q og qp for hvert led
    // qp(t) = f(t) = at + b                ->      b = 0       a = (qp(t) - b) / t
    //  q(t) = F(t) = a/2*t^2 + bt + c      ->      c = q_s     a = 2(q(t) - bt - c) / t^2
    _qp_k = qp_k;
    return qp_k;
}
Eigen::MatrixXd Kinematics::calc_acc(Eigen::MatrixXd qp_k, double t)
{
    // Calculate acceleration matrix
    Eigen::MatrixXd a(6, 1);
    for (int i = 0; i < 6; i++)
    {
        a(i) = (qp_k(i) - 0) / t;
    }
    _acc = a;
    return a;
}
double Kinematics::calc_max_acc(Eigen::MatrixXd acc)
{
    double acceleration = 0;
    for (int i = 0; i < 6; i++)
    {
        if (acc(i) > acceleration)
        {
            acceleration = acc(i);
        }
        else if (acc(i) < acceleration)
        {
            double temp = acc(i) * -1;
            if (temp > acceleration)
                acceleration = temp;
        }
    }
    return acceleration;
}
// void Kinematics::printValues()
// {
//     // Print calculation values
//     cout << "---------- Values used in calculations ----------" << endl;
//     // xm (Maalposition)
//     cout << left << setw(25) << "Maalposition: " << "xm = [";
//     for (int i = 0; i < 3; i++)
//         cout << _xm(i) << ", ";
//     cout << "]" << endl;
//         // xk (Kasteposition)
//     cout << left << setw(25) << "Kasteposition: " << "xk = [";
//     for (int i = 0; i < 3; i++)
//         cout << _xk(i) << ", ";
//     cout << "]" << endl;
//     // xp (Throws speed)
//     cout << left << setw(25) << "Throw speed: " << "xp_k = [";
//     for (int i = 0; i < 6; i++)
//         cout << _xp_k(i) << ", ";
//     cout << "]" << endl;
//     // q_s (start position)
//     // cout << left << setw(25) << "Start position: " << "q_s = [";
//     // for (int i = 0; i < 6; i++){
//     //     cout << q_s(i) << ", ";
//     // } cout << "]" << endl;
//     // cout << left << setw(25) << "Start position: " << "q_sDEG = [";
//     // for (int i = 0; i < 6; i++){
//     //     cout << q_sDEG.at(i) << ", ";
//     // } cout << "]" << endl;
//     // q_k (throw position)
//     cout << left << setw(25) << "Throw position: " << "q_k = [";
//     for (int i = 0; i < 6; i++){
//         cout << _q_k(i) << ", ";
//     } cout << "]" << endl;
//     // qp_k (JointSpeed kast)
//     cout << left << setw(25) << "JointSpeed kast: " << "qp_k = [";
//     for (int i = 0; i < 6; i++){
//         cout << _qp_k(i) << ", ";
//         } cout << "]" << endl;
//     // qpp (Joint acceleration)
//     cout << left << setw(25) << "Joint acceleration: " << "qpp = [";
//     for (int i = 0; i < 6; i++){
//         cout << _acc(i) << ", ";
//     } cout << "] " << "Highest acceleration = " << acceleration << endl;
//     // t Time
//     cout << left << setw(25) << "Time : " << "t = " << t << endl;
// }
/*
int main()
{
    // For local host: "127.0.0.1"
    string ip = "127.0.0.1";
    Movement m;
    vector<double> kastpositionDeg = {99, -90, 106, -124, -85, -101};
    vector<double> udgangspositionDeg = {130, -110, 125, -130, -62, -72};
    vector<double> maalpositionDeg = {100, -70, 101, -136, -87, -100};
    try
    {
        cout << boolalpha;
        cout << "UR_RTDE: Attempting connection to robot socket at " << ip << " " << endl;
        // Connecting to robot socket
        RTDEControlInterface rtde_control(ip);
        RTDEReceiveInterface rtde_receive(ip);
        cout << "UR_RTDE: Connected to robot socket at " << ip << "\n"
             << endl;
        // --------------- Kinematik udregninger ---------------
        // Get throw position xk and target position xm
        Eigen::Vector3d xk(0.4674, -0.1943, -0.043);
        Eigen::Vector3d xm(0.218, -0.566, 0.228);
        // Calculate throw speed
        Throw kast;
        Eigen::Vector3d tmp = kast.calculate3DSpeed(xk, xm);
        Eigen::MatrixXd xp_k(6, 1);
        xp_k(0) = tmp(0);
        xp_k(1) = tmp(1);
        xp_k(2) = tmp(2);
        xp_k(3) = 0;
        xp_k(4) = 0;
        xp_k(5) = 0;
        // Get joint values for start positionen
        // Eigen::MatrixXd q_s(6, 1);
        // vector<double> udgangspositionRad = udgangspositionDeg;
        // m.radConversion(udgangspositionRad);
        // q_s << udgangspositionRad.at(0), udgangspositionRad.at(1),
        //     udgangspositionRad.at(2), udgangspositionRad.at(3),
        //     udgangspositionRad.at(4), udgangspositionRad.at(5);
        // Get joint values for throw positionen
        Eigen::MatrixXd q_k(6, 1);
        vector<double> kastpositionRad = kastpositionDeg;
        m.radConversion(kastpositionRad);
        q_k << kastpositionRad.at(0), kastpositionRad.at(1),
            kastpositionRad.at(2), kastpositionRad.at(3),
            kastpositionRad.at(4), kastpositionRad.at(5);
        // Calculate joint-speed from Jacobian and throw-speed
        Jacobian Jac(q_k);
        Eigen::MatrixXd J = Jac.get();
        Eigen::MatrixXd qp_k = J.inverse() * xp_k;
        qp_k(0) = 0;
        // Opstil grafer for q og qp for hvert led
        // qp(t) = f(t) = at + b                ->      b = 0       a = (qp(t) - b) / t
        //  q(t) = F(t) = a/2*t^2 + bt + c      ->      c = q_s     a = 2(q(t) - bt - c) / t^2
        // Vælg kast-tiden
        double t = 0.1;
        // Calculate acceleration matrix
        Eigen::MatrixXd a(6, 1);
        for (int i = 0; i < 6; i++)
        {
            a(i) = (qp_k(i) - 0) / t;
        }
        double acceleration = 0;
        for (int i = 0; i < 6; i++)
        {
            if (a(i) > acceleration)
            {
                acceleration = a(i);
            }
            else if (a(i) < acceleration)
            {
                double temp = a(i) * -1;
                if (temp > acceleration)
                    acceleration = temp;
            }
        }
        // Calculate start position from throw joint speed
        // Eigen::MatrixXd q_s(6, 1);
        // for (int i = 0; i < 6; i++){
        //     q_s(i) = q_k(i) - qp_k(i)*t/2;                 // startpos = kastpos - throwJSPeed*t   =>  q_s = q_k-qp_k*t/2           <-- /s er lidt sus
        // }
        // vector<double> q_sDEG(6);
        // for (int i = 0; i < 6; i++){
        //     q_sDEG.at(i) = q_s(i) * 360/(2*3.1415);
        // }
        std::vector<double> qpp = Jacobian::eig2Vec(a);
        std::vector<double> joint_speed = Jacobian::eig2Vec(qp_k);
        // Print calculation values
        cout << "---------- Values used in calculations ----------" << endl;
        // xm (Maalposition)
        cout << left << setw(25) << "Maalposition: " << "xm = [";
        for (int i = 0; i < 3; i++)
            cout << xm(i) << ", ";
        cout << "]" << endl;
         // xk (Kasteposition)
        cout << left << setw(25) << "Kasteposition: " << "xk = [";
        for (int i = 0; i < 3; i++)
            cout << xk(i) << ", ";
        cout << "]" << endl;
        // xp (Throws speed)
        cout << left << setw(25) << "Throw speed: " << "xp_k = [";
        for (int i = 0; i < 6; i++)
            cout << xp_k(i) << ", ";
        cout << "]" << endl;
        // q_s (start position)
        // cout << left << setw(25) << "Start position: " << "q_s = [";
        // for (int i = 0; i < 6; i++){
        //     cout << q_s(i) << ", ";
        // } cout << "]" << endl;
        // cout << left << setw(25) << "Start position: " << "q_sDEG = [";
        // for (int i = 0; i < 6; i++){
        //     cout << q_sDEG.at(i) << ", ";
        // } cout << "]" << endl;
        // q_k (throw position)
        cout << left << setw(25) << "Throw position: " << "q_k = [";
        for (int i = 0; i < 6; i++){
            cout << q_k(i) << ", ";
        } cout << "]" << endl;
        // qp_k (JointSpeed kast)
        cout << left << setw(25) << "JointSpeed kast: " << "qp_k = [";
        for (int i = 0; i < 6; i++){
            cout << qp_k(i) << ", ";
            } cout << "]" << endl;
        // qpp (Joint acceleration)
        cout << left << setw(25) << "Joint acceleration: " << "qpp = [";
        for (int i = 0; i < 6; i++){
            cout << a(i) << ", ";
        } cout << "] " << "Highest acceleration = " << acceleration << endl;
        // Time
        cout << left << setw(25) << "Time : " << "t = " << t << endl;
        // Print robot values
        cout << "---------- Values of robot ----------" << endl;
        cout << left << setw(25) << "Start-position: " << "x_s = [";
        for (int i = 0; i < 6; i++){
            cout << rtde_receive.getActualTCPPose().at(i) << ", ";
        } cout << "]" << endl;
        cout << left << setw(25) << "Start-position: " << "q_s = [";
        for (int i = 0; i < 6; i++){
            cout << rtde_receive.getTargetQ().at(i) << ", ";
        } cout << endl;
        // --------------- Robot bevægelse ---------------
        // Maalposition
        m.moveJoints(rtde_control, maalpositionDeg);
        cout << "Maalpositionen!" << endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(2));
        // Kastposition
        m.moveJoints(rtde_control, kastpositionDeg);
        cout << "Kastepositionen!" << endl;
        std::this_thread::sleep_for(std::chrono::duration<double>(2));
        // Udgangsposition
        vector<double> negSpeed(6);
        for (int i = 0; i < 6; i++)
            negSpeed.at(i) = joint_speed.at(i)*(-1);
        rtde_control.speedJ(negSpeed, acceleration, t);
        std::this_thread::sleep_for(std::chrono::duration<double>(t));
        rtde_control.speedStop();
        cout << "Udgangsposition!" << endl;
        vector<double> ud(6);
        ud = rtde_receive.getTargetQ();
        cout << "udgangspos = ";
        for (int i = 0; i < 6; i++){
            cout << ud.at(i) << ", ";
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(2));
        //rtde_control.stopScript();
        // m.moveJoints(rtde_control, q_sDEG);
        // cout << "Udgangsposition!" << endl;
        // std::this_thread::sleep_for(std::chrono::duration<double>(2));
        // Kast
        // m.kast(rtde_control, rtde_receive);
        // Start speedJ
        std::cout << "Throw!" << std::endl;
        rtde_control.speedJ(joint_speed, acceleration, t);
        // Timing
        chrono::duration<double> dur;
        chrono::time_point<chrono::system_clock> startTime;
        startTime = chrono::system_clock::now();
        do  {
            dur = chrono::system_clock::now() - startTime;
        }
        while(dur.count() < t);
        // Timing
        cout << left << setw(25) << "Throw position: " << "q_k = [";
        for (int i = 0; i < 6; i++){
            cout << rtde_receive.getTargetQ().at(i) << ", ";
        } cout << "]" << endl;
        // Stop speedJ
        rtde_control.speedStop();
        //rtde_control.stopScript();
        // --------------- Print robot values ---------------
        cout << left << setw(25) << "Throw position: " << "q_k = [";
        for (int i = 0; i < 6; i++){
            cout << rtde_receive.getTargetQ().at(i) << ", ";
        } cout << "]" << endl;
        // q_k (Throw position)
        cout << left << setw(25) << "Expected end position: " << "q_k = [";
        for (int i = 0; i < 6; i++){
            cout << q_k(i) << ", ";
            } cout << "]" << endl;
    }   catch (const runtime_error &error)
    {
        cout << "UR_RTDE: Failed connecting to robot socket at " << ip << "\n"
             << endl;
        cout << "System : Exiting" << endl;
    }
    return 0;
}
*/
