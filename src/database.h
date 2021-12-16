#ifndef DATABASE_H
#define DATABASE_H

#include<QCoreApplication>
#include<QSqlDatabase>
#include<QtSql>
#include<QString>
#include<QDebug>
#include<sstream>
#include<iostream>
#include <vector>

class Database
{
public:
    Database();

    void add_boldposition(std::vector<double> robotBallCoordinates);
    void add_kopposition(std::vector<double> robotCupCoordinates);
    void add_joint_throw_values(std::vector<double> kastJointValues);
    void add_kast_data(std::vector<double> robotCupCoordinates, std::vector<double> robotThrowPos, double time, bool hit, double angle, double maxJointAcc, double speed);

    void sync_data();
    void create_tables();
    void drop_tables();
    void disconnect();
    void connect();

private:
    QSqlDatabase db;
    QString x_coord;
    QString y_coord;
};

#endif // DATABASE_H
