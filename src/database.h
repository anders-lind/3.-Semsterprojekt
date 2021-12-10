#ifndef DATABASE_H
#define DATABASE_H
#include<QCoreApplication>
#include<QSqlDatabase>
#include<QtSql>
#include<QString>
#include<sstream>
#include<iostream>
#include<QDebug>

class Database
{
public:
    Database();

    void add_boldposition(std::vector<double> robotBallCoordinates);
    void add_kopposition(std::vector<double> robotCupCoordinates);
    void add_joint_nulpunkt(std::vector<double> nulpunktJointValues);
    void add_joint_slut(std::vector<double> kastJointValues);
    void add_kast_data();

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
