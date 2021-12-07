#ifndef DATABASE_H
#define DATABASE_H

#include <QCoreApplication>
#include <QSqlDatabase>
#include <iostream>
#include <QDebug>
#include <QtSql>
#include <QString>
#include <sstream>

class Database
{
private:


public:
    QSqlDatabase db;
    QString x_coord;
    QString y_coord;
    Database();

    std::vector<std::vector<double>> add_boldposition();
    std::vector<std::vector<double>> add_kopposition();
    std::vector<std::vector<double>> add_joint_nulpunkt();
    std::vector<std::vector<double>> add_joint_slut();
    std::vector<std::vector<double>> add_kast_data();
    void sync_data();
    void create_tables();
    void drop_tables();
    void disconnect();
    void connect();
};

#endif // DATABASE_H
