#include "database.h"

Database::Database()
{
    db = QSqlDatabase::addDatabase("QMYSQL");
    db.setHostName("localhost");
    db.setDatabaseName("semesterprojekt");
    db.setUserName("root");
    db.setPassword("1234");
    if (db.open()){
        std::cout << "Database loaded" << std::endl;
    }
    else{
        std::cout << "Database could not load" << std::endl;
    }
}

void Database::create_tables() {


    QSqlQuery query;

    std::cout << "Genererer tables vent venligst..." << std::endl;

    query.prepare("create table if not exists kast_data(kastid int not null auto_increment,\
                  lenght double(100, 4) not null, cartesianAcceleration double(100, 4) not null,\
                  maxJointAcc double(100, 4) not null, speed double(100, 10) not null, time double(100, 5) not null, \
                  ramt bool not null, angle double not null, primary key(kastid));");

    bool success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Kast_data table genereret..." << std::endl;

    query.prepare("create table if not exists kast_kop(kopid int not null auto_increment,\
                  positionx double(100, 4) not null, positiony double(100, 4) not null, primary key (kopid));");

    success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Kast_Kop table genereret..." << std::endl;

    query.prepare("create table if not exists kast_bold(boldid int not null auto_increment,\
                  positionx double(100, 4) not null, positiony double(100, 4) not null, primary key (boldid));");

    success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Kast_Bold table genererert..." << std::endl;

    query.prepare("create table if not exists joint_nulpunkt(nulpunktid int not null auto_increment, \
                  j1 double(100,5) not null, j2 double(100,5) not null, j3 double(100,5) not null,\
                  j4 double(100,5) not null, j5 double(100,5) not null, j6 double(100,5) not null,\
                  primary key(nulpunktid));");

    success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Joint_Nulpunkt table genereret..." << std::endl;

    query.prepare("create table if not exists joint_slut(slutid int not null auto_increment, \
                  j1 double(100,5) not null, j2 double(100,5) not null, j3 double(100,5) not null,\
                  j4 double(100,5) not null, j5 double(100,5) not null, j6 double(100,5) not null,\
                  primary key(slutid));");

    success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Joint_Slut table genereret..." << std::endl;

    std::cout << "Alle tables genereret! Opdaterer key connections..." << std::endl;

    //query.exec("ALTER TABLE `semesterprojekt`.`joint_nulpunkt` \
    ADD CONSTRAINT `nulpunkt_kastid` \
            FOREIGN KEY (`nulpunktid`)\
            REFERENCES `semesterprojekt`.`kast_data` (`kastid`);\
    \
    ALTER TABLE `semesterprojekt`.`joint_slut` \
            ADD CONSTRAINT `slut_kastid`\
            FOREIGN KEY (`slutid`)\
            REFERENCES `semesterprojekt`.`kast_data` (`kastid`);\
    \
    ALTER TABLE `semesterprojekt`.`kast_bold` \
            ADD CONSTRAINT `bold_kastid`\
            FOREIGN KEY (`boldid`)\
            REFERENCES `semesterprojekt`.`kast_data` (`kastid`);\
    \
    ALTER TABLE `semesterprojekt`.`kast_kop` \
            ADD CONSTRAINT `kop_kastid`\
            FOREIGN KEY (`kopid`)\
            REFERENCES `semesterprojekt`.`kast_data` (`kastid`);\
    \
    ALTER TABLE `semesterprojekt`.`kast_data` \
            ADD CONSTRAINT `kd_nulpunktid`\
            FOREIGN KEY (`kastid`)\
            REFERENCES `semesterprojekt`.`joint_nulpunkt` (`nulpunktid`),\
            ADD CONSTRAINT `kd_slutid`\
            FOREIGN KEY (`kastid`)\
            REFERENCES `semesterprojekt`.`joint_slut` (`slutid`),\
            ADD CONSTRAINT `kd_boldid`\
            FOREIGN KEY (`kastid`)\
            REFERENCES `semesterprojekt`.`kast_bold` (`boldid`),\
            ADD CONSTRAINT `kd_kopid`\
            FOREIGN KEY (`kastid`)\
            REFERENCES `semesterprojekt`.`kast_kop` (`kopid`);");


    std::cout << "Key connections opdateret.. DONE!" << std::endl;

}

void Database::drop_tables() {
    QSqlQuery query;
    std::string svar;

    std::cout << "Er du sikker på du vil slette alle tables? (Y/N)" << std::endl;
    std::cin >> svar;

    if(svar == "y" or svar == "yes" or svar == "Y") {
        std::cout << "Sletter alle tables vent venligst..." << std::endl;

        query.exec("ALTER TABLE joint_nulpunkt DROP CONSTRAINT nulpunkt_kastid; \
                   ALTER TABLE joint_slut DROP CONSTRAINT slut_kastid;\
                ALTER TABLE kast_bold DROP CONSTRAINT bold_kastid;\
        ALTER TABLE kast_kop DROP CONSTRAINT kop_kastid;\
        ALTER TABLE kast_data DROP CONSTRAINT kd_boldid;\
        ALTER TABLE kast_data DROP CONSTRAINT kd_kopid;\
        ALTER TABLE kast_data DROP CONSTRAINT kd_slutid;\
        ALTER TABLE kast_data DROP CONSTRAINT kd_nulpunktid;");


        query.prepare("Drop table joint_nulpunkt");
        query.exec();
        query.prepare("Drop table joint_slut");
        query.exec();
        query.prepare("Drop table kast_bold");
        query.exec();
        query.prepare("Drop table kast_kop");
        query.exec();
        query.prepare("Drop table kast_data");
        query.exec();

        std::cout << "Alle tables slettet!" << std::endl;
    }
    else std::cout << "Afbryder sletning..." << std::endl;
}

void Database::sync_data()
{

}

void Database::disconnect() {
    std::string svar;
    std::cout << "Vil du afbryde forbindelsen? (Y/N)" << std::endl;
    std::cin >> svar;

    if(svar == "yes" or svar == "y" or svar == "Y") {
        db.close();
    }
    else std::cout << "fortsætter forbindelsen.." << std::endl;
}

void Database::connect() {
    std::cout << "Opretter forbindelse..." << std::endl;
    db.setHostName("localhost");
    db.setDatabaseName("semesterprojekt");
    db.setUserName("root");
    db.setPassword("1234");
    if(db.open()) {
        std::cout << "Forbindelse oprettet!" << std::endl;
    }
    else std::cout << "Fejl.. forbindelse ikke oprettet" << std::endl;
}

void Database::add_boldposition(std::vector<double> robotBallCoordinates)
{
    QSqlQuery query;

    double x_coord = robotBallCoordinates.at(0); //std::string sx_coord = x_coord.toStdString();
    double y_coord = robotBallCoordinates.at(1); //std::string sy_coord = y_coord.toStdString();

    std::cout << "Bold data opdateret" << std::endl;

    query.prepare("INSERT INTO kast_bold (positionx, positiony) "
                  "VALUES(:positionx, :positiony);");
    query.bindValue(":positionx", x_coord);
    query.bindValue(":positiony", y_coord);

    bool success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Boldposition indsat: " << "(X: " << x_coord << " | Y: " << y_coord << ")" << std::endl;
}

void Database::add_kopposition(std::vector<double> robotCupCoordinates)
{
    QSqlQuery query;

    double x_coord = robotCupCoordinates.at(0);
    double y_coord = robotCupCoordinates.at(1);

    std::cout << "Kop data opdateret" << std::endl;

    query.prepare("INSERT INTO kast_kop (positionx, positiony) "
                  "VALUES(:positionx, :positiony);");
    query.bindValue(":positionx", x_coord);
    query.bindValue(":positiony", y_coord);

    bool success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Kopdata indsat: " << "(X: " << x_coord << " | Y: " << y_coord << ")" << std::endl;
}

void Database::add_joint_throw_values(std::vector<double> kastJointValues)
{
    QSqlQuery query;

    double j1 = kastJointValues.at(0);
    double j2 = kastJointValues.at(1);
    double j3 = kastJointValues.at(2);
    double j4 = kastJointValues.at(3);
    double j5 = kastJointValues.at(4);
    double j6 = kastJointValues.at(5);

    std::cout << "Joint data opdateret" << std::endl;

    query.prepare("INSERT INTO joint_slut (j1, j2, j3, j4, j5, j6) "
                  "VALUES(:j1, :j2, :j3, :j4, :j5, :j6);");

    query.bindValue(":j1", j1);
    query.bindValue(":j2", j2);
    query.bindValue(":j3", j3);
    query.bindValue(":j4", j4);
    query.bindValue(":j5", j5);
    query.bindValue(":j6", j6);

    bool success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Joint slutpunkt data indsat: " << "(J1: " << j1 << " | J2: " << j2
              << " | J3: " << j3 << " | J4: " << j4
              << " | J5: " << j5 << " | J6: " << j6 << ")" << std::endl;
}

void Database::add_kast_data(std::vector<double> robotCupCoordinates, std::vector<double> robotThrowPos, double time, bool hit, double angle, double maxJointAcc, double speed) //Skal vi lige finde ud af
{
    QSqlQuery query;

    double lenght = 0;
    for(int i = 0; i < 3; ++i){
        lenght += (robotCupCoordinates.at(i) - robotThrowPos.at(i)) * (robotCupCoordinates.at(i) - robotThrowPos.at(i));
    }
    lenght = std::sqrt(lenght);

    double cartesianAcceleration = lenght/time;

    query.prepare("INSERT INTO kast_data (lenght, cartesianAcceleration, maxJointAcc, speed, time, ramt, angle) "
                  "VALUES(:lenght, :cartesianAcceleration, :maxJointAcc, :speed, :time, :ramt, :angle);");

    query.bindValue(":lenght", lenght);
    query.bindValue(":cartesianAcceleration", cartesianAcceleration);
    query.bindValue(":maxJointAcc", maxJointAcc);
    query.bindValue(":speed", speed);
    query.bindValue(":time", time);
    query.bindValue(":ramt", hit);
    query.bindValue(":angle", angle);

    bool success = false;
    success = query.exec();
    if (!success) {
        qDebug() << query.lastError();
        db.rollback();
    }

    std::cout << "Kast data indsat: " << "(Lenght: " << lenght << " | acceleration: " << cartesianAcceleration
              << " | speed: " << speed << " | time: " << time
              << " | ramt: " << hit << " | maxJointAcc: " << maxJointAcc
              << " | angle: " << angle << ")" << std::endl;
}
