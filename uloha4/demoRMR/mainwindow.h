#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <queue>
#include<windows.h>
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<vector>
//#include "ckobuki.h"
//#include "rplidar.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "robot.h"

#include <QJoysticks.h>

typedef struct
{
    int scanQuality;
    double scanAngle;
    double scanDistance;
}MojeLaserData;

typedef struct
{
    int x;
    int y;
}Index;

typedef struct
{
    queue<Index> unvisited;
    Index current;
}Alg;

typedef struct
{

    float x;
    float y;
    float angle;
    int translation;
    int numberOfScans;
    MojeLaserData Data[1000];
    bool stop;
    bool moving;
    int numOfPoints;

}MojRobot;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    cv::Mat frame[3];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);

    int processThisRobot(TKobukiData robotdata);

    int processThisCamera(cv::Mat cameraData);

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();
    void getNewFrame();
    bool vectorExist(int x, int y);

private:
     JOYINFO joystickInfo;
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     Robot robot;
     TKobukiData robotdata;
     int datacounter;
     float odchylka_pol;
     QTimer *timer;
     MojRobot mojRobot;
     QJoysticks *instance;
     std::vector<std::vector<char>> mapa;

     double forwardspeed;//mm/s
     double rotationspeed;//omega/s
     int offset[8][2] = {
                                                 {-1, -1},
                                                 {-1, 0},
                                                 {-1, 1},
                                                 {0, -1},
                                                 {0, 1},
                                                 {1, -1},
                                                 {1, 0},
                                                 {1, 1}
                                              };

public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo


};



/*#define tTM 0.000085292090497737556558
#define diameter 0.23
#define pi1 3.14159265359*/

#endif // MAINWINDOW_H
