#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
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
namespace Ui {
class MainWindow;
}
typedef struct
{
    int scanQuality;
    double scanAngle;
    double scanDistance;
}MojeLaserData;

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
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera1;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];

    cv::Mat frame[3];
    bool Left_wall = false;
    bool wall_follow = false;


    double side_check[4];
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    int processThisLidar(LaserMeasurement laserData);
    void ramp(double amp, bool forward_go);
    std::vector<float> location_vecto_x;
    std::vector<float> location_vecto_y;
    int processThisRobot(TKobukiData robotdata);
    int robotErrorDetection(TKobukiData robotdata);
    void robotStagnationDetection(double xx1, double yy1);
    int processThisCamera(cv::Mat cameraData);
    void MoveRobot(float x, float y,float rads);
    void RotateRobot(float x, float y,float rads);
    double calculate_Distance(double xx1, double yy1, double xx2, double yy2);

private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();
    void getNewFrame();

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

     double forwardspeed;//mm/s
     double rotationspeed;//omega/s
public slots:
     void setUiValues(double robotX,double robotY,double robotFi);
signals:
     void uiValuesChanged(double newrobotX,double newrobotY,double newrobotFi); ///toto nema telo


};

/*#define tTM 0.000085292090497737556558
#define diameter 0.23
#define pi1 3.14159265359*/

#endif // MAINWINDOW_H
