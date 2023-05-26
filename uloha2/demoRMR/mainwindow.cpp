#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <iostream>
#include <queue>
#include <thread>
#include <chrono>


////Pavol Lukac & Denis Svec

static double x_pos = 0, y_pos = 0;
static queue<double> qxr, qyr;
static double yr = 0, xr = 0;
static double finish_X = 4.6, finish_Y = 1.8;



//LIDAR K coord 1,70,140,210 counter clockwise;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";
            // 192.168.1.14
  //  cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
  //  timer = new QTimer(this);
//    connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera1=false;




    datacounter=0;


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    ///prekreslujem obrazovku len vtedy, ked viem ze mam nove data. paintevent sa
    /// moze pochopitelne zavolat aj z inych dovodov, napriklad zmena velkosti okna
    painter.setBrush(Qt::black);//cierna farba pozadia(pouziva sa ako fill pre napriklad funkciu drawRect)
    QPen pero;
    pero.setStyle(Qt::SolidLine);//styl pera - plna ciara
    pero.setWidth(3);//hrubka pera -3pixely
    pero.setColor(Qt::green);//farba je zelena
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);
    painter.drawRect(rect);

    if(useCamera1==true && actIndex>-1)/// ak zobrazujem data z kamery a aspon niektory frame vo vectore je naplneny
    {
        ///std::cout<<actIndex<<std::endl;
        QImage image = QImage((uchar*)frame[actIndex].data, frame[actIndex].cols, frame[actIndex].rows, frame[actIndex].step, QImage::Format_RGB888  );//kopirovanie cvmat do qimage
        painter.drawImage(rect,image.rgbSwapped());
    }
    else
    {
        if(updateLaserPicture==1) ///ak mam nove data z lidaru
        {
            updateLaserPicture=0;

            painter.setPen(pero);
            //teraz tu kreslime random udaje... vykreslite to co treba... t.j. data z lidaru
         //   std::cout<<copyOfLaserData.numberOfScans<<std::endl;
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x(); //prepocet do obrazovky
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();//prepocet do obrazovky
                if(rect.contains(xp,yp) )//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
                    painter.drawEllipse(QPoint(xp, yp),2,2);


            }

        }
    }
}


/// toto je slot. niekde v kode existuje signal, ktory je prepojeny. pouziva sa napriklad (v tomto pripade) ak chcete dostat data z jedneho vlakna (robot) do ineho (ui)
/// prepojenie signal slot je vo funkcii  on_pushButton_9_clicked
void  MainWindow::setUiValues(double robotX,double robotY,double robotFi)
{
     ui->lineEdit_2->setText(QString::number(robotX));
     ui->lineEdit_3->setText(QString::number(robotY));
     ui->lineEdit_4->setText(QString::number(robotFi));
}
//int MainWindow::robotErrorDetection(TKobukiData robotdata){



//    std::printf("Thred1 \n");
//    return 0;
//}
///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
///
///
///
//void MainWindow::robotStagnationDetection(double xx1, double yy1){

//    location_vecto_x.push_back(xx1);
//    location_vecto_y.push_back(yy1);
//    float average = 0.0;
//    bool same_x= false;

//    if(location_vecto_x.size() >= 20){
//        for (const float& value : location_vecto_x) {
//            average += value;
//            //std::printf("%f - stop detection\n", value);
//              }
//              average /= location_vecto_x.size();

//                // Check if all values are close to the average within the given tolerance
//                for (float value : location_vecto_x) {
//                    if (std::abs(value - average) == 0.0) {

//                        same_x = true;
//                        std::printf("GOT HERE\n");
//                    }
//                }
//                if(same_x && average != 0){
//                    average = 0.0;
//                for (float value : location_vecto_y) {
//                    average += value;
//                      }
//                      average /= location_vecto_y.size();

//                        // Check if all values are close to the average within the given tolerance
//                        for (float value : location_vecto_y) {
//                            if (std::abs(value - average) == 0.0) {
//                                location_vecto_y.clear();
//                                location_vecto_x.clear();
//                                std::printf("STOOOOOOOOP\n");
//                                stuck= true;
//                            }
//                        }
//                }
//                location_vecto_y.clear();
//                location_vecto_x.clear();
//        }

//    std::this_thread::sleep_for(std::chrono::milliseconds(500));

// }

float calculateHypotenuse(float side1, float side2) {
    return std::sqrt(std::pow(side1, 2) + std::pow(side2, 2));
}

//bool crossProduct(float startPos_x, float startPos_y, float currentPos_x, float currentPos_y, float finalPos_x, float finalPos_y){
//    float x1 = currentPos_x - startPos_x;
//    float y1 = currentPos_y - startPos_y;
//    float x2 = finalPos_x - startPos_x;
//    float y2 = finalPos_y - startPos_y;
//    float cross = x1 * y2 - y1 * x2;

//    std::printf("%f -cross \n", cross);
//    if (std::abs(cross) <= 0.05){
//        return true;
//    }else{
//        return false;
//    }

//}
//void  MainWindow::ramp(double amp, bool forward_go){

//    if(forward_go){ //move
//        if(amp == 1){ //move forward
//            amp = 0;
//            while (amp <= 1){

//                       amp += 0.02;

//                       robot.setTranslationSpeed(200*amp);
//            }
//        }else{ //move backword
//            amp = 0;
//            while (amp >= -1){

//                       amp -= 0.02;

//                       robot.setTranslationSpeed(200*amp);
//            }


//        }
//    }else{ //stop
//        if(amp == 1){ //forward stop

//            while (amp >= 0){

//                       amp -= 0.02;

//                       robot.setTranslationSpeed(200*amp);
//            }
//        }else{ //move backword

//            while (amp <= 0){

//                       amp += 0.02;

//                       robot.setTranslationSpeed(200*amp);
//            }
//        }

//    }

//}


int MainWindow::processThisRobot(TKobukiData robotdata)
{
    static int Front_back_det = 1;
    static float distance_to_finish = 0.5;
    static bool start = true;
    static bool distance_count = false;
    static bool switch_go = false;
    static bool stuck = false;
    static bool step_one = false;
    static int bump_count = 0;
//    static bool step_two = false;
    static int previousEncoderLeft = robotdata.EncoderLeft, previousEncoderRight = robotdata.EncoderRight;
    ///static double odometerLeft, odometerRight = 0;

    static float previousRads = 0;
    static float distance_travelled =0;
    static float temp_distance =0;
    static float rads= 0;
//    static bool bug2 = false;
    //int rotation;
    static float x = 0, y = 0;
    static float last_movex = 0, last_movey = 0;

    static int speed = 300;
    static float side_distance = 0.6;
    static double tTM = 0.000085292090497737556558;

    static double pi1 = 3.14159265359;
    double ramp_dor =1;
    //static float e_sum = 0;
    double diameter = 0.23;
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);

    static double cloud[360][3];
    int side_int = 1;


    int scale = 1000;


    //body kde ma ist robot
    if(start){

         qxr.push(4.6);


         qyr.push(1.8);
        start = false;
    }





    if(abs(previousEncoderLeft - robotdata.EncoderLeft) > 10000){
       printf("\nLeft encoder pretec\n");
       if(previousEncoderLeft > robotdata.EncoderLeft){
           previousEncoderLeft -= 65535;
       }else if(previousEncoderLeft < robotdata.EncoderLeft) previousEncoderLeft += 65535;

    }
    if(abs(previousEncoderRight - robotdata.EncoderRight) > 10000){
       printf("\nRight encoder pretec\n");
       if(previousEncoderRight > robotdata.EncoderRight){
           previousEncoderRight -= 65535;
       }else if(previousEncoderRight < robotdata.EncoderRight) previousEncoderRight += 65535;

    }

    float rightWheel = tTM*(robotdata.EncoderRight - previousEncoderRight);
    float leftWheel = tTM*(robotdata.EncoderLeft - previousEncoderLeft);

    if((rightWheel != 0) || (leftWheel != 0)){
        mojRobot.moving = true;
    }else if((rightWheel == 0) && (leftWheel == 0)) mojRobot.moving = false;


    previousEncoderLeft = robotdata.EncoderLeft;
    previousEncoderRight = robotdata.EncoderRight;



        rads += (rightWheel - leftWheel)/diameter;
        if(rads > (6.283185/2)){
            rads = -(6.283185/2) + (rads-(6.283185/2));
        }else if(rads < -(6.283185/2)){
            rads = (6.283185/2) + (rads + (6.283185/2));
        }

        x +=  ((rightWheel + leftWheel)/2.0)*cos(rads);
        y += ((rightWheel + leftWheel)/2.0)*sin(rads);


    //printf("\n%d",mojRobot.stop);

        //std::printf("%f - x\n", x);
        //std::printf("%f - y\n", y);
        x_pos = x;
        y_pos = y;
        previousRads = rads;


        emit uiValuesChanged(x, y, previousRads);

        ///POLOHOVANIE

        //pridat rampovanie s HMI
        //Pridat detekciu finalneho bodu
        //Pridat opravu kolizie (cuvanie dozadu?)

//        bug2 = crossProduct(0,0,x,y,finish_X,finish_Y);


        for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
        {

            temp_distance = copyOfLaserData.Data[k].scanDistance/scale;
            if (stuck == false && temp_distance <= 0.22 && temp_distance >= 0.01){

                robot.setTranslationSpeed(0);
                bump_count += 1;

                if(k <=70 || k >=205){
                    //predok
                    Front_back_det = -1;
                }else{
                    Front_back_det = 1;
                }

                last_movex= x;
                last_movey= y;
                stuck = true;
                step_one = true;


//                step_two = false;
                std::printf("STUCK\n");
               // std::printf("%f - %d \n", (copyOfLaserData.Data[k].scanDistance/scale), k);
            }
//            if (temp_distance <= 0.3 && temp_distance >= 0.01){
//                bug2 = false;
//            }


            if(k == 0 || k == 68|| k == 207  || k == 28 || k == 247){

                if( k == 0){
                    //front 0deg
                    side_int=0;
                }else if ( k == 68){
                    //left 90deg
                    side_int=1;

                }else if( k == 207){
                    //right271 deg
                    side_int=2;
                }else if(k == 28){
                    //left 38deg
                    side_int=3;
                }else if (k == 247){
                    //right38 deg
                    side_int=4;
                }

                //distance
                cloud[side_int][0]=copyOfLaserData.Data[k].scanDistance/scale; ///vzdialenost nahodne predelena 20 aby to nejako vyzeralo v okne.. zmen podla uvazenia
                //x
                cloud[side_int][1]=(rect.width()-(rect.width()/2+cloud[side_int][0]*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x())/scale; //prepocet do obrazovky
                //y
                cloud[side_int][2]=(rect.height()-(rect.height()/2+cloud[side_int][0]*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y())/scale;//prepocet do obrazovky
                //std::printf("%f - %d \n", (((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0) * 180/3.14159), k);
               //std::printf("%f - %d \n", (copyOfLaserData.Data[k].scanDistance/scale), k);
            }

        }


            //std::thread thread1(&MainWindow::robotStagnationDetection,this,x, y);

//double side_dis = calculateHypotenuse(0.15,0.1);

//std::printf("%f -this \n", side_dis);
//if(side_move == false && stuck == false){
//        if(calculate_Distance((cloud[0][1] + x_pos),(cloud[0][2] + y_pos),finish_X,finish_Y) <= 0.3){

//            robot.setRotationSpeed(0);
//            robot.setTranslationSpeed(0);
//            MoveRobot( x,  y, rads);


distance_travelled = abs(calculate_Distance(last_movex,last_movey,x,y));



//std::printf("%f  \n", distance_travelled);


if(abs(calculate_Distance(finish_X,finish_Y,x,y)) <= 0.03){
    robot.setTranslationSpeed(0);
    robot.setRotationSpeed(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    distance_to_finish = distance_to_finish*2;
    std::printf("FINISH stop\n");
}
else if(abs(calculate_Distance(finish_X,finish_Y,x,y)) <= distance_to_finish){
    robot.setRotationSpeed(0);
    robot.setTranslationSpeed(0);
    wall_follow = false;
    Left_wall = false;
    MoveRobot( x,  y, rads);

     std::printf("FINISH move\n");

//}else if(bug2){
//    robot.setRotationSpeed(0);
//    robot.setTranslationSpeed(0);
//    MoveRobot( x,  y, rads);

//    bug2 = false;


}else{


    if(stuck){



    //    else if(step_two){
    //        robot.setTranslationSpeed(speed);
    //        std::printf("stuck 2\n");
    //        if(distance_travelled >= 0.1 ){
    //            last_movex= x;
    //            last_movey= y;

    //            step_two = false;
    //            stuck = false;
    //        }

    //    }
        if(distance_travelled >= 0.1 ){
    //        std::printf("stuck 3\n");
            robot.setTranslationSpeed(0);
            robot.setRotationSpeed(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));


            last_movex= x;
            last_movey= y;

            switch_go = true;
            step_one = false;
            stuck = false;
            //v

        }else if(step_one){

            //robot.setTranslationSpeed(Front_back_det*speed);
            robot.setTranslationSpeed(Front_back_det*300);
    //        std::printf("stuck 1\n");
        }



    }else if(switch_go){

//        if(((cloud[0][0] <= 0.10 && cloud[0][0] >= 0.01) || (cloud[1][0] <= 0.10 && cloud[1][0] >= 0.01) || (cloud[2][0] <= 0.10 && cloud[2][0] >= 0.01))){
//            ramp(1, false);
//            distance_count = false;
//            switch_go = false;
//        }
//        else
    if(distance_travelled >= 0.2 && distance_count){
            robot.setTranslationSpeed(0);
                std::printf("stop move\n");
                last_movex= x;
                last_movey= y;
            distance_count = false;
            switch_go = false;
            bump_count = 0;
        }else if(distance_count){

                robot.setTranslationSpeed(300);
    }
    }else if(switch_go == false){

     if(bump_count >=1){

         robot.setTranslationSpeed(0);
         robot.setRotationSpeed(-1);

         //if(copyOfLaserData.Data[0].scanDistance/scale >= 0.6 && copyOfLaserData.Data[0].scanDistance/scale != 0){
         temp_distance = copyOfLaserData.Data[0].scanDistance/scale;
             while(temp_distance >= 0.4 && temp_distance >= 0.01){
                 std::this_thread::sleep_for(std::chrono::milliseconds(100));
                 //std::printf("%f  \n", (copyOfLaserData.Data[1].scanDistance/scale));
                 robot.setRotationSpeed(-1);
                 std::printf("loop3 \n");
                 temp_distance = copyOfLaserData.Data[0].scanDistance/scale;
             }
        robot.setRotationSpeed(0);
        std::printf("bump \n");
         //}
        bump_count =0;

    }else if (((cloud[0][0] <= 0.40 && cloud[0][0] >= 0.01) || (cloud[1][0] <= 0.40 && cloud[1][0] >= 0.01) || (cloud[2][0] <= 0.40 && cloud[2][0] >= 0.01))){

                    robot.setTranslationSpeed(0);


                    robot.setRotationSpeed(1);


                    //std::printf("%f \n", (copyOfLaserData.Data[0].scanDistance/scale));
                        temp_distance = copyOfLaserData.Data[0].scanDistance/scale;
                        if(temp_distance <= 1 && temp_distance >= 0.01){

                            while((temp_distance <= 1 && temp_distance >= 0.01) || (copyOfLaserData.Data[28].scanDistance/scale <= side_distance && copyOfLaserData.Data[28].scanDistance/scale >= 0.01)|| (copyOfLaserData.Data[247].scanDistance/scale <= side_distance && copyOfLaserData.Data[247].scanDistance/scale >= 0.01)){
                                std::this_thread::sleep_for(std::chrono::milliseconds(10));
    //                            std::printf("%f  \n", (copyOfLaserData.Data[1].scanDistance/scale));
                                //std::printf("here x1 \n");
                                robot.setRotationSpeed(1);
                                std::printf("loop2 \n");
                                temp_distance = copyOfLaserData.Data[0].scanDistance/scale;
                            }

                        }
    //                    else if (copyOfLaserData.Data[28].scanDistance/scale <= side_distance || copyOfLaserData.Data[247].scanDistance/scale <= side_distance){
    //                        while((copyOfLaserData.Data[0].scanDistance/scale <= 1 && copyOfLaserData.Data[0].scanDistance/scale >= 0.01) || (copyOfLaserData.Data[28].scanDistance/scale <= side_distance && copyOfLaserData.Data[28].scanDistance/scale >= 0.01)|| (copyOfLaserData.Data[247].scanDistance/scale <= side_distance && copyOfLaserData.Data[247].scanDistance/scale >= 0.01) ){
    //                            std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ////                            std::printf("%f  \n", (copyOfLaserData.Data[1].scanDistance/scale));
    //                            std::printf("here y1 \n");
    //                        }
    //                    }

                        robot.setRotationSpeed(0);

                    std::printf("here1 \n");
                    //robot.setTranslationSpeed(speed);
                    if(distance_count == false){
                        last_movex= x;
                        last_movey= y;
                        distance_count = true;

                    }
                    switch_go = true;
                    wall_follow = true;
                    bump_count =0;

                }else if((wall_follow) && ((cloud[1][0] >= 0.6) || (cloud[2][0] >= 0.6) || (copyOfLaserData.Data[140].scanDistance/scale >= 0.6))){
                    //std::printf("here2 \n");
                    robot.setTranslationSpeed(0);


                        robot.setRotationSpeed(-1);

                        //if(copyOfLaserData.Data[0].scanDistance/scale >= 0.6 && copyOfLaserData.Data[0].scanDistance/scale != 0){
                        temp_distance = copyOfLaserData.Data[0].scanDistance/scale;
                            while(temp_distance >= 0.8){
                                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                                //std::printf("%f  \n", (copyOfLaserData.Data[1].scanDistance/scale));
                                std::printf("loop1 \n");
                                robot.setRotationSpeed(-1);
                                temp_distance = copyOfLaserData.Data[0].scanDistance/scale;
                            }

                        //}


                    std::printf("here3 \n");
                    robot.setRotationSpeed(0);

                    if(distance_count == false){
                        last_movex= x;
                        last_movey= y;
                        distance_count = true;
                    }
                    //robot.setTranslationSpeed(speed);
                    switch_go = true;


                }else{
                    std::printf("here4 \n");
                    wall_follow = false;
                    Left_wall = false;
                    MoveRobot( x,  y, rads);

                }

    }
}

//thread1.join();

        datacounter++;

        return 0;

}

double MainWindow::calculate_Distance(double xx1, double yy1, double xx2, double yy2) {
    double dx = xx2 - xx1;
    double dy = yy2 - yy1;
    double distance = std::sqrt(dx * dx + dy * dy);
    return distance;
}

void MainWindow::MoveRobot(float x, float y,float rads)
{
    static double fi = 0;
    double Pr = 1.00;
    int Pt = 500;
    static bool centered = false;
    static int translation = 0;
    static double arc_reg = 10000;



                if(!qyr.empty()){
                    yr = qyr.front();
                    xr = qxr.front();
                }
                double finish = 0.1;
                /*if (qyr.size() > 1){
                    finish = 0.2;
                }*/


                if((yr-y) >= 0){
                    fi = acos((xr-x)/(sqrt(pow((xr-x),2) + pow((yr-y),2) )));

                }else if((xr-x) >= 0){
                    fi = asin((yr-y)/(sqrt(pow((xr-x),2) +
                                           pow((yr-y),2) )));

                }else{
                    fi = (-6.283185/2)+atan((yr-y)/(xr-x));
                }

                double e_fi = fi - rads;
                if(e_fi > (6.283185/2)){
                    e_fi -= 6.283185;
                }else if(e_fi < (-6.283185/2)){
                    e_fi +=6.283185;
                }


                float e_pos =sqrt(pow((xr-x),2)+ pow((yr-y),2));
                float translation_reg = Pt * e_pos + 50;
                arc_reg = 200/e_fi;
                if(arc_reg == 0) arc_reg = 32767;



                if(abs(e_fi) >= 0.2) centered = false;

                if((abs(e_fi) < 0.2) && centered){

                    if(translation_reg > 400){
                        translation_reg = 400;
                    }

                    if(translation <= translation_reg){
                        translation += 5;
                    }else if(translation > translation_reg) translation -= 5;

                    if(translation > translation_reg) translation = translation_reg;
                    if(translation < 0) translation = 0;

                    if(e_pos < finish){
                        translation = 0;
                        if(!qyr.empty()){
                            qyr.pop();
                            qxr.pop();
                        }
                    }

                    robot.setArcSpeed(translation,arc_reg);


                }else{
                    translation -= 10;
                    if(translation < 0) translation = 0;
                    if(translation > 0) robot.setTranslationSpeed(translation);

                    if(translation == 0){
                        double rotacia = Pr*e_fi;
                        if(rotacia > 3.14159/3) rotacia = 3.14159/3;
                        if(rotacia < -3.14159/3) rotacia = -3.14159/3;
                        robot.setRotationSpeed(rotacia);

                    }
                    if(abs(e_fi) < 0.04) centered = true;
                }




}

void MainWindow::RotateRobot(float x, float y,float rads)
{
    static double fi = 0;
    double Pr = 1.00;
    static int translation = 0;


    if(!qyr.empty()){
        yr = qyr.front();
        xr = qxr.front();
    }

                if((yr-y) >= 0){
                    fi = acos((xr-x)/(sqrt(pow((xr-x),2) + pow((yr-y),2) )));

                }else if((xr-x) >= 0){
                    fi = asin((yr-y)/(sqrt(pow((xr-x),2) +
                                           pow((yr-y),2) )));

                }else{
                    fi = (-6.283185/2)+atan((yr-y)/(xr-x));
                }

                double e_fi = fi - rads;
                if(e_fi > (6.283185/2)){
                    e_fi -= 6.283185;
                }else if(e_fi < (-6.283185/2)){
                    e_fi +=6.283185;
                }



                    if(translation < 0) translation = 0;
                    if(translation > 0) robot.setTranslationSpeed(translation);

                    if(translation == 0){
                        double rotacia = Pr*e_fi;
                        if(rotacia > 3.14159/3) rotacia = 3.14159/3;
                        if(rotacia < -3.14159/3) rotacia = -3.14159/3;
                        robot.setRotationSpeed(rotacia);

                    }






}


///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    QRect rect;
    rect= ui->frame->geometry();//ziskate porametre stvorca,do ktoreho chcete kreslit
    rect.translate(0,15);



    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru



    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}


///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z kamery
int MainWindow::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    ///cout<<"W: " << cameraData.size().width<< endl;
    ///cout<<"H: " << cameraData.size().height<< endl;
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka
    updateLaserPicture=1;
    return 0;
}
void MainWindow::on_pushButton_9_clicked() //start button
{

    forwardspeed=0;
    rotationspeed=0;
    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    connect(this,SIGNAL(uiValuesChanged(double,double,double)),this,SLOT(setUiValues(double,double,double)));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robot.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&MainWindow::processThisLidar,this,std::placeholders::_1));
    robot.setRobotParameters(ipaddress,53000,5300,std::bind(&MainWindow::processThisRobot,this,std::placeholders::_1));
    robot.setCameraParameters("http://"+ipaddress+":8889/stream.mjpg",std::bind(&MainWindow::processThisCamera,this,std::placeholders::_1));
    /// 8000
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robot.robotStart();



    //ziskanie joystickov
    instance = QJoysticks::getInstance();


    /// prepojenie joysticku s jeho callbackom... zas cez lambdu. neviem ci som to niekde spominal,ale lambdy su super. okrem toho mam este rad ternarne operatory a spolocneske hry ale to tiez nikoho nezaujima
    /// co vas vlastne zaujima? citanie komentov asi nie, inak by ste citali toto a ze tu je blbosti
    connect(
        instance, &QJoysticks::axisChanged,
        [this]( const int js, const int axis, const qreal value) { if(/*js==0 &&*/ axis==1){forwardspeed=-value*300;}
            if(/*js==0 &&*/ axis==0){rotationspeed=-value*(3.14159/2.0);}}
    );
}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu + rampa
    int targetVelocity,currentVelocity;


    targetVelocity = 50;
    /*currentVelocity = 50;
    robot.setTranslationSpeed(currentVelocity);
    while(currentVelocity < targetVelocity){
        this_thread::sleep_for(50ms);
        currentVelocity += 10;
        robot.setTranslationSpeed(currentVelocity);
    }*/
     robot.setTranslationSpeed(targetVelocity);


}

void MainWindow::on_pushButton_3_clicked() //back
{
    robot.setTranslationSpeed(-250);

}

void MainWindow::on_pushButton_6_clicked() //left
{
robot.setRotationSpeed(3.14159/3);

}

void MainWindow::on_pushButton_5_clicked()//right
{
robot.setRotationSpeed(-3.14159/3);

}

void MainWindow::on_pushButton_4_clicked() //stop
{
    robot.setTranslationSpeed(0);

}




void MainWindow::on_pushButton_clicked()
{
    if(useCamera1==true)
    {
        useCamera1=false;

        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera1=true;

        ui->pushButton->setText("use laser");
    }
}

void MainWindow::getNewFrame()
{

}
