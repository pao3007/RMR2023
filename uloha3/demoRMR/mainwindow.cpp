#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <iostream>
#include <queue>

////Pavol Lukac & Denis Svec


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{

    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    //ipaddress="192.168.1.14";
    ipaddress="127.0.0.1";
            //
    // 127.0.0.1
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
                if(rect.contains(xp,yp))//ak je bod vo vnutri nasho obdlznika tak iba vtedy budem chciet kreslit
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

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int MainWindow::processThisRobot(TKobukiData robotdata)
{
    static bool start = true;
    static int previousEncoderLeft = robotdata.EncoderLeft, previousEncoderRight = robotdata.EncoderRight;
    ///static double odometerLeft, odometerRight = 0;
    static float x = 0, y = 0;
    static float previousRads = 0;
    static queue<double> qxr, qyr;
    static double yr = 0, xr = 0;
    static double fi = 0;
    static float rads= 0;
    static int translation = 0;
    static int prev_translation = 0;
    static double arc_reg = 10000;
    //int rotation;
    double Pr = 2.8;
    int Pt = 500;
    static bool centered = false;
    double tTM = 0.000085292090497737556558;
    double diameter = 0.23;
    static bool mapovanie = false;
    static int timer = 0;
    static float e_pos = 0;
    double pi1 = 3.14159265359;
    double finish = 0.15;
    //static float e_sum = 0;

    if(start){
       /* qxr.push(5.00);
        qyr.push(0.00);*/
        qxr.push(0);qyr.push(0);

        //qxr.push(0);qyr.push(1.45);
        qxr.push(0);qyr.push(3.20);

        //qxr.push(1.40);qyr.push(2.90);
        qxr.push(2.70);qyr.push(3.30);

        qxr.push(3.70);qyr.push(3.50);

        qxr.push(2.70);qyr.push(3.30);

       // qxr.push(2.70);qyr.push(1.30);
        qxr.push(2.70);qyr.push(0.0);

        //qxr.push(3.5);qyr.push(0.0);
        qxr.push(4.6);qyr.push(0.0);

        qxr.push(4.6);qyr.push(1.8);
        ///
        ///









        mojRobot.numOfPoints = qxr.size();
        start = false;
    }


    /*        qxr.push(0);
        qxr.push(2.60);
        qxr.push(2.60);
        qxr.push(4.5);
        qxr.push(4.5);

        qyr.push(2.90);
        qyr.push(2.90);
        qyr.push(0.0);
        qyr.push(0.0);
        qyr.push(1.6);*/








///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX



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

        if(translation == 0){
            rads = (robotdata.GyroAngle/100.0) * (pi1/180.0);
        }else {
            rads += (rightWheel - leftWheel)/diameter;
            if(rads > (6.283185/2)){
                rads = -(6.283185/2) + (rads-(6.283185/2));
            }else if(rads < -(6.283185/2)){
                rads = (6.283185/2) + (rads + (6.283185/2));
            }
        }

        if((rightWheel - leftWheel) != 0 && translation != 0){
            x += ((diameter*(rightWheel + leftWheel)) / (2.0*(rightWheel - leftWheel)))*(sin(rads) - sin(previousRads));
            y -= ((diameter*(rightWheel + leftWheel)) / (2.0*(rightWheel - leftWheel)))*(cos(rads) - cos(previousRads));

        }else{
            x += ((rightWheel + leftWheel)/2.0)*cos(rads);
            y += ((rightWheel + leftWheel)/2.0)*sin(rads);

        }

        previousRads = rads;

        //printf("\n%d",mojRobot.stop);


        ///POLOHOVANIE
        if(!mojRobot.stop/* && !mapovanie*/){


            if(!qyr.empty()){
                yr = qyr.front();
                xr = qxr.front();
            }

            /*if (qyr.size() > 1){
                finish = 0.2;
            }*/


            /*if((yr-y) ==0 && (xr-x)==0){
                fi = rads;
            }else if((yr-y) >= 0){
                fi = acos((xr-x)/(sqrt(pow((xr-x),2) + pow((yr-y),2) )));

            }else if((xr-x) >= 0){
                fi = asin((yr-y)/(sqrt(pow((xr-x),2) +
                                       pow((yr-y),2) )));

            }else {
                fi = (-6.283185/2)+atan((yr-y)/(xr-x));
            }*/

            fi = atan2(yr-y,xr-x);

            double e_fi = fi - rads;
            if(e_fi > (6.283185/2)){
                e_fi -= 6.283185;
            }else if(e_fi < (-6.283185/2)){
                e_fi +=6.283185;
            }


            float e_pos =sqrt(pow((xr-x),2)+ pow((yr-y),2));
            odchylka_pol = e_pos;
            float translation_reg = Pt * e_pos + 50;
            arc_reg = 100/e_fi;
            if(arc_reg == 0) arc_reg = 35000;



            if(abs(e_fi) >= 0.2) centered = false;

            if((abs(e_fi) < 0.2) && centered){


                if(e_pos < finish){
                    translation = 0;

                    if(!qyr.empty()){
                        qyr.pop();
                        qxr.pop();
                        mojRobot.numOfPoints = qxr.size();

                    }
                    mapovanie = true;
                }else{

                    if(translation_reg > 400){
                        translation_reg = 400;
                    }

                    if(translation <= translation_reg){
                        translation += 5;
                    }else if(translation > translation_reg) translation -= 5;

                    if(translation > translation_reg) translation = translation_reg;
                    if(translation < 0) translation = 0;
                }

                robot.setArcSpeed(translation,arc_reg);


            }else{
                translation -= 10;
                if(translation < 0) translation = 0;
                if(translation > 0) robot.setTranslationSpeed(translation);

                if(translation == 0){
                    double rotacia = Pr*e_fi;
                     if(e_pos > finish){
                        if(rotacia > 3.14159/3) rotacia = 3.14159/3;
                        if(rotacia < -3.14159/3) rotacia = -3.14159/3;
                        robot.setRotationSpeed(rotacia);
                     }


                }
                if(abs(e_fi) < 0.03) centered = true;
            }
    }else{
            translation -= 50;
            if (translation < 0) translation = 0;
            robot.setArcSpeed(translation,arc_reg);
            printf("\nEMERGENCY STOP");
        }

   /* if(mapovanie){
        timer++;
        if(timer >= 120){
            mapovanie = false;
            timer = 0;
        }
    }*/



    emit uiValuesChanged(e_pos, fi, rads/*e_fi/*rads*(180/pi1)*/);


    datacounter++;
    prev_translation = translation;

    mojRobot.angle = rads;
    mojRobot.x = x;
    mojRobot.y = y;
    mojRobot.translation = translation;

    return 0;

}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii on_pushButton_9_clicked
/// vola sa ked dojdu nove data z lidaru
int MainWindow::processThisLidar(LaserMeasurement laserData)
{
    int i = 0;
    LaserMeasurement prevCopyOfLaserData;
    static int prevNumOfPoints = mojRobot.numOfPoints;
    static float odchylkaCelkova = odchylka_pol;
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));


    const int gSize = 300;
    static int occ_grid[gSize][gSize];
    double pi1 = 3.14159265359;
    double rads, lidarAngle, calAngle;

    if(mojRobot.angle < 0){
        rads = mojRobot.angle + 6.283185;
    }else rads = mojRobot.angle;

    int Xgi;
    int Ygi;
    double scanDist;

    if(mojRobot.translation != 0 && odchylka_pol > 0.5 && (odchylkaCelkova*0.97 < odchylka_pol)){

        for(int k=0;k<copyOfLaserData.numberOfScans;k++){
            scanDist = copyOfLaserData.Data[k].scanDistance / 1000;

            if(((copyOfLaserData.Data[k].scanDistance/1000) < 0.23) && (copyOfLaserData.Data[k].scanDistance/1000) != 0) i++;


            if((scanDist > 0.13) && (scanDist < 3.0) && !((scanDist < 0.7) && (scanDist > 0.62))){
                lidarAngle = copyOfLaserData.Data[k].scanAngle * (pi1/180.0);
                calAngle = rads - lidarAngle;
                if(calAngle > 6.283185) calAngle -= 6.283185;
                ///printf("MERAM");
                Xgi = gSize/3 + (int)((mojRobot.x + scanDist*cos(calAngle))/0.05);
                Ygi = gSize/3 + (int)((mojRobot.y + scanDist*sin(calAngle))/0.05);
                occ_grid[Xgi][Ygi] = 1;
            }
        }
    }

        if(i > 0){
            mojRobot.stop = true;
        }else mojRobot.stop = false;



        if(prevNumOfPoints != mojRobot.numOfPoints){
            odchylkaCelkova = odchylka_pol;
            ofstream mapa("C:/Users/lukac/Desktop/RMR/RMR2023/uloha3/mapa.txt");
            ofstream occGrid("C:/Users/lukac/Desktop/RMR/RMR2023/uloha3/occGrid.txt");
            printf("Zapisujem do mapy");
            for (int i = 0; i < gSize; i++) {
                for (int j = 0; j < gSize; j++) {
                    occGrid << occ_grid[j][i];
                    if(occ_grid[j][i] == 0){
                        mapa << " ";
                    }else mapa << "x";
                }
                occGrid << endl;
                mapa << endl;
            }
            occGrid.close();
            mapa.close();
        }
        prevNumOfPoints = mojRobot.numOfPoints;
        updateLaserPicture=1;
        update();

    memcpy( &prevCopyOfLaserData,&laserData,sizeof(LaserMeasurement));
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

     robot.setTranslationSpeed(350);


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
