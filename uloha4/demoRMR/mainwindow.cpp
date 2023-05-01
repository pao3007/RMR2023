#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include <fstream>



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
    static bool test1 = true;


    if(test1){
        printf("IDEM0\n");
        int algMapa[300][300] = {0};


        int idx[2] = {0,0};
        Index nodeIdx;
        Alg alg;


        for (int i = 0; i < mapa.size(); i++)
        {
            for (int j = 0; j < mapa[i].size(); j++)
            {
                if(mapa[i][j] == 'G'){
                    idx[0] = i;
                    idx[1] = j;
                }
            }
        }
        alg.current.x = idx[0];
        alg.current.y = idx[1];

        for (int k = 0; k < 8; k++)
        {
            int indexX = alg.current.x + offset[k][0];
            int indexY = alg.current.y + offset[k][1];
             mapa[indexX][indexY] = 'u';
             nodeIdx.x = indexX;
             nodeIdx.y = indexY;
             alg.unvisited.push(nodeIdx);
        }
        int test = 0;
        while(!alg.unvisited.empty()){
            nodeIdx = alg.unvisited.front();
            alg.unvisited.pop();
            alg.current.x = nodeIdx.x;
            alg.current.y = nodeIdx.y;
            int min = 999999;

            for (int k = 0; k < 8; k++)
            {
                int indexX = alg.current.x + offset[k][0];
                int indexY = alg.current.y + offset[k][1];
                if(mapa[indexX][indexY] == '0'){
                    mapa[indexX][indexY] = 'u';
                    nodeIdx.x = indexX;
                    nodeIdx.y = indexY;
                    alg.unvisited.push(nodeIdx);
                }else if(algMapa[indexX][indexY] > 1 && mapa[indexX][indexY] != 'G'){
                    if(algMapa[indexX][indexY] < min){
                        min = algMapa[indexX][indexY];
                    }
                }else if(mapa[indexX][indexY] == 'G'){
                    min = 1;
                }
            }
            mapa[alg.current.x][alg.current.y] = 'v';
            algMapa[alg.current.x][alg.current.y] = min+1;
            ///std::cout<<mapa[alg.current.x][alg.current.y]<<std::endl;

            //std::cout<<test<<std::endl;
        }

        Index smallestIndex;
        int min = algMapa[100][100];
        alg.current.x = 100;
        alg.current.y = 100;
        bool end = false;
        int test0 = 0;
        while(!end){

            for (int k = 0; k < 8; k++)
            {
                int indexX = alg.current.x + offset[k][0];
                int indexY = alg.current.y + offset[k][1];
                if(indexX < 300 && indexX >= 0 && indexY < 300 && indexY >= 0){
                    if(algMapa[indexX][indexY]<min && algMapa[indexX][indexY] > 0){

                        min = algMapa[indexX][indexY];
                        smallestIndex.x = indexX;
                        smallestIndex.y = indexY;

                    }

                }

            }
            alg.current.x = smallestIndex.x;
            alg.current.y = smallestIndex.y;
            path.push_back(smallestIndex);
            cout << algMapa[smallestIndex.x][smallestIndex.y] << endl;
            if(algMapa[smallestIndex.x][smallestIndex.y] < 3) end = true;

        }
        Index prevIdx = path.front();
        Index prevSmer;

        prevSmer.x = 0;
        prevSmer.y = 0;

        for(int k = 0; k < path.size();k++){

            int dx = path[k].x - prevIdx.x;
            int dy = path[k].y - prevIdx.y;
            if(prevSmer.x != dx && prevSmer.y != dy){
               pathPoints.push_back(path[k]);
            }
            prevSmer.x = dx;
            prevSmer.y = dy;

        }



        //////////////////
        ofstream occGridALG("C:/Users/lukac/Desktop/RMR/RMR2023/uloha4/occGridALG.txt");
        ofstream occGridALG2("C:/Users/lukac/Desktop/RMR/RMR2023/uloha4/occGridALG2.txt");
        ///ofstream occGridALG("C:/Users/pao/Desktop/RMR/RMR2023/uloha4/occGridALG.txt");
        printf("Zapisujem do mapy");

        for (int i = 0; i < mapa.size(); i++) {
            for (int j = 0; j < mapa[0].size(); j++) {


                bool pathWriten = false;

                for(int k = 0; k < pathPoints.size();k++){
                    if(pathPoints[k].x == i && pathPoints[k].y == j){
                        occGridALG << 'T';
                        pathWriten = true;
                        ///path.erase(path.begin() + k);
                    }
                    if(pathWriten){
                        break;
                    }

                }


                for(int k = 0; k < path.size();k++){
                    if(pathWriten){
                        break;
                    }
                    if(path[k].x == i && path[k].y == j){
                        occGridALG << 'X';
                        pathWriten = true;
                        ///path.erase(path.begin() + k);
                    }

                }


                if(!pathWriten) occGridALG << mapa[i][j];

                if(algMapa[i][j] > 2){

                    occGridALG2 << 'U';
                }else occGridALG2 << algMapa[i][j];

                ///std::cout << i << ":" << j << " = " << algMapa[i][j] << std::endl;



            }
            occGridALG << endl;
            occGridALG2 << endl;
        }
        occGridALG.close();
        occGridALG2.close();

        test1=false;



    }







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
    static int prevNumOfPoints = mojRobot.numOfPoints;
    static float odchylkaCelkova = odchylka_pol;
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));


    const int gSize = 300;
    static int occ_grid[gSize][gSize];

        for(int k=0;k<copyOfLaserData.numberOfScans;k++){

            if(((copyOfLaserData.Data[k].scanDistance/1000) < 0.23) && (copyOfLaserData.Data[k].scanDistance/1000) != 0) i++;
        }


        if(i > 0){
            mojRobot.stop = true;
        }else mojRobot.stop = false;



        updateLaserPicture=1;
        update();


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
    std::string eachrow;

    std::ifstream myfile("C:/Users/lukac/Desktop/RMR/RMR2023/uloha4/occGrid_ideal.txt");
    ///std::ifstream myfile("C:/Users/pao/Desktop/RMR/RMR2023/uloha4/occGrid_ideal.txt");

    while (std::getline(myfile, eachrow))
    {
        std::vector<char> row;

        for (char &x : eachrow)
        {
            if (x != ' ')row.push_back(x);
        }

        mapa.push_back(row);
    }

    for(int k = 0; k < 3; k++){
        for (int i = 0; i < mapa.size(); i++)
        {
            for (int j = 0; j < mapa[i].size(); j++)
            {
                if(mapa[i][j] == '1'){

                    for (int k = 0; k < 8; k++)
                    {
                        int indexX = i + offset[k][0];
                        int indexY = j + offset[k][1];
                        if(mapa[indexX][indexY] != '1')
                            mapa[indexX][indexY] = '2';
                    }
                }
            }
        }

        for (int i = 0; i < mapa.size(); i++)
        {
            for (int j = 0; j < mapa[i].size(); j++)
            {
                if(mapa[i][j] == '2')mapa[i][j] = '1';
            }
        }
    }

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

bool MainWindow::vectorExist(int x, int y){
    if(x < mapa[0].size() && x >= 0 && y < mapa.size() && y >= 0){
        return true;
    }else return false;

}
