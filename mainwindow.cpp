#include "mainwindow.h"
#include "ui_mainwindow.h"

/*
 * UI介绍
 * 1、按钮 “打开串口”。列出系统中存在的串口，让用户自己选择对应的usb 串口，最后获取对应的串口名称。后续使用这个串口名称进行
 * 串口打开动作。
 *
 * 2、舵机的ID选择以及显示电池电量的Label。不用实时的采集，1s更新一次即可。
 *
 * 3、采集的视频帧分辨率选择列表。
 *
 * 4、在窗体中显示采集与处理后的video frame；包括在video frame上显示角度偏移量[如果找不到则回复初始位置]。
 *
 * */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle(QString("摄像头跟踪物体程序"));
    serialPortForServoMotor = new SerialPortForServoMotor();

    //串口设置
    comboBox_SerialPort = ui->comboBox_SerialPort;
    comboBox_baudRate = ui->comboBox_baudRate;
    comboBox_dataBits = ui->comboBox_dataBits;
    comboBox_parity = ui->comboBox_parity;
    comboBox_stopBits = ui->comboBox_stopBits;
    comboBox_flowControl = ui->comboBox_flowControl;

    pushButton_openSerialPort  = ui->pushButton_openSerialPort;
    pushButton_closeSerialPort = ui->pushButton_closeSerialPort;

    comboBox_dataBits->setCurrentIndex(3); //设置dataBits的默认值
    list_SerialPortInfo = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &info,list_SerialPortInfo) {
        comboBox_SerialPort->addItem(info.portName()+QString(" ")+info.description());
    }
    //ServoMotor Move test
    pushButton_up = ui->pushButton_Up;
    pushButton_down = ui->pushButton_Down;
    pushButton_left = ui->pushButton_Left;
    pushButton_right = ui->pushButton_Right;
    pushButton_initial = ui->pushButton_Initial;
    spinBox_time = ui->spinBox_time;

    comboBox_yawID = ui->comboBox_yawID;
    comboBox_pitchID = ui->comboBox_pitchID;

    spinBox_time->setMaximum(1000);
    spinBox_time->setMinimum(20);
    spinBox_time->setValue(100); //设置为100ms

    comboBox_yawID->setCurrentIndex(5); //设置默认舵机编号
    comboBox_pitchID->setCurrentIndex(2); //设置默认舵机编号

    label_yawPosition = ui->label_yawPosition;
    label_pitchPosition = ui->label_pitchPosition;

    yawPosition = 1500;
    pitchPosition = 1500;

    //Video
    label_video = ui->label_video;
    comboBox_videoResolution = ui->comboBox_videoResolution;
    comboBox_cameraNumber = ui->comboBox_cameraNumber;
    pushButton_videoCalibration = ui->pushButton_videoCalibration;
    pushButton_videoTracking = ui->pushButton_videoTracking;
    pushButton_openCamera = ui->pushButton_openCamera;
    pushButton_closeCamera = ui->pushButton_closeCamera;
    spinBox_videoFPS = ui->spinBox_videoFPS;
    pTimer = new QTimer();

    spinBox_videoFPS->setValue(30); //FPS then timerTick is 1000/30 ms
    spinBox_videoFPS->setMaximum(60);
    spinBox_videoFPS->setMinimum(10);

    QStringList cameraNumList;
    comboBox_cameraNumber->addItems(cameraNumList<<"0"<<"1"<<"2"<<"3"<<"4"
                                   <<"5"<<"6"<<"7"<<"8"<<"9");
    comboBox_videoResolution->setCurrentIndex(6); //1280*720
    comboBox_cameraNumber->setCurrentIndex(0);

    //open and close Serial Port
    connect(pushButton_openSerialPort,&QPushButton::clicked,this,openSerialPort);
    connect(pushButton_closeSerialPort,&QPushButton::clicked,this,closeSerialPort);

    //ServoMotor Move test
    connect(pushButton_up,&QPushButton::clicked,this,servoMotorUp);
    connect(pushButton_down,&QPushButton::clicked,this,servoMotorDown);
    connect(pushButton_left,&QPushButton::clicked,this,servoMotorLeft);
    connect(pushButton_right,&QPushButton::clicked,this,servoMotorRight);
    connect(pushButton_initial,&QPushButton::clicked,this,initialServoMotorPosition);

    //Video
    connect(pushButton_openCamera,&QPushButton::clicked,this,openCamera);
    connect(pushButton_closeCamera,&QPushButton::clicked,this,closeCamera);
    connect(pTimer,&QTimer::timeout,this,nextFrame);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openSerialPort()
{
    serialPortForServoMotor->openControlPort(getSerialPortInfo(),
                                             getBaudRate(),
                                             getDataBits(),
                                             getParity(),
                                             getStopBits(),
                                             getFlowControl());
}

void MainWindow::closeSerialPort()
{
    serialPortForServoMotor->closeControlPort();
    qDebug()<<"Com port closed";

}

void MainWindow::initialServoMotorPosition()
{
    uint8_t id;
    uint16_t time;

    id = comboBox_pitchID->currentIndex()+1;
    pitchPosition=1500;
    time = 1000; //ms
    if(serialPortForServoMotor->servoMotorMove(id,pitchPosition,time)) {
         label_pitchPosition->setText(QString::number(pitchPosition,10));
    }

    id = comboBox_yawID->currentIndex()+1;
    yawPosition=1500;
    time = 1000; //ms
    if(serialPortForServoMotor->servoMotorMove(id,yawPosition,time)) {
         label_yawPosition->setText(QString::number(yawPosition,10));
    }
}

bool MainWindow::initVideoCamera()
{
    cap.open(comboBox_cameraNumber->currentIndex());

    if( !cap.isOpened() )
    {
        qDebug() << "***Could not initialize capturing...***";
        qDebug() << "Current parameter's value: \n";
        return false;
    }
    qDebug()<<"Camera opened!";
    return true;
}


void MainWindow::openCamera()
{
    VideoResolution videoSize = getVideoResolution();
    frame = Mat::zeros(videoSize.height, videoSize.width, CV_8UC3);
    //cap.set(CV_CAP_PROP_FRAME_WIDTH,videoSize.width);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT,videoSize.height);
    //label_video->setFixedWidth(videoSize.width);
    //label_video->setFixedHeight(videoSize.height);

    if(!initVideoCamera()) {
        qDebug()<<"Open Camera failed!";
        return;
    }
    else {
        qDebug()<<spinBox_videoFPS->value();
        pTimer->setInterval(1000/spinBox_videoFPS->value());
        pTimer->start();
    }
}

void MainWindow::closeCamera()
{
    ;
}

void MainWindow::nextFrame()
{
    //采集数据
    cap>>frame;
    //处理数据

    //在界面上显示数据
    Mat2QImage(frame);
    label_video->setPixmap(QPixmap::fromImage(frameImage)); //暂时先用这个了，比较简单
}

void MainWindow::Mat2QImage(Mat cvImg)
{
    if(cvImg.channels()==3) //3 channels color image
    {
        cv::cvtColor(cvImg,cvImg,CV_BGR2RGB);
        frameImage =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols, cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_RGB888);
    }
    else if(cvImg.channels()==1) //grayscale image
    {
        frameImage =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols,cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_Indexed8);
    }
    else
    {
        frameImage =QImage((const unsigned char*)(cvImg.data),
                    cvImg.cols,cvImg.rows,
                    cvImg.cols*cvImg.channels(),
                    QImage::Format_RGB888);
    }
}

void MainWindow::servoMotorUp() //pitch
{
    uint8_t id = comboBox_pitchID->currentIndex()+1;
    pitchPosition-=5;
    uint16_t time = 100; //ms
    if(pitchPosition>500 && pitchPosition<2500) {
        if(serialPortForServoMotor->servoMotorMove(id,pitchPosition,time)) {
            label_pitchPosition->setText(QString::number(pitchPosition,10));
        }
        else{
            pitchPosition+=5;
        }
    }
}

void MainWindow::servoMotorDown() //pitch
{
    uint8_t id = comboBox_pitchID->currentIndex()+1;
    pitchPosition+=5;
    uint16_t time = 100; //ms
    if(pitchPosition>500 && pitchPosition<2500) {
        serialPortForServoMotor->servoMotorMove(id,pitchPosition,time);
        label_pitchPosition->setText(QString::number(pitchPosition,10));
    }
}

void MainWindow::servoMotorLeft() //yaw
{
    uint8_t id = comboBox_yawID->currentIndex()+1;
    yawPosition-=5;
    uint16_t time = 100; //ms
    if(yawPosition>500 && yawPosition<2500) {
        serialPortForServoMotor->servoMotorMove(id,yawPosition,time);
        label_yawPosition->setText(QString::number(yawPosition,10));
    }
}

void MainWindow::servoMotorRight() //yaw
{
    uint8_t id = comboBox_yawID->currentIndex()+1;
    yawPosition+=5;
    uint16_t time = 100; //ms
    if(yawPosition>500 && yawPosition<2500) {
        serialPortForServoMotor->servoMotorMove(id,yawPosition,time);
        label_yawPosition->setText(QString::number(yawPosition,10));
    }
}

const QSerialPortInfo& MainWindow::getSerialPortInfo()
{
    return list_SerialPortInfo.at(comboBox_SerialPort->currentIndex());
}

qint32 MainWindow::getBaudRate()
{
    return comboBox_baudRate->currentText().toInt();
}

QSerialPort::DataBits MainWindow::getDataBits()
{
    switch(comboBox_dataBits->currentText().toInt()) {
    case 5:
        return QSerialPort::Data5;
        break;
    case 6:
        return QSerialPort::Data6;
        break;
    case 7:
        return QSerialPort::Data7;
        break;
    case 8:
        return QSerialPort::Data8;
        break;
    default:
        return QSerialPort::UnknownDataBits;
        break;
    }

}

QSerialPort::Parity MainWindow::getParity()
{
    switch(comboBox_parity->currentIndex()) {
    case 0: //None
        return QSerialPort::NoParity;
        break;
    case 1: //Even
        return QSerialPort::EvenParity;
        break;
    case 2: //Odd
        return QSerialPort::OddParity;
        break;
    case 3: //Mark
        return QSerialPort::MarkParity;
        break;
    case 4: //Space
        return QSerialPort::SpaceParity;;
        break;
    default:
        return QSerialPort::UnknownParity;
        break;
    }
}

QSerialPort::StopBits MainWindow::getStopBits()
{
    switch(comboBox_stopBits->currentIndex()) {
    case 0: //1
        return QSerialPort::OneStop;
        break;
    case 1: //1.5
        return QSerialPort::OneAndHalfStop;
        break;
    case 2: //2
        return QSerialPort::TwoStop;
        break;
    default:
        return QSerialPort::UnknownStopBits;
        break;
    }
}

QSerialPort::FlowControl MainWindow::getFlowControl()
{
    switch(comboBox_flowControl->currentIndex()) {
    case 0: //None
        return QSerialPort::NoFlowControl;
        break;
    case 1: //RTS/CTS
        return QSerialPort::HardwareControl;
        break;
    case 2: //XON/XOFF
        return QSerialPort::SoftwareControl;
        break;
    default:
        return QSerialPort::UnknownFlowControl;
        break;
    }
}

VideoResolution MainWindow::getVideoResolution()
{
    VideoResolution videoSize;
    switch(comboBox_videoResolution->currentIndex()) {
    case 0: //160x120
        videoSize.width=160;
        videoSize.height=120;
        break;
    case 1: //320x240
        videoSize.width=320;
        videoSize.height=240;
        break;
    case 2: //424x240
        videoSize.width=424;
        videoSize.height=240;
        break;
    case 3: //640x360
        videoSize.width=640;
        videoSize.height=360;
        break;
    case 4: //800x448
        videoSize.width=800;
        videoSize.height=448;
        break;
    case 5: //960x544
        videoSize.width=960;
        videoSize.height=544;
        break;
    case 6: //1280x720
        videoSize.width=1280;
        videoSize.height=720;
        break;
    default:
        videoSize.width=1280;
        videoSize.height=720;
        break;
    }
    return videoSize;
}
