#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QComboBox>
#include <QList>
#include <QtSerialPort/QSerialPortInfo>
#include <QSerialPort>
#include <QPushButton>
#include <QLabel>
#include <QSpinBox>
#include <QWidget>

#include <QDebug>
#include "serialportforservomotor.h"

//openCV
#include "opencv2/core/utility.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

namespace Ui {
class MainWindow;
}

using namespace cv;

struct VideoResolution {
    int width;
    int height;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

    QComboBox *comboBox_SerialPort;
    QComboBox *comboBox_baudRate;
    QComboBox *comboBox_dataBits;
    QComboBox *comboBox_parity;
    QComboBox *comboBox_stopBits;
    QComboBox *comboBox_flowControl;

    QPushButton *pushButton_openSerialPort;
    QPushButton *pushButton_closeSerialPort;

    //Serial Port Info
    QList<QSerialPortInfo> list_SerialPortInfo;
    const QSerialPortInfo& getSerialPortInfo();
    qint32 getBaudRate();
    QSerialPort::DataBits getDataBits();
    QSerialPort::Parity getParity();
    QSerialPort::StopBits getStopBits();
    QSerialPort::FlowControl getFlowControl();
    SerialPortForServoMotor *serialPortForServoMotor;

    //ServoMotor Move test
    QPushButton *pushButton_up;
    QPushButton *pushButton_down;
    QPushButton *pushButton_left;
    QPushButton *pushButton_right;
    QPushButton *pushButton_initial;
    QSpinBox *spinBox_time;

    QComboBox *comboBox_yawID;
    QComboBox *comboBox_pitchID;

    QLabel *label_yawPosition;
    QLabel *label_pitchPosition;

    uint16_t yawPosition;
    uint16_t pitchPosition;

    //Video part
    QWidget *widget_video;
    QComboBox *comboBox_videoResolution;
    QComboBox *comboBox_cameraNumber;
    QPushButton *pushButton_videoCalibration;
    QPushButton *pushButton_videoTracking;

    VideoCapture cap;
    VideoResolution getVideoResolution();
    bool initVideoCamera();


private slots:
    void openSerialPort();
    void closeSerialPort();

    //ServoMotor Move test
    void servoMotorUp();
    void servoMotorDown();
    void servoMotorLeft();
    void servoMotorRight();
    void initialServoMotorPosition();

};

#endif // MAINWINDOW_H
