#ifndef SERIALPORTFORSERVOMOTOR_H
#define SERIALPORTFORSERVOMOTOR_H
#include <QIntegerForSize>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QtMath>

//发送部分的指令
#define FRAME_HEADER            0x55   //帧头
#define CMD_SERVO_MOVE          0x03   //舵机移动指令
#define CMD_ACTION_GROUP_RUN    0x06   //运行动作组指令
#define CMD_ACTION_GROUP_STOP   0x07   //停止动作组运行指令
#define CMD_ACTION_GROUP_SPEED  0x0B   //设置动作组运行速度指令
#define CMD_GET_BATTERY_VOLTAGE 0x0F   //获得电池电压指令

//接收部分的指令
#define BATTERY_VOLTAGE       0x0F  //电池电压
#define ACTION_GROUP_RUNNING  0x06  //动作组被运行
#define ACTION_GROUP_STOPPED  0x07  //动作组被停止
#define ACTION_GROUP_COMPLETE 0x08  //动作组完成

//类型定义
typedef quint8 uint8_t;
typedef quint16 uint16_t;


struct LobotServo {  //舵机ID和位置结构体
  uint8_t  ID;       //舵机ID
  uint16_t Position; //舵机数据
};

#define GET_LOW_BYTE(A) (uint8_t)((A))
//宏函数 获得A的低八位
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
//宏函数 获得A的高八位
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//宏函数 以A为高八位 B为低八位 合并为16位整形

class SerialPortForServoMotor
{
public:
    SerialPortForServoMotor();


    bool openControlPort(const QSerialPortInfo &info,qint32 boundRate,
                         QSerialPort::DataBits dataBits,
                         QSerialPort::Parity parity,
                         QSerialPort::StopBits stopBits,
                         QSerialPort::FlowControl flowControl);

    bool closeControlPort();

    bool servoMotorMove(uint8_t servoID,uint16_t position,uint16_t time);
    bool servoMotorPitchYawMove(uint16_t time,
                                uint8_t yawServoID,uint16_t yawPosition,
                                uint8_t pitchServoID,uint16_t pitchPosition);

private:
    QSerialPort serial;
    uint16_t angleToPosition(float angle);
};

#endif // SERIALPORTFORSERVOMOTOR_H
