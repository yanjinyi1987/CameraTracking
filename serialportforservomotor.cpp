#include "serialportforservomotor.h"

SerialPortForServoMotor::SerialPortForServoMotor()
{

}

bool SerialPortForServoMotor::openControlPort(const QSerialPortInfo &info,qint32 boundRate,
                                              QSerialPort::DataBits dataBits,
                                              QSerialPort::Parity parity,
                                              QSerialPort::StopBits stopBits,
                                              QSerialPort::FlowControl flowControl
                                              )
{
    qDebug()<<boundRate
            <<dataBits
            <<parity
            <<stopBits
            <<flowControl;

    serial.setPort(info);
    if(serial.open(QIODevice::ReadWrite)) {

        if(
        serial.setBaudRate(boundRate) &&
        serial.setDataBits(dataBits) &&
        serial.setParity(parity) &&
        serial.setStopBits(stopBits) &&
        serial.setFlowControl(flowControl)) {
            qDebug()<<"Open and Set SerialPortSuccess";
            return true;
        }
        else {
            qDebug()<<"Set SerialPortError";
            return false;
        }
    }
    else {
        qDebug()<<"Open SerialPortError";
        return false;
    }
}

bool SerialPortForServoMotor::closeControlPort()
{
    serial.close();
    return true;
}

bool SerialPortForServoMotor::servoMotorMove(uint8_t servoID, uint16_t position, uint16_t time)
//time ms
{
    uint8_t buf[11];
    if(servoID > 6 || !(time>0)) { //舵机ID不能大于6
        return false;
    }
    buf[0] = FRAME_HEADER;
    buf[1] = FRAME_HEADER;
    buf[2] = 8; //data length = 要控制舵机数目*3+5
    buf[3] = CMD_SERVO_MOVE;
    buf[4] = 1; //要控制的舵机数目
    buf[5] = GET_LOW_BYTE(time);
    buf[6] = GET_HIGH_BYTE(time);            //填充时间的高八位
    buf[7] = servoID;                        //舵机ID
    buf[8] = GET_LOW_BYTE(position);         //填充目标位置的低八位
    buf[9] = GET_HIGH_BYTE(position);        //填充目标位置的高八位

    qDebug()<<QString::number(buf[0],16)
            <<QString::number(buf[1],16)
            <<QString::number(buf[2],16)
            <<QString::number(buf[3],16)
            <<QString::number(buf[4],16)
            <<QString::number(buf[5],16)
            <<QString::number(buf[6],16)
            <<QString::number(buf[7],16)
            <<QString::number(buf[8],16)
            <<QString::number(buf[9],16);
    if(serial.write((const char *)buf,10)!=10) { //发送帧，长度为数据长度+两个字节的帧头
        qDebug()<<"Move failed!";
        return false;
    }
    else {
        qDebug()<<"Move success!";
        return true;
    }
}

bool SerialPortForServoMotor::servoMotorPitchYawMove(uint16_t time,
                                                     uint8_t yawServoID, uint16_t yawPosition,
                                                     uint8_t pitchServoID, uint16_t pitchPosition)
//time ms
{
    uint8_t buf[13];
    if(yawServoID > 6 || pitchServoID>6 || !(time>0)) { //舵机ID不能大于6
        return false;
    }
    buf[0] = FRAME_HEADER;
    buf[1] = FRAME_HEADER;
    buf[2] = 11; //data length = 要控制舵机数目*3+5
    buf[3] = CMD_SERVO_MOVE;
    buf[4] = 2; //要控制的舵机数目
    buf[5] = GET_LOW_BYTE(time);
    buf[6] = GET_HIGH_BYTE(time);            //填充时间的高八位

    buf[7] = yawServoID;                        //Yaw 舵机ID
    buf[8] = GET_LOW_BYTE(yawPosition);         //填充目标位置的低八位
    buf[9] = GET_HIGH_BYTE(yawPosition);        //填充目标位置的高八位

    buf[10] = pitchServoID;                        //Pitch 舵机ID
    buf[11] = GET_LOW_BYTE(pitchPosition);         //填充目标位置的低八位
    buf[12] = GET_HIGH_BYTE(pitchPosition);        //填充目标位置的高八位

    if(serial.write((const char *)buf,13)!=13) { //发送帧，长度为数据长度+两个字节的帧头
        qDebug()<<"Move failed!";
        return false;
    }
    else {
        qDebug()<<"Move success!";
        return true;
    }
}

uint16_t SerialPortForServoMotor::angleToPosition(float angle) //<-90<angle<90
{
    return qRound(angle/0.45)*5+1500; //以5为position的移动单元
}
