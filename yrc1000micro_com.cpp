#include "yrc1000micro_com.h"
#include "yrc1000micro_command.h"
#include <iostream>
#include <QMessageBox>
#include "QtMath"
#include <QMatrix4x4>

YRC1000micro_com::YRC1000micro_com(QObject *parent) : QObject(parent)
{
    request_id_index = 0;
    response_id_index = 0;
    status_code = 0;
    robot_position.append(0);
    robot_position.append(0);
    robot_position.append(0);
    robot_position.append(0);
    robot_position.append(0);
    robot_position.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
    robot_pulse.append(0);
    connect(&udp_server, SIGNAL(dataRecieveSignal()), this, SLOT(YRC1000microDataCallback()));
}

YRC1000micro_com::~YRC1000micro_com(){
    udp_server.disconnect();
}

bool YRC1000micro_com::YRC1000microConnect(){
    bool connect_satus = udp_server.udpConnect(udp_address,udp_port);
    return connect_satus;
}

void YRC1000micro_com::YRC1000microDisConnect(){
    udp_server.udpDisConnect();
}

bool YRC1000micro_com::YRC1000microFileTransferConnect(){
    bool connect_satus = udp_file_control.udpConnect(udp_address,udp_file_port);
    return connect_satus;
}

void YRC1000micro_com::YRC1000microFileTransferDisConnect(){
    udp_file_control.udpDisConnect();
}

void YRC1000micro_com::YRC1000microSetConnection(QHostAddress address,quint16 port,quint16 file_port){
    udp_address = address;
    udp_port = port;
    udp_file_port = file_port;
}

void YRC1000micro_com::YRC1000microDataCallback(){
    QByteArray data = udp_server.getUdpData();
//    qDebug() << "Received Data: " << data;

    quint8 res_id_index = (quint8)data[CMD_REQUEST_ID];
    char response_id = request_code[res_id_index];
//    qDebug() << "response_id: " << response_id;

    if(response_id == CMD_ID_READ_ROBOT_POS){
        if (read_position_type == READ_POSITION_XYZ)
            YRC1000microReadPositionResponse(data);
        else if (read_position_type == READ_POSITION_PULSE)
            YRC1000microReadPulseResponse(data);
    }
    else if(response_id == CMD_ID_STATUS_READING){
        YRC1000microReadStatusResponse(data);
    }
    else if(response_id == CMD_ID_SAVE_JOB_TO_PC){
        YRC1000microSaveJobResponse(data);
    }
    else if(response_id == CMD_ID_LOAD_JOB_TO_YRC){
        load_file_block_num ++;
        qDebug() << "load_file_block_num: " << load_file_block_num;
        YRC1000microLoadJobResponse(data,load_file_block_num);
    }
    emit dataUIRecieveSiUIgnal();
}

void YRC1000micro_com::YRC1000microOnServo(){
    QByteArray data = yrc1000micro_command.setServoOn(request_id_index);
    udp_server.sendData(udp_address,udp_port,data);
    request_code[request_id_index] = CMD_ID_SERVO_ON;
    request_id_index ++;
}

void YRC1000micro_com::YRC1000microOffServo(){
    QByteArray data = yrc1000micro_command.setServoOff(request_id_index);
    udp_server.sendData(udp_address,udp_port,data);
    request_code[request_id_index] = CMD_ID_SERVO_ON;
    request_id_index ++;
}

void YRC1000micro_com::YRC1000microReadPosition(){
    QByteArray data = yrc1000micro_command.readRobotPosition(request_id_index);
    udp_server.sendData(udp_address,udp_port,data);
    request_code[request_id_index] = CMD_ID_READ_ROBOT_POS;
    read_position_type = READ_POSITION_XYZ;
    request_id_index ++;
}

void YRC1000micro_com::YRC1000microReadPulse() {
    QByteArray data = yrc1000micro_command.readRobotPulse(request_id_index);
    udp_server.sendData(udp_address,udp_port,data);
    request_code[request_id_index] = CMD_ID_READ_ROBOT_POS;
    read_position_type = READ_POSITION_PULSE;
    request_id_index ++;
}


void YRC1000micro_com::YRC1000microReadPositionResponse(QByteArray data){
    qint32 x_axis = 0;
    qint32 y_axis = 0;
    qint32 z_axis = 0;
    qint32 roll_angle = 0;
    qint32 pitch_angle = 0;
    qint32 yaw_angle = 0;
    x_axis = ((quint32)(quint8)data[DATA_X_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_X_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_X_AXIS+1])<<8 | (quint32)(quint8)data[DATA_X_AXIS];

    y_axis = ((quint32)(quint8)data[DATA_Y_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Y_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_Y_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Y_AXIS];

    z_axis = ((quint32)(quint8)data[DATA_Z_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Z_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_Z_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Z_AXIS];

    roll_angle = ((quint32)(quint8)data[DATA_ROLL_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_ROLL_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_ROLL_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_ROLL_ANGLE];

    pitch_angle = ((quint32)(quint8)data[DATA_PITCH_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_PITCH_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_PITCH_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_PITCH_ANGLE];

    yaw_angle = ((quint32)(quint8)data[DATA_YAW_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_YAW_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_YAW_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_YAW_ANGLE];

//    QVector<double> pos;
    robot_position[0]=(x_axis/1000.0);
    robot_position[1]=(y_axis/1000.0);
    robot_position[2]=(z_axis/1000.0);
    robot_position[3]=(roll_angle/10000.0);
    robot_position[4]=(pitch_angle/10000.0);
    robot_position[5]=(yaw_angle/10000.0);
}

void YRC1000micro_com::YRC1000microReadPulseResponse(QByteArray data) {
    qint32 j1 = 0;
    qint32 j2 = 0;
    qint32 j3 = 0;
    qint32 j4 = 0;
    qint32 j5 = 0;
    qint32 j6 = 0;
    j1 = ((quint32)(quint8)data[DATA_X_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_X_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_X_AXIS+1])<<8 | (quint32)(quint8)data[DATA_X_AXIS];

    j2 = ((quint32)(quint8)data[DATA_Y_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Y_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_Y_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Y_AXIS];

    j3 = ((quint32)(quint8)data[DATA_Z_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Z_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_Z_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Z_AXIS];

    j4 = ((quint32)(quint8)data[DATA_ROLL_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_ROLL_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_ROLL_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_ROLL_ANGLE];

    j5 = ((quint32)(quint8)data[DATA_PITCH_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_PITCH_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_PITCH_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_PITCH_ANGLE];

    j6 = ((quint32)(quint8)data[DATA_YAW_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_YAW_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_YAW_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_YAW_ANGLE];

//    QVector<double> pos;
    robot_pulse[0]=j1;
    robot_pulse[1]=j2;
    robot_pulse[2]=j3;
    robot_pulse[3]=j4;
    robot_pulse[4]=j5;
    robot_pulse[5]=j6;
}

QVector<double> YRC1000micro_com::updateRobotPosition(){
    return robot_position;
}

QVector<double> YRC1000micro_com::updateRobotPulse(){
    return robot_pulse;
}

void YRC1000micro_com::YRC1000microMoveCartesian(quint8 coordinate,quint8 move_type, quint8 speed_type,
                                                 double speed, QVector<double>* position){
    QByteArray data = yrc1000micro_command.setRobotPositionCartesian(request_id_index,
                                                 coordinate,move_type,speed_type,speed,position);
    udp_server.sendData(udp_address,udp_port,data);
    request_code[request_id_index] = CMD_ID_MOVE_ROBOT_CARTESIAN;
    request_id_index ++;
}

void YRC1000micro_com::YRC1000microReadStatus(){
    QByteArray data = yrc1000micro_command.readStatus(request_id_index);
    udp_server.sendData(udp_address,udp_port,data);
    request_code[request_id_index] = CMD_ID_STATUS_READING;
    request_id_index ++;
}

void YRC1000micro_com::YRC1000microReadStatusResponse(QByteArray data){
    status_code = 0;
    status_code = status_code | (((quint16)(quint8)data[RES_STATUS_READING_DATA_1+HEADER_SIZE]) << 8);
    status_code = status_code | ((quint16)(quint8)data[RES_STATUS_READING_DATA_1+HEADER_SIZE]);
//    qDebug() << "status_code: " << status_code;
}

quint16 YRC1000micro_com::updateRobotStatus(){
    return status_code;
}

void YRC1000micro_com::YRC1000microSaveJob(quint32 block_num, QString send_data){
    request_id_index ++;
    QByteArray data = yrc1000micro_command.saveJob(request_id_index,CMD_HEADER_ACK_REQUEST,block_num,send_data);
    udp_server.sendData(udp_address,udp_port,data);
    request_code[request_id_index] = CMD_ID_SAVE_JOB_TO_PC;
}

quint32 YRC1000micro_com::YRC1000microSaveJobResponse(QByteArray data){
    QByteArray data_rev;
    for(int i=HEADER_SIZE;i<data.size()-1;i++)
      data_rev.append(data[i]);
//    qDebug() << "Data revieve: " << data_rev;
    QString filename = "Data.txt";
    QFile file(filename);
    if (file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        QTextStream stream(&file);
        stream << data_rev << Qt::endl;
        file.flush();
        file.close();
        qDebug() << "save to txt";
    }
    quint32 block_num=0;
    block_num |= ((quint32)(quint8)data[CMD_BLOCK_NUM]);
    block_num |= ((quint32)(quint8)data[CMD_BLOCK_NUM+1])<<8;
    block_num |= ((quint32)(quint8)data[CMD_BLOCK_NUM+2])<<16;
    block_num |= ((quint32)(quint8)data[CMD_BLOCK_NUM+3])<<24;

    qDebug() << "block_num: " << block_num;
    QByteArray send_data = yrc1000micro_command.saveJob(request_id_index,
                                 CMD_HEADER_ACK_NOT_REQUEST,block_num,"NODATA");
    udp_server.sendData(udp_address,udp_port,send_data);

    return 1;
}

void YRC1000micro_com::YRC1000microLoadJob(QString send_data){
    request_id_index ++;
//    Create socket
//    YRC1000microFileTransferConnect();
    load_file_block_num = 0;
    QByteArray data = yrc1000micro_command.loadJob(request_id_index,CMD_HEADER_ACK_REQUEST,load_file_block_num,send_data);
    udp_server.sendData(udp_address,udp_port,data);
    request_code[request_id_index] = CMD_ID_LOAD_JOB_TO_YRC;
    QFile load_file("Data.txt");
    if(load_file.open(QFile::ReadOnly | QFile::Text)){
        QTextStream load(&load_file);
        str_load_file = load.readAll();
        //qDebug() << "Read file: " << str_load_file << endl;
        load_file.flush();
        load_file.close();
        load_data_status = true;
    }
}

quint32 YRC1000micro_com::YRC1000microLoadJobResponse(QByteArray data, quint32 block_num){
    if(data[RES_STATUS] != '\x00'){
       //qDebug() << "Load file command error" << endl;
       return 0;
    }
    QString str_data = QString(data);
    if(load_data_status){
      qDebug() << "Load file ......";
      if(str_load_file.size()>400){
          qDebug() << "Load file > 400";
          QString temp_str_load_file = str_load_file.left(400);
          str_load_file = str_load_file.right(str_load_file.size()-400);
          QByteArray send_data = yrc1000micro_command.loadJob(request_id_index,
                                       CMD_HEADER_ACK_NOT_REQUEST,block_num,temp_str_load_file);
          udp_server.sendData(udp_address,udp_port,send_data);
          //qDebug() << "Load to be load: " << temp_str_load_file << endl;
      }
      else {
          //qDebug() << "Load file < 400" ;
          QByteArray send_data = yrc1000micro_command.loadJob(request_id_index,
                                       CMD_HEADER_ACK_NOT_REQUEST,block_num+2147483648,str_load_file);
          udp_server.sendData(udp_address,udp_port,send_data);
          //qDebug() << "Load to be load: " << str_load_file << endl;
          load_data_status = false;
      }
    }
    else {
          //qDebug() << "Close socket" << endl;
          YRC1000microDisConnect();
    }

    return 1;
}
void YRC1000micro_com::foward_kinematic(QVector<double>*theta, QVector<double>*output_cartesian)
{
    QVector<double> a(6, 0);
    QVector<double> d(6, 0);
    QVector<double> cv_theta(6, 0);
    QVector<double> alpha(6, 0);

    a[0] = 20;
    a[1] = 165;
    d[3] = -165;
    d[5] = -40;

    cv_theta[0] = theta->at(0);
    cv_theta[1] = theta->at(1) - M_PI/2;
    cv_theta[2] = theta->at(2);
    cv_theta[3] = theta->at(3);
    cv_theta[4] = theta->at(4) - M_PI/2;
    cv_theta[5] = theta->at(5);

    alpha[0] = -M_PI/2;
    alpha[1] = -M_PI;
    alpha[2] = -M_PI/2;
    alpha[3] = M_PI/2;
    alpha[4] = -M_PI/2;
    alpha[5] = M_PI;
    QVector<QMatrix4x4> A(6);
    for(int i = 0; i <= 5; i++){
        A[i] = QMatrix4x4(cos(cv_theta[i]) , -sin(cv_theta[i])*cos(alpha[i]) , sin(cv_theta[i])*sin(alpha[i])  , a[i]*cos(cv_theta[i]) ,
                          sin(cv_theta[i]) , cos(cv_theta[i])*cos(alpha[i])  , -cos(cv_theta[i])*sin(alpha[i]) , a[i]*sin(cv_theta[i]) ,
                          0                , sin(alpha[i])                   , cos(alpha[i])                   , d[i]                ,
                          0                , 0                               , 0                               , 1);
    }
    QMatrix4x4 T = A[0]*A[1]*A[2]*A[3]*A[4]*A[5];
    (*output_cartesian)[0] = T(0, 3);
    (*output_cartesian)[1] = T(1, 3);
    (*output_cartesian)[2] = T(2, 3);
    double sy = sqrt(pow(T(0,0), 2) + pow(T(1, 0), 2));
    if(sy >= 1e-6){
        (*output_cartesian)[3] = qAtan2(T(2,1),T(2,2));
        (*output_cartesian)[4] = qAtan2(-T(2,0),sy);
        (*output_cartesian)[5] = qAtan2(T(1,0),T(0,0));
    }else{
        (*output_cartesian)[3] = qAtan2(-T(1,2),T(1,1));
        (*output_cartesian)[4] = qAtan2(-T(2,0),sy);
        (*output_cartesian)[5] = 0;
    }
}
