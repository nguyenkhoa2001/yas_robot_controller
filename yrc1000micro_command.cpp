#include "yrc1000micro_command.h"
#include <QVector>
#include <QDebug>

YRC1000micro_command::YRC1000micro_command(QObject *parent) : QObject(parent)
{
//    Init header
    header_to_send.append('\x59');
    header_to_send.append('\x45');
    header_to_send.append('\x52');
    header_to_send.append('\x43');
    header_to_send.append('\x20');
    header_to_send.append('\x00');
    header_to_send.append('\x04');
    header_to_send.append('\x00');
    header_to_send.append('\x03');
    header_to_send.append('\x01');
    header_to_send.append('\x00');
    header_to_send.append('\x01');
    header_to_send.append('\x00');
    header_to_send.append('\x00');
    header_to_send.append('\x00');
    header_to_send.append('\x00');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x83');
    header_to_send.append('\x00');
    header_to_send.append('\x02');
    header_to_send.append('\x00');
    header_to_send.append('\x01');
    header_to_send.append('\x10');
    header_to_send.append('\x00');
    header_to_send.append('\x00');
//  Init data
    data_to_send.append('\x00');
    data_to_send.append('\x00');
    data_to_send.append('\x00');
    data_to_send.append('\x00');
//  Init move data
    for(int i=0;i<104;i++){
        move_cartesian_data.append('\x00');
    }
    move_cartesian_data[DATA_MOVE_ROBOT_NUMBER] = '\x01';
    move_cartesian_data[DATA_MOVE_USER_COORDINATE] = '\x01';

}

YRC1000micro_command::~YRC1000micro_command(){

}

QByteArray YRC1000micro_command::setServoOn(quint8 request_id_index){
    QByteArray cmd = header_to_send + data_to_send;
    cmd[CMD_REQUEST_ID] = request_id_index;
    cmd[CMD_ID_ADDRESS] = CMD_ID_SERVO_ON;
    cmd[CMD_INSTANCE] = CMD_HEADER_SERVO_INSTANCE_ON;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_SERVO_ATTRIBUTE;
    cmd[CMD_SERVICE] = CMD_HEADER_SERVO_SERVICE;

    cmd[DATA_BYTE0] = CMD_DATA_SERVO_ON;
    //qDebug() << cmd[4];
    return cmd;
}

QByteArray YRC1000micro_command::setServoOff(quint8 request_id_index){
    QByteArray cmd = header_to_send + data_to_send;
    cmd[CMD_REQUEST_ID] = request_id_index;
    cmd[CMD_ID_ADDRESS] = CMD_ID_SERVO_ON;
    cmd[CMD_INSTANCE] = CMD_HEADER_SERVO_INSTANCE_ON;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_SERVO_ATTRIBUTE;
    cmd[CMD_SERVICE] = CMD_HEADER_SERVO_SERVICE;

    cmd[DATA_BYTE0] = CMD_DATA_SERVO_OFF;
    return cmd;
}

QByteArray YRC1000micro_command::readRobotPosition(quint8 request_id_index){
    QByteArray cmd = header_to_send;
    cmd[CMD_REQUEST_ID] = request_id_index;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_READ_POS_DATA_SIZE;
    cmd[CMD_ID_ADDRESS] = CMD_ID_READ_ROBOT_POS;
    cmd[CMD_INSTANCE] = CMD_HEADER_READ_POS_INSTANCE_CARTESIAN;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_READ_POS_ATTRIBUTE_ALL;
    cmd[CMD_SERVICE] = CMD_HEADER_READ_POS_SERVICE_ALL;
    return cmd;
}

QByteArray YRC1000micro_command::readRobotPulse(quint8 request_id_index){
    QByteArray cmd = header_to_send;
    cmd[CMD_REQUEST_ID] = request_id_index;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_READ_POS_DATA_SIZE;
    cmd[CMD_ID_ADDRESS] = CMD_ID_READ_ROBOT_POS;
    cmd[CMD_INSTANCE] = CMD_HEADER_READ_POS_INSTANCE_PULSE;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_READ_POS_ATTRIBUTE_ALL;
    cmd[CMD_SERVICE] = CMD_HEADER_READ_POS_SERVICE_ALL;
    return cmd;
}

void YRC1000micro_command::initRobot(quint8 robot_number){
    move_cartesian_data[DATA_MOVE_ROBOT_NUMBER] = robot_number;
}

QByteArray YRC1000micro_command::setRobotPositionCartesian(quint8 request_id_index, quint8 coordinate,
                      quint8 move_type,quint8 speed_type, double speed, QVector<double>* position){
    QByteArray cmd = header_to_send + move_cartesian_data;
    cmd[CMD_REQUEST_ID] = request_id_index;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_MOVE_CARTESIAN_DATA_SIZE;
    cmd[CMD_ID_ADDRESS] = CMD_ID_MOVE_ROBOT_CARTESIAN;
    cmd[CMD_INSTANCE] = move_type;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_MOVE_ATTRIBUTE;
    cmd[CMD_SERVICE] = CMD_HEADER_MOVE_SERVICE_ALL;

    if(speed < 0)
        speed = 0;
    quint32 speed_u= (quint32)(speed*10);
    quint32 x_u = (quint32)(position->at(0)*1000);
    quint32 y_u = (quint32)(position->at(1)*1000);
    quint32 z_u = (quint32)(position->at(2)*1000);
    quint32 roll_u = (quint32)(position->at(3)*10000);
    quint32 pitch_u = (quint32)(position->at(4)*10000);
    quint32 yaw_u = (quint32)(position->at(5)*10000);

    cmd[DATA_MOVE_SPEED_TYPE+HEADER_SIZE] = speed_type;
    cmd[DATA_MOVE_COORDINATE+HEADER_SIZE]= coordinate;

    cmd[DATA_MOVE_SPEED+HEADER_SIZE+3]= (quint8)(speed_u>>24);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE+2]= (quint8)(speed_u>>16);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE+1]= (quint8)(speed_u>>8);
    cmd[DATA_MOVE_SPEED+HEADER_SIZE]= (quint8)(speed_u);

    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+3]= (quint8)(x_u>>24);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+2]= (quint8)(x_u>>16);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE+1]= (quint8)(x_u>>8);
    cmd[DATA_MOVE_X_CARTESIAN+HEADER_SIZE]= (quint8)(x_u);

    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+3]= (quint8)(y_u>>24);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+2]= (quint8)(y_u>>16);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE+1]= (quint8)(y_u>>8);
    cmd[DATA_MOVE_Y_CARTISIAN+HEADER_SIZE]= (quint8)(y_u);

    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+3]= (quint8)(z_u>>24);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+2]= (quint8)(z_u>>16);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE+1]= (quint8)(z_u>>8);
    cmd[DATA_MOVE_Z_CARTESIAN+HEADER_SIZE]= (quint8)(z_u);

    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+3]= (quint8)(roll_u>>24);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+2]= (quint8)(roll_u>>16);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE+1]= (quint8)(roll_u>>8);
    cmd[DATA_MOVE_ROLL_CARTESIAN+HEADER_SIZE]= (quint8)(roll_u);

    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+3]= (quint8)(pitch_u>>24);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+2]= (quint8)(pitch_u>>16);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE+1]= (quint8)(pitch_u>>8);
    cmd[DATA_MOVE_PITCH_CARTESIAN+HEADER_SIZE]= (quint8)(pitch_u);

    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+3]= (quint8)(yaw_u>>24);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+2]= (quint8)(yaw_u>>16);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+1]= (quint8)(yaw_u>>8);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE]= (quint8)(yaw_u);
    return cmd;
}

QByteArray YRC1000micro_command::readStatus(quint8 request_id_index){
    QByteArray cmd = header_to_send;
    cmd[CMD_REQUEST_ID] = request_id_index;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_STATUS_DATA_SIZE;
    cmd[CMD_ID_ADDRESS] = CMD_ID_STATUS_READING;
    cmd[CMD_INSTANCE] = CMD_HEADER_STATUS_INSTANCE;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_STATUS_ATTRIBUTE_ALL;
    cmd[CMD_SERVICE] = CMD_HEADER_STATUS_SERVICE_ALL;
    return cmd;
}

QByteArray YRC1000micro_command::loadJob(quint8 request_id_index, quint8 ack, quint32 block_num, QString data){
    QByteArray cmd = header_to_send;
    if(data.size() > 497){
        QByteArray data_size("ERROR");
        qDebug() << "Data size error";
        return data_size;
    }
    QByteArray data_br = data.toUtf8();
    cmd[CMD_REQUEST_ID] = request_id_index;
    cmd[CMD_ACK] = ack;
    cmd[CMD_PROCESS_DIVTION] = CMD_HEADER_PROCESS_FILE_CONTROL;

    cmd[CMD_DATA_SIZE+1] = (quint8)(data.size()>>8);
    cmd[CMD_DATA_SIZE] = (quint8)(data.size());

    cmd[CMD_BLOCK_NUM+3] = (quint8)(block_num>>24);
    cmd[CMD_BLOCK_NUM+2] = (quint8)(block_num>>16);
    cmd[CMD_BLOCK_NUM+1] = (quint8)(block_num>>8);
    cmd[CMD_BLOCK_NUM] = (quint8)(block_num);

    cmd[CMD_ID_ADDRESS] = CMD_ID_LOAD_SAVE_JOB;
    cmd[CMD_INSTANCE] = CMD_HEADER_LOAD_JOB_INSTANCE;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_LOAD_JOB_ATTRIBUTE;
    cmd[CMD_SERVICE] = CMD_HEADER_LOAD_JOB_SERVICE;
    for(int i=0;i<data.size();i++){
        cmd.append(data_br[i]);
    }
    qDebug() << "Load file cmd: " << cmd << Qt::endl;
    return cmd;
}

QByteArray YRC1000micro_command::saveJob(quint8 request_id_index, quint8 ack, quint32 block_num, QString data){
    QByteArray cmd = header_to_send;
    QByteArray data_br = data.toUtf8();
    qDebug() << "Data br: " << data_br;

    int data_size = data.size();

    if(data_size > 497){
        QByteArray data_size("ERROR");
        return data_size;
    }
    if(data == "NODATA"){
      data_size = 0;
    }
    else{
      for(int i=0;i<data.size();i++){
          cmd.append(data_br[i]);
      }
    }

    qDebug() << "data_size: " << data_size;

    cmd[CMD_REQUEST_ID] = request_id_index;
    cmd[CMD_PROCESS_DIVTION] = CMD_HEADER_PROCESS_FILE_CONTROL;
    cmd[CMD_ACK] = ack;
    cmd[CMD_DATA_SIZE+1] = (quint8)(data_size>>8);
    cmd[CMD_DATA_SIZE] = (quint8)(data_size);

    cmd[CMD_BLOCK_NUM+3] = (quint8)(block_num>>24);
    cmd[CMD_BLOCK_NUM+2] = (quint8)(block_num>>16);
    cmd[CMD_BLOCK_NUM+1] = (quint8)(block_num>>8);
    cmd[CMD_BLOCK_NUM] = (quint8)(block_num);

    cmd[CMD_ID_ADDRESS] = CMD_ID_LOAD_SAVE_JOB;
    cmd[CMD_INSTANCE] = CMD_HEADER_SAVE_JOB_INSTANCE;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_SAVE_JOB_ATTRIBUTE;
    cmd[CMD_SERVICE] = CMD_HEADER_SAVE_JOB_SERVICE;

    qDebug() << "cmd: " << cmd;
    return cmd;
}


