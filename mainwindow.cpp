#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QMessageBox>
#include <QThread>
#include <QButtonGroup>
#include <QCheckBox>
#include <QString>
#include <iostream>
#include <qmath.h>
#include <controller.h>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->btn_connect->setAutoFillBackground(true);
    pal_green.setColor(QPalette::Button,QColor(Qt::green));
    pal_red.setColor(QPalette::Button,QColor(Qt::red));

    connect(&yrc100micro_com, SIGNAL(dataUIRecieveSiUIgnal()), this, SLOT(updateUICallback()));

    timer = new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timerCallback()));

    joggingTimer = new QTimer(this);
    connect(joggingTimer, SIGNAL(timeout()), this, SLOT(jogging_timerCallback()));

    impedanceTimer = new QTimer(this);
    connect(impedanceTimer, SIGNAL(timeout()), this, SLOT(updateDisplacement()));

    ui->StopJoggingBtn->setEnabled(false);
    ui->ImStopBtn->setCheckable(false);
    ui->releaseBtn->setCheckable(false);

    coor_type.addButton(ui->rbtn_xyz, 1);
    coor_type.addButton(ui->rbtn_rpy, 2);
    coor_type.setExclusive(true);
    ui->rbtn_xyz->setChecked(true);

    move_type.addButton(ui->rbtn_linkAbs, 1);
    move_type.addButton(ui->rbtn_lineAbs, 2);
    move_type.addButton(ui->rbtn_LineInc, 3);
    move_type.setExclusive(true);
    ui->rbtn_linkAbs->setChecked(true);

    autoGetPos = false;

    ui->ForceXSlider->setValue(50);
    ui->ForceYSlider->setValue(50);
    ui->ForceZSlider->setValue(50);

    im_controller = new Controller();
    //map twelve buttons to one slot
    connect(ui->t1_inc, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t2_inc, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t3_inc, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t4_inc, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t5_inc, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t6_inc, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t1_dec, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t2_dec, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t3_dec, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t4_dec, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t5_dec, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));
    connect(ui->t6_dec, SIGNAL(clicked()), this, SLOT(inc_dec_bt_clicked()));

    connect(ui->ForceXSlider, SIGNAL(valueChanged(int)), this, SLOT(currentForceX(int)));
    connect(ui->ForceYSlider, SIGNAL(valueChanged(int)), this, SLOT(currentForceY(int)));
    connect(ui->ForceZSlider, SIGNAL(valueChanged(int)), this, SLOT(currentForceZ(int)));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btn_connect_clicked()
{
    if(ui->btn_connect->text() == "Connect"){
//      UDP connect
        QHostAddress udp_address;
        quint16 udp_port;
        quint16 udp_file_port;
        QString ip_string = ui->txt_ip->text();
        QStringList ip_list = ip_string.split(".");
        quint32 ip_int32 = (ip_list.at(0).toUInt() << 24) | (ip_list.at(1).toUInt() << 16)
                            | (ip_list.at(2).toUInt() << 8) | ip_list.at(3).toUInt();
        udp_address.setAddress(ip_int32);
        udp_port = ui->txt_port->text().toUShort();
        udp_file_port = ui->txt_filePort->text().toUShort();

        yrc100micro_com.YRC1000microSetConnection(udp_address,udp_port,udp_file_port);
        bool connection_satus = yrc100micro_com.YRC1000microConnect();
        if(connection_satus == false){
            QMessageBox::warning(this,"UDP Connection","Can not connect to udp address");
            return;
        }
        ui->btn_connect->setText("Disconnect");
        ui->btn_connect->setPalette(pal_green);

//        timer->start(100);
    }
    else if(ui->btn_connect->text() == "Disconnect"){
        ui->btn_connect->setText("Connect");
        ui->btn_connect->setPalette(pal_red);
        yrc100micro_com.YRC1000microDisConnect();
    }
}

void MainWindow::on_btn_servo_clicked()
{
    if(ui->btn_servo->text() == "Servo On"){
        yrc100micro_com.YRC1000microOnServo();
        ui->btn_servo->setText("Servo Off");
    }
    else if(ui->btn_servo->text() == "Servo Off"){
        yrc100micro_com.YRC1000microOffServo();
        ui->btn_servo->setText("Servo On");
    }
}

void MainWindow::on_btn_get_pos_clicked()
{
    if(autoGetPos)
    {
        timer->start(50);
    }else
    yrc100micro_com.YRC1000microReadPosition();
}

void MainWindow::on_btn_set_pos_clicked()
{
    double set_speed = ui->txt_setSpeed->text().toDouble();
    QVector<double> set_position;
//    qDebug() << "Set position: " << set_position;
    if(ui->rbtn_xyz->isChecked()){
        set_position.append(ui->txt_setX->text().toDouble());
        set_position.append(ui->txt_setY->text().toDouble());
        set_position.append(ui->txt_setZ->text().toDouble());
        set_position.append(ui->txt_setRoll->text().toDouble());
        set_position.append(ui->txt_setPitch->text().toDouble());
        set_position.append(ui->txt_setYaw->text().toDouble());
        if(ui->rbtn_linkAbs->isChecked())
            yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
                CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,CMD_DATA_MOVE_SPEED_TYPE_LINK,set_speed,&set_position);
        else if(ui->rbtn_lineAbs->isChecked())
            yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
                CMD_HEADER_MOVE_INSTANCE_STRAIGHT_ABSOLUTE,CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
        else if(ui->rbtn_LineInc->isChecked())
            yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
                CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
    }
    else if(ui->rbtn_rpy->isChecked()){
        set_position.append(0);
        set_position.append(0);
        set_position.append(0);
        set_position.append(ui->txt_setRoll->text().toDouble());
        set_position.append(ui->txt_setPitch->text().toDouble());
        set_position.append(ui->txt_setYaw->text().toDouble());

        if(ui->rbtn_linkAbs->isChecked())
            yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
                CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
        else if(ui->rbtn_lineAbs->isChecked())
            yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
                CMD_HEADER_MOVE_INSTANCE_STRAIGHT_ABSOLUTE,CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
        else if(ui->rbtn_LineInc->isChecked())
            yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
                CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
    }
}

void MainWindow::on_btn_home_clicked()
{
    QVector<double> set_position= {185,0,85,180,0,0};
    double set_speed = 50;

    yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
        CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED,set_speed,&set_position);
}

void MainWindow::updateUICallback(){
    QVector<double> position = yrc100micro_com.updateRobotPosition();
    ui->txt_getX->setText(QString::number(position[0]));
    ui->txt_getY->setText(QString::number(position[1]));
    ui->txt_getZ->setText(QString::number(position[2]));
    ui->txt_getRoll->setText(QString::number(position[3]));
    ui->txt_getPitch->setText(QString::number(position[4]));
    ui->txt_getYaw->setText(QString::number(position[5]));

    QVector<double> pulses = yrc100micro_com.updateRobotPulse();
    QVector<double> degs = convert_pulses_to_degs(pulses);
    ui->txt_j1->setText(QString::number(degs[0]));
    ui->txt_j2->setText(QString::number(degs[1]));
    ui->txt_j3->setText(QString::number(degs[2]));
    ui->txt_j4->setText(QString::number(degs[3]));
    ui->txt_j5->setText(QString::number(degs[4]));
    ui->txt_j6->setText(QString::number(degs[5]));

    quint16 status_code = yrc100micro_com.updateRobotStatus();
    status_code &= RES_VALUE_READING_RUNNING_MASK;
    if(status_code){
        ui->lb_run_status->setText("Running");
    }
    else
        ui->lb_run_status->setText("Stop");

}

void MainWindow::timerCallback()
{
    //yrc100micro_com.YRC1000microReadStatus();
    yrc100micro_com.YRC1000microReadPosition();
}

//void MainWindow::on_btn_saveJob_clicked()
//{
////  qDebug() << "ui data: " << ui->txt_saveJob->text();
////    yrc100micro_com.YRC1000microSaveJob(0,ui->txt_saveJob->text());
//}

//void MainWindow::on_btn_loadJob_clicked()
//{
////    yrc100micro_com.YRC1000microLoadJob(ui->txt_loadJob->text());
////    qDebug() << "File name " << ui->txt_loadJob->text() << endl;
//}

void MainWindow::inc_dec_bt_clicked()
{
    QPushButton *current_bt = (QPushButton*)sender();

    double set_speed;
    bool mode_selection;
    double distance_mm = (double)ui->slider_dis_mm->value();
    double distance_deg = (double)ui->slider_dis_deg->value();

    QVector<double> set_position(6, 0);
    if (current_bt->objectName() == "t1_inc"){
        set_position[0] = distance_mm;
        mode_selection=true;
    }else if(current_bt->objectName() == "t2_inc"){
        set_position[1] = distance_mm;
        mode_selection=true;
    }else if(current_bt->objectName() == "t3_inc"){
        set_position[2] = distance_mm;
        mode_selection=true;
    }else if(current_bt->objectName() == "t4_inc"){
        set_position[3] = distance_deg;
        mode_selection=false;
    }else if(current_bt->objectName() == "t5_inc"){
        set_position[4] = distance_deg;
        mode_selection=false;
    }else if(current_bt->objectName() == "t6_inc"){
        set_position[5] = distance_deg;
        mode_selection=false;
    }else if(current_bt->objectName() == "t1_dec"){
        set_position[0] = -distance_mm;
        mode_selection=true;
    }else if(current_bt->objectName() == "t2_dec"){
        set_position[1] = -distance_mm;
        mode_selection=true;
    }else if(current_bt->objectName() == "t3_dec"){
        set_position[2] = -distance_mm;
        mode_selection=true;
    }else if(current_bt->objectName() == "t4_dec"){
        set_position[3] = -distance_deg;
        mode_selection=false;
    }else if(current_bt->objectName() == "t5_dec"){
        set_position[4] = -distance_deg;
        mode_selection=false;
    }else if(current_bt->objectName() == "t6_dec"){
        set_position[5] = -distance_deg;
        mode_selection=false;
    }

    set_speed = (mode_selection==true)?(double)ui->slider_sp_mps->value():(double)ui->slider_sp_degps->value();
    quint8 selected_sp_type = (mode_selection==true)?CMD_DATA_MOVE_SPEED_TYPE_V_SPEED:CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED;

    yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
        CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT, selected_sp_type, set_speed, &set_position);

}


void MainWindow::on_fk_calculate_clicked()
{
    QVector<double> input_theta(6);
    QVector<double> output_result(6);
    input_theta[0] = qDegreesToRadians(ui->txt_j1->text().toDouble());
    input_theta[1] = qDegreesToRadians(ui->txt_j2->text().toDouble());
    input_theta[2] = qDegreesToRadians(ui->txt_j3->text().toDouble());
    input_theta[3] = qDegreesToRadians(ui->txt_j4->text().toDouble());
    input_theta[4] = qDegreesToRadians(ui->txt_j5->text().toDouble());
    input_theta[5] = qDegreesToRadians(ui->txt_j6->text().toDouble());
    yrc100micro_com.foward_kinematic(&input_theta, &output_result);
    ui->fk_x->setText(QString::number(output_result[0],'f',4));
    ui->fk_y->setText(QString::number(output_result[1],'f',4));
    ui->fk_z->setText(QString::number(output_result[2],'f',4));
    ui->fk_rx->setText(QString::number(qRadiansToDegrees(output_result[3]),'f',4));
    ui->fk_ry->setText(QString::number(qRadiansToDegrees(output_result[4]),'f',4));
    ui->fk_rz->setText(QString::number(qRadiansToDegrees(output_result[5]),'f',4));

}

void MainWindow::on_btn_get_joint_clicked()
{
    yrc100micro_com.YRC1000microReadPulse();
}

QVector<double> MainWindow::convert_pulses_to_degs(QVector<double> &pulses)
{
    QVector<double> degs(6);
    qDebug() << pulses;
    degs[0] = pulses.at(0)*30.0/34816.0;
    degs[1] = pulses.at(1)*90.0/102400.0;
    degs[2] = pulses.at(2)*90.0/51200.0;
    degs[3] = pulses.at(3)*30.0/10240.0;
    degs[4] = pulses.at(4)*30.0/10240.0;
    degs[5] = pulses.at(5)*30.0/10240.0;
    return degs;
}

void MainWindow::on_chbox_autoGetPos_clicked(bool checked)
{
    autoGetPos = checked;
    if(!checked)
        timer->stop();
}

void MainWindow::jogging_timerCallback()
{
    QVector<double> set_position= {0.7,0,0,0,0,0};
    if(ui->ReverseCheck->isChecked())
    {
        set_position= {-0.7,0,0,0,0,0};
    }
    double set_speed = 20;

    yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
        CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&set_position);
}
void MainWindow::on_StartJoggingBtn_clicked()
{
     joggingTimer->start(50);
     ui->StopJoggingBtn->setEnabled(true);
     ui->StartJoggingBtn->setEnabled(false);
}

void MainWindow::on_StopJoggingBtn_clicked()
{
     joggingTimer->stop();
     ui->StopJoggingBtn->setEnabled(false);
     ui->StartJoggingBtn->setEnabled(true);
}

void MainWindow::currentForceX(int F)
{
    float realForce = float(F) * 0.1 - 5.0;
    QString force = QString::number(realForce, 'g', 3);
    ui->ForceX_Value->setText(force);
}

void MainWindow::currentForceY(int F)
{
    float realForce = float(F) * 0.1 - 5.0;
    QString force = QString::number(realForce, 'g', 3);
    ui->ForceY_Value->setText(force);
}

void MainWindow::currentForceZ(int F)
{
    float realForce = float(F) * 0.1 - 5.0;
    QString force = QString::number(realForce, 'g', 3);
    ui->ForceZ_Value->setText(force);
}

void MainWindow::on_ImStartBtn_clicked()
{

    ui->ImStartBtn->setCheckable(false);
    ui->releaseBtn->setCheckable(true);
    ui->ImStopBtn->setCheckable(true);
    im_controller->setParam(ui->massValue->text().toDouble(),
                            ui->stiffnessValue->text().toDouble(),
                            ui->dissipationValue->text().toDouble(),
                            0.05);
    ui->massValue->setEnabled(false);
    ui->stiffnessValue->setEnabled(false);
    ui->dissipationValue->setEnabled(false);
    impedanceTimer->start(50);
}

void MainWindow::on_ImStopBtn_clicked()
{
    impedanceTimer->stop();
    ui->ImStartBtn->setCheckable(true);
    ui->releaseBtn->setCheckable(false);
    ui->ImStopBtn->setCheckable(false);
    ui->massValue->setEnabled(true);
    ui->stiffnessValue->setEnabled(true);
    ui->dissipationValue->setEnabled(true);
}

void MainWindow::updateDisplacement()
{
    double currFx = ui->ForceX_Value->text().toDouble();
    double currFy = ui->ForceY_Value->text().toDouble();
    double currFz = ui->ForceZ_Value->text().toDouble();

    im_controller->setForce({currFx, currFy, currFz});
    QVector<double> new_dis = im_controller->getCurrentDis();
    new_dis.append({0, 0, 0});
    qDebug() << new_dis << '\n';

    double set_speed = 35;

    yrc100micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
        CMD_HEADER_MOVE_INSTANCE_STRAIGHT_INCREMENT,CMD_DATA_MOVE_SPEED_TYPE_V_SPEED,set_speed,&new_dis);
}

void MainWindow::on_releaseBtn_clicked()
{
    ui->ForceXSlider->setValue(50);
    ui->ForceYSlider->setValue(50);
    ui->ForceZSlider->setValue(50);
}
