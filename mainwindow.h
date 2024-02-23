#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QButtonGroup>
#include "yrc1000micro_com.h"
#include <QTimer>
#include <controller.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


private slots:
    void on_btn_connect_clicked();

    void on_btn_servo_clicked();

    void updateUICallback();

    void timerCallback();

    void jogging_timerCallback();

//    void on_btn_saveJob_clicked();

//    void on_btn_loadJob_clicked();

    void inc_dec_bt_clicked();

    void on_fk_calculate_clicked();

    void on_btn_get_joint_clicked();

    void on_btn_home_clicked();

    void on_btn_set_pos_clicked();

    void on_btn_get_pos_clicked();

    void on_chbox_autoGetPos_clicked(bool checked);

    void on_StartJoggingBtn_clicked();

    void on_StopJoggingBtn_clicked();

    void currentForceX(int);

    void currentForceY(int);

    void currentForceZ(int);

    void updateDisplacement();

    void on_ImStartBtn_clicked();

    void on_ImStopBtn_clicked();

    void on_releaseBtn_clicked();

private:
    Ui::MainWindow *ui;
    QPalette pal_red = palette();
    QPalette pal_green = palette();
    QButtonGroup coor_type;
    QButtonGroup move_type;
    //    Init udp connection
//    UDP udp_sever;

    YRC1000micro_com yrc100micro_com;
    QTimer *timer;
    QTimer *joggingTimer;
    QTimer *impedanceTimer;
    bool autoGetPos;

    Controller *im_controller;

    QVector<double> convert_pulses_to_degs(QVector<double> &pulses);
};

#endif // MAINWINDOW_H
