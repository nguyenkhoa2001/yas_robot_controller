#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QVector>

class Controller
{
public:
    Controller();

    void setParam(double M, double K, double B, double Ts);

    void setForce(QVector<double> f);

    QVector<double> getCurrentDis();

private:
    // Properpies:
    double M, K, B, Ts;

    QVector<double> curr_f, f_prev, f_prev2;

    QVector<double> curr_dis, dis_prev, dis_prev2;

    // Method
    double calculate_delta(double curr_f, double prev_dis, double prev2_dis, double prev_f, double prev2_f);

    void update_value();
};

#endif // CONTROLLER_H
