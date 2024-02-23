#include "controller.h"

Controller::Controller()
{
    M = 20;
    B = 250;
    K = 4000;

    f_prev = {0, 0, 0};
    f_prev2 = {0, 0, 0};

    dis_prev = {0, 0, 0};
    dis_prev2 = {0, 0, 0};
    curr_dis = {0, 0, 0};
}

void Controller::setParam(double M, double K, double B, double Ts)
{
    this->M = M;
    this->K = K;
    this->B = B;
    this->Ts = Ts;
}

void Controller::setForce(QVector<double> f)
{
    curr_f = f;
}

QVector<double> Controller::getCurrentDis()
{
    curr_dis.clear();
    QVector<double> convert_dis;
    for(int i = 0; i<3; i++)
    {
        curr_dis.append(calculate_delta(curr_f.at(i), dis_prev.at(i), dis_prev2.at(i), f_prev.at(i), f_prev2.at(i)));
        convert_dis.append(double(qRound(1e6*curr_dis.at(i)))/1e3);
    }

    update_value();
    return convert_dis;
}

void Controller::update_value()
{
    dis_prev = curr_dis;
    dis_prev2 = dis_prev;

    f_prev = curr_f;
    f_prev2 = f_prev;
}

double Controller::calculate_delta(double curr_f, double prev_dis, double prev2_dis, double prev_f, double prev2_f)
{
    double Ts_sq = Ts*Ts;
    double numerator = Ts_sq*(curr_f + 2*prev_f + prev2_f) - (2*K*Ts_sq - 8*M)*prev_dis - (4*M - 2*B*Ts + K*Ts_sq)*prev2_dis;
    double denominator = 4*M + 2*B*Ts + K*Ts_sq;
    return numerator/denominator;
}
