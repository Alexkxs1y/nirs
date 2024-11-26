#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <math.h>
#include <vector>
#include "PointMass.hpp"

class RigidBody: public PointMass{
    public:
        RigidBody();

        bool init(double _m, std::vector<double> _stateVector, std::vector<double> _J, std::vector<double> _roll_yaw_pitch, std::vector<double> _w);

        bool STEP(double dt) override; //Шаг по вревемин для твердого тела

        ~RigidBody();

        bool set_torques(std::vector<double> _torques); //Установка вектора моментов

       
        std::vector<double> get_J(); //Вернуть значение вектора моментов инерции (Jx, Jy, Jz)
        double get_yaw(); //Вернуть значение рысканья
        double get_pitch(); //Вернуть значение тангажа
        double get_roll(); //Вернуть значение крена
        double get_dyaw_dt(); //Вернуть значение производной рысканья
        double get_dpitch_dt(); //Вернуть значение производной тангажа
        double get_droll_dt(); //Вернуть значение производной крена
        double get_wx(); //Вернуть значение угловой скорости относительно связной оси Ох
        double get_wy(); //Вернуть значение угловой скорости относительно связной оси Оy
        double get_wz(); //Вернуть значение угловой скорости относительно связной оси Оz
        double get_alpha(); //Вернуть значение угла атаки
        double get_beta(); //Вернуть значение угла скольжения

        std::vector<double> get_ypr(); //Вернуть значение вектора (yaw, pitch, roll)
        std::vector<double> get_dypr_dt(); //Вернуть значение вектора (dyaw_dt, dpitch_dt, droll_dt)
        std::vector<double> get_alpha_beta(); //Вернуть значение вектора (alpha, beta)
        std::vector<double> get_w(); //Вернуть значение вектора угловых скоростей w = (wx, wy, wz)

    protected:

        std::vector<double> orientationVector; //Вектор углового состояния  (roll, yaw, pitch, wx, wy, wz)
        std::vector<double> J; // Вектор моментов инерции J = (Jx, Jy, Jz)
        std::vector<double> torques; // Вектор моментов M = (Mx, My, Mz)
        bool torquesUpToDate;  

        double droll_dt(); //Производная крена
        double dyaw_dt(); //Производная рысканья
        double dpitch_dt(); //Производная тангажа
        double dwx_dt(); //Производная угловой скорости относительно связной оси Ох
        double dwy_dt(); //Производная угловой скорости относительно связной оси Оy
        double dwz_dt(); //Производная угловой скорости относительно связной оси Оz

};

#endif
