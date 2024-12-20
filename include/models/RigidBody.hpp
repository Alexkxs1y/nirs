#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <math.h>
#include <vector>
#include "PointMass.hpp"

class RigidBody: public PointMass{
    public:
        RigidBody();
        RigidBody(RigidBody &_rigidBody);
        bool init(double _m, std::vector<double>& _stateVector, std::vector<double>& _J, std::vector<double>& _roll_yaw_pitch, std::vector<double>& _w);

        //Установка новго состояния твердого тела
        bool set_state(std::vector<double>& _stateVector, std::vector<double>& _roll_yaw_pitch, std::vector<double>& _w);

        bool STEP(double dt) override; //Шаг по вревемин для твердого тела
        //using PointMass::STEP_der(double dt, std::vector<double> dr_dt, std::vector<double> dV_dt);
        bool STEP(double dt, std::vector<double> dr_dt, std::vector<double> dV_dt, std::vector<double> dryp_dt, std::vector<double> dw_dt); //Шаг по времени с заданием производных


        ~RigidBody();

        bool set_torques(std::vector<double>& _torques); //Установка вектора моментов

       
        std::vector<double> get_J() const; //Вернуть значение вектора моментов инерции (Jx, Jy, Jz)
        double get_yaw() const; //Вернуть значение рысканья
        double get_pitch() const; //Вернуть значение тангажа
        double get_roll() const; //Вернуть значение крена
        double get_dyaw_dt() const; //Вернуть значение производной рысканья
        double get_dpitch_dt() const; //Вернуть значение производной тангажа
        double get_droll_dt() const; //Вернуть значение производной крена
        double get_wx() const; //Вернуть значение угловой скорости относительно связной оси Ох
        double get_wy() const; //Вернуть значение угловой скорости относительно связной оси Оy
        double get_wz() const; //Вернуть значение угловой скорости относительно связной оси Оz
        double get_alpha() const; //Вернуть значение угла атаки
        double get_beta() const; //Вернуть значение угла скольжения

        std::vector<double> get_ypr() const; //Вернуть значение вектора (yaw, pitch, roll)
        std::vector<double> get_ryp() const; //Вернуть значение вектора (roll, yaw, pitch)
        std::vector<double> get_dryp_dt() const; //Вернуть значение вектора (droll_dt, dyaw_dt, dpitch_dt)
        std::vector<double> get_dypr_dt() const; //Вернуть значение вектора (dyaw_dt, dpitch_dt, droll_dt)
        std::vector<double> get_alpha_beta() const; //Вернуть значение вектора (alpha, beta)
        std::vector<double> get_w() const; //Вернуть значение вектора угловых скоростей w = (wx, wy, wz)
        std::vector<double> get_dw_dt() const; //Вернуть значение вектора угловых скоростей dw_dt = (dwx_dt, dwy_dt, dwz_dt)
        std::vector<double> get_orientationVector() const; //Вернуть значения вектора ориентации (roll, yaw, pitch, wx, wy, wz)

    protected:

        std::vector<double> orientationVector; //Вектор углового состояния  (roll, yaw, pitch, wx, wy, wz)
        std::vector<double> J; // Вектор моментов инерции J = (Jx, Jy, Jz)
        std::vector<double> torques; // Вектор моментов M = (Mx, My, Mz)
        bool torquesUpToDate;  

        double droll_dt() const; //Производная крена
        double dyaw_dt() const; //Производная рысканья
        double dpitch_dt() const; //Производная тангажа
        double dwx_dt() const; //Производная угловой скорости относительно связной оси Ох
        double dwy_dt() const; //Производная угловой скорости относительно связной оси Оy
        double dwz_dt() const; //Производная угловой скорости относительно связной оси Оz

};

#endif
