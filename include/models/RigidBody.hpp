#ifndef RIGID_BODY.H
#define RIGID_BODY.H

#include <math.h>
#include <vector>
#include "PointMass.hpp"

class RigidBody: public PointMass{
    public:
        RigidBody();

        bool init(double m, std::vector<double> _stateVector, std::vector<double> _J, std::vector<double> _angleVector);

        bool STEP(double dt) override; //Шаг по вревемин для твердого тела

        ~RigidBody();

        bool set_torques(std::vector<double> M); // Установка вектора моментов

        //====================================================================//
        //============================Гетеры==================================//
        //====================================================================//
        std::vector<double> get_J(); 
        double get_yaw();
        double get_pitch();
        double get_roll();
        double get_wx();
        double get_wy();
        double get_wz();

        std::vector<double> get_ypr(); //Вернуть значение вектора (yaw, pitch, roll)
        std::vector<double> get_w(); //Вернуть значение вектора угловых скоростей w = (wx, wy, wz)

    protected:

        std::vector<double> J; // Вектор моментов инерции J = (Jx, Jy, Jz)
        std::vector<double> torques; // Вектор моментов M = (Mx, My, Mz)
        std::vector<double> angleVector; // Вектор углового состояния  (roll, yaw, pitch, wx, wy, wz)
        bool torquesUpToDate;  

        double droll_dt();
        double dyaw_dt();
        double dpitch_dt();
        double dwx_dt();
        double dwy_dt();
        double dxz_dt();
};

#endif
