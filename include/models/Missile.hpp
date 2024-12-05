#ifndef MISSILE_H
#define MISSILE_H

#include <math.h>
#include <vector>
#include "RigidBody.hpp"
#include "../aerodynamics/IAerodynamic.hpp"
#include "../control/MissileGuidance.hpp"
#include "../control/MissileStabilization.hpp"

class Missile: public RigidBody{
    public:
        Missile();
        ~Missile();
        bool init(  double _m, std::vector<double>& _stateVector, std::vector<double>& _J, std::vector<double>& _roll_yaw_pitch, 
                    double _l, double _d,
                    std::vector<double>& _w, double _delta_max, IAerodynamic* _missileAerodynamic,
                    MissileStabilization* _missileStabilization, MissileGuidance* _missileGuidance, PointMass* _target = 0
                    );
        bool STEP(double dt) override;
        bool set_controlParams(); //Установка текущих параметров управления
        void set_target(PointMass* _target); //Установка цели
        PointMass* get_target(); //Выдача указателя на цель
        std::vector<double> get_deltas(); //Выдача параметров управления

    private:
        IAerodynamic* missileAerodynamic;
        MissileStabilization* missileStabilization;
        MissileGuidance* missileGuidance;
        PointMass* target;
        double l; //Длина ЛА
        double d; //Диаметр ЛА
        double delta_max; //Максимальное значение управляющих параметров
        std::vector<double> deltas; //Управляющие параметры
        bool deltaUpToDate;

        std::vector<double> calc_bodyRelatedAeroForce(); //Подсчёт сил, действующих в связанной с телом СК
        bool calc_forces(); //Подсчёт и установка действующих на ЛА, как на точечную массу, сил в НЗСК
        bool calc_torques(); //Подсчёт и установка действующих на ЛА, как на твердое тело, моментов

};


#endif