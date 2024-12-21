#ifndef MISSILE_H
#define MISSILE_H

#include <math.h>
#include <vector>
#include "RigidBody.hpp"
#include "Target.hpp"
#include "../control/IGuidance.hpp"
#include "../aerodynamics/IAerodynamic.hpp"
#include "../control/MissileGuidance.hpp"
#include "../control/MissileStabilization.hpp"

class Missile: public RigidBody{
    public:
        Missile();
        ~Missile();
        Missile(Missile &_missile);
        bool init(  double _m, std::vector<double>& _stateVector, std::vector<double>& _J, std::vector<double>& _roll_yaw_pitch, std::vector<double>& _w, 
                    double _l, double _d, double _delta_max, IAerodynamic* _missileAerodynamic,
                    MissileStabilization* _missileStabilization, IGuidance* _missileGuidance, Target* _target = 0
                    );
        bool STEP(double dt) override;
        bool set_actualForceAndTorques(); //Установка сил и моментов
        bool set_controlParams(); //Установка текущих параметров управления
        void set_target(Target* _target); //Установка цели
        void set_target(std::vector<Target*> _targets); //Установка целей
        void set_propGuidance(IGuidance* _propGuidance);
        void set_crossGuidance(IGuidance* _crossGuidance);

        double get_l();
        double get_d();
        double get_delta_max();
        MissileStabilization* get_missileStab();
        IAerodynamic* get_missileAero();
        IGuidance* get_Guidance();//Возвращает ссылку на текущую систему наведения
        std::vector<Target*> get_targets(); //Выдача указателя на цель
        std::vector<double> get_deltas(); //Выдача параметров управления
        void choose_Guidance();//Выбор системы наведения в зависимости от количества целей

    private:
        IAerodynamic* missileAerodynamic;
        MissileStabilization* missileStabilization;
        IGuidance* workGuidance;//Система наведения, которая работает в текущий момент(выбор из пропорциональной и по двум целям)
        IGuidance* propGuidance;//Пропорциональная система наведения установленная на ракете
        IGuidance* crossGuidance;//Система наведения по двум целям
        //Набор целей, которых преследует ракеты(обычно 1 цель)
        std::vector<Target*> targets;
        double l; //Длина ЛА
        double d; //Диаметр ЛА
        double delta_max; //Максимальное значение управляющих параметров
        std::vector<double> deltas; //Управляющие параметры
        bool deltaUpToDate;

        void solveControlConflict(std::vector<PointMass*>& _tags);//Проверка и обновление информации о наведении. Возвращает точечные массы, по которым наводимся
        std::vector<double> calc_bodyRelatedAeroForce(); //Подсчёт сил, действующих в связанной с телом СК
        bool calc_forces(); //Подсчёт и установка действующих на ЛА, как на точечную массу, сил в НЗСК
        bool calc_torques(); //Подсчёт и установка действующих на ЛА, как на твердое тело, моментов

};


#endif