#ifndef APERIOD_MISSILE_H
#define APERIOD_MISSILE_H

#include <math.h>
#include <vector>
#include "PointMass.hpp"
#include "Target.hpp"
#include "../control/IGuidance.hpp"

class AperiodMissile: public PointMass{
    public:
        AperiodMissile();
        ~AperiodMissile();
        AperiodMissile(AperiodMissile &_missile);
        bool init(double _m, std::vector<double>& _stateVector, double _n_max, double _T_missisle, IGuidance* _propGuidance, Target* _target = 0);
        bool STEP(double dt) override;
        bool STEP(double dt, std::vector<double>& dr_dt, std::vector<double>& dV_dt, std::vector<double>& d_n_xyz_body);
        bool set_actualForceAndTorques(); //Установка сил и моментов
        bool set_controlParams(); //Установка текущих параметров управления
        void set_target(Target* _target); //Установка цели
        void set_target(std::vector<Target*>& _targets); //Установка целей
        void set_propGuidance(IGuidance* _propGuidance);
        void set_crossGuidance(IGuidance* _crossGuidance);
        void set_state(std::vector<double>& _stateVector_missile, std::vector<double> & _n_xyz_body);

        IGuidance* get_Guidance();//Возвращает ссылку на текущую систему наведения
        std::vector<Target*> get_targets(); //Выдача указателя на цель
        std::vector<double> get_n_xyz(); //Выдача параметров управления
        std::vector<double> get_n_xyz_body(); //Выдача параметров управления
        std::vector<double> get_d_n_xyz_body();
        double get_T_missile();
        double get_n_max();
        void choose_Guidance();//Выбор системы наведения в зависимости от количества целей

    private:
        IGuidance* workGuidance;//Система наведения, которая работает в текущий момент(выбор из пропорциональной и по двум целям)
        IGuidance* propGuidance;//Пропорциональная система наведения установленная на ракете
        IGuidance* crossGuidance;//Система наведения по двум целям
        //Набор целей, которых преследует ракеты(обычно 1 цель)
        std::vector<Target*> targets;
        double n_max; //Максимальное значение управляющих параметров
        double T_missisle; //Постоянная времени ракеты
        std::vector<double> n_xyz; //Управляющие параметры
        std::vector<double> n_xyz_body;
        std::vector<double> d_n_xyz_body; //производная нормальных перегрузок
        bool nUpToDate;

        void solveControlConflict(std::vector<PointMass*>& _tags);//Проверка и обновление информации о наведении. Возвращает точечные массы, по которым наводимся
        bool calc_forces(); //Подсчёт и установка действующих на ЛА, как на точечную массу, сил в НЗСК
};


#endif