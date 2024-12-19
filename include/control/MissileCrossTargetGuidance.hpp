#ifndef MISSILE_CROSS_TARGET_GUIDANCE_H
#define MISSILE_CROSS_TARGET_GUIDANCE_H

#include <math.h>
#include <vector>
#include <string>
#include "IGuidance.hpp"

class MissileCrossTargetGuidance: public IGuidance{
    public:
        MissileCrossTargetGuidance();
        ~MissileCrossTargetGuidance();
        bool init(std::vector<double>& _K_guidance);
        std::vector<double> get_GuidanceSignal(PointMass* missile, std::vector<PointMass*> targets) override; //Возвращает требуемые угловые скорости (dTheta_dt, dPsi_dt)

    private:
        std::vector<double>  K_guidance; //Вектор коэффициентов наведения по ошибке
        //Радиус поражения, точность с которой надо соблюдать радиус, время переопределения траектории, 
        //время последнего переопределения траектории, шаг по времени с которым будет проводиться анализ траекторий
        double effectiveRadius, tolerance, reGuidanceTime, reGuidanceTime, dt;
        //Пропорциональное наведение на которое следует переключиться, если уже невозможно лететь в пересечение допустимых зон каждой цели
        IGuidance* proportionalGuidance;
        std::pair<std::vector<double>, std::vector<double>>  polynomCoefs; //Коэффициента полинома задающего траекторию на текущем участке
        bool updateInformation(PointMass* missile, PointMass* target_1, PointMass* target_2);// Инициализация полей класса в зависимости от цели и ракеты

}; 

#endif