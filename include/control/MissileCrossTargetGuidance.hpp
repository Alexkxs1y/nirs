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
        bool init(  std::vector<double>& _K_guidance, double _reGuidanceTime, double _dt);
        std::vector<double> get_GuidanceSignal(PointMass* missile, std::vector<PointMass*> targets) override; //Возвращает требуемые угловые скорости (dTheta_dt, dPsi_dt)
        bool needToUpdateData() override;
        void updateData(std::pair<std::vector<double>, std::vector<double>>& data) override;  
    private:
        std::pair<std::vector<double>, std::vector<double>>  polynomCoefs; //Коэффициента полинома задающего траекторию на текущем участке
        std::vector<double>  K_guidance; //Вектор коэффициентов наведения по ошибке
        //Радиус поражения, точность с которой надо соблюдать радиус, время переопределения траектории, 
        //время последнего переопределения траектории, текущее время и шаг по времени с которым будет проводиться анализ траекторий
        double reGuidanceTime, last_reGuidanceTime, time, dt;
       
        bool needToUpdate;

        bool updateInformation(PointMass* missile, std::vector<PointMass*> targets);// Инициализация полей класса в зависимости от цели и ракеты
        std::vector<double> currentMiss(PointMass* missile); //Определение текущего отклонения от траектории
}; 

#endif