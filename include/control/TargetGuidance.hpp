#ifndef TARGETGUIDANCE_H
#define TARGETGUIDANCE_H

#include <math.h>
#include <vector>
#include <string>
#include "IGuidance.hpp"

class TargetGuidance: public IGuidance{
    public:
        TargetGuidance();
        ~TargetGuidance();
        std::vector<double> get_GuidanceSignal(PointMass* target, std::vector<PointMass*> pursuers) override; //Возвращает требуемые угловые скорости (dTheta_dt, dPsi_dt)

    private:

        bool updateInformation(PointMass* target, std::vector<PointMass*> pursuers);// Инициализация полей класса в зависимости от цели и ракеты
        double r_rel; //Расстояние между целью и ракетой
        std::vector<double> phi_hi; //Вектор проекций углов пеленга
        std::vector<double> V_rel; //Относительная скорость в НЗСК

        std::vector<double> B_1() const; //Вторая строка матрицы перехода к сферической СК с центром в ракете и осью Ох по вектор ракета-цель
        std::vector<double> B_2() const; //Третья строка матрицы перехода к сферической СК с центром в ракете и осью Ох по вектор ракета-цель
        std::vector<double> A_0() const; //Первая строка матрицы перехода СфСК(цель-ракета) -> НЗСК
        std::vector<double> A_1() const; //Вторая строка матрицы перехода СфСК(цель-ракета) -> НЗСК
        std::vector<double> A_2() const; //Третья строка матрицы пперехода СфСК(цель-ракета) -> НЗСК   
            
        std::vector<double> V_phi_hi() const; //Скорость в сферической СК               
        std::vector<double> Transform_V_to_signal() const; //Преобразование скоростей в сферической СК к сигналу наведения
        
};  

#endif