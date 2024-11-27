#ifndef MISSILEGUIDANCE_H
#define MISSILEGUIDANCE_H

#include <math.h>
#include <vector>
#include <string>
#include "IGuidance.hpp"

class MissileGuidance: public IGuidance{
    public:
        MissileGuidance();
        ~MissileGuidance();
        bool init(std::vector<double>& _K_guidance);
        std::vector<double> get_GuidanceSignal(PointMass* missile, PointMass* target) override; //Возвращает требуемые угловые скорости (dTheta_dt, dPsi_dt)

    private:
        std::vector<double>  K_guidance; //Вектор коэффициентов пропорциональности

        bool updateInformation(PointMass* missile, PointMass* target);// Инициализация полей класса в зависимости от цели и ракеты
        double r_rel; //Расстояние между целью и ракетой
        std::vector<double> phi_hi; //Вектор проекций углов пеленга
        std::vector<double> V_rel; //Относительная скорость в НЗСК

        std::vector<double> B_1() const; //Вторая строка матрицы перехода к сферической СК с центром в ракете и осью Ох по вектор ракета-цель
        std::vector<double> B_2() const; //Третья строка матрицы перехода к сферической СК с центром в ракете и осью Ох по вектор ракета-цель
        std::vector<double> d_eta_dt() const; //Производная угла пеленга (d_phi_dt, d_hi_dt);     
}; 

#endif