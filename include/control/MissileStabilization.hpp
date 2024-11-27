#ifndef MISSILESTABILIZATION_H
#define MISSILESTABILIZATION_H

#include <math.h>
#include <vector>
#include <string>
#include "IStabilization.hpp"

class MissileStabilization: public IStabilization{
    public:
        MissileStabilization();
        ~MissileStabilization();
        bool init(std::vector<double>& _K_stabilization, double _K_roll);
        std::vector<double> get_controlParams(RigidBody* missile, std::vector<double>& guidanceSignal); //Возвращает параметры управления

    private:
        std::vector<double>  K_stabilization; //Коэффициенты усиления системы стаблизации K = (K_droll, K_dyaw, K_dpitch)
        double K_roll; //Ещё один коэффициент системы стабилизации
}; 

#endif