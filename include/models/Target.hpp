#ifndef TARGET_H
#define TARGET_H

#include <math.h>
#include <vector>
#include "PointMass.hpp"
#include "../control/TargetGuidance.hpp"

class Target: public PointMass{
    public:
        Target();
        ~Target();
        bool init(  double _m, std::vector<double>& _stateVector, double n_max,
                    TargetGuidance* _targetGuidance, PointMass* _pursuer = 0
                    );
        bool STEP(double dt) override;
        bool set_actualForceAndTorques();
        bool set_controlParams(); //Установка текущих параметров управления
        void set_pursuer(PointMass* _pursuer); //Установка цели
        void set_pursuer(std::vector<PointMass*> _pursuer); //Установка цели
        std::vector<PointMass*> get_pursuers() const; //Возвращает указатель на преследователя
        std::vector<double> get_n_xyz() const; //Выдача параметров управления

    private:
        TargetGuidance* targetGuidance;
        std::vector<PointMass*> pursuers;
        double n_max; //Максимальное значение управляющих параметров
        std::vector<double> n_xyz; //Управляющие параметры
        bool nUpToDate;
};


#endif