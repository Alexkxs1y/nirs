#ifndef POINT_MASS_H
#define POINT_MASS_H

#include <math.h>
#include <vector>


class PointMass{
    public:
        PointMass();

        bool init(double _m, std::vector<double> _stateVector);

        virtual bool STEP(double dt); //Функция шага по времени 

        virtual ~PointMass();   

        bool set_forces(std::vector<double> _forces); //Установка текущих сил, действующих на центр масс

        //====================================================================//
        //============================Гетеры==================================//
        //====================================================================//
        double get_m();
        double get_x();
        double get_y();
        double get_z();
        double get_Vx();
        double get_Vy();
        double get_Vz();

        std::vector<double> get_r(); //Вернуть значение радиус-вектор
        std::vector<double> get_V(); //Вернуть значение вектора скорости
        std::vector<double> get_forces(); //Вернуть значение сил

    protected:

        std::vector<double> stateVector; //Вектор состояния (x, y, z, Vx, Vy, Vz)
        std::vector<double> forces; //Вектор сил (Fx, Fy, Fz)
        double m; //Масса
        bool forcesUpToDate; //Флаг на проверку того, что после вызова шага были обновлены силы

        double dx_dt();
        double dy_dt();
        double dz_dt();
        double dVx_dt();
        double dVy_dt();
        double dVz_dt();
        //double dm_dt();
};

#endif