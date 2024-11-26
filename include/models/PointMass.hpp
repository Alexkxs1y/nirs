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

        
        double get_m(); //Вернуть значение массы
        double get_x(); //Вернуть значение координаты х
        double get_y(); //Вернуть значение координаты y
        double get_z(); //Вернуть значение координаты z
        double get_Vx(); //Вернуть значение скорости Vx
        double get_Vy(); //Вернуть значение скорости Vy
        double get_Vz(); //Вернуть значение скорости Vz
        double get_Vabs(); //Вернуть значение модуля скорости
        double get_rabs(); //Вернуть значение модуля радиус-вектора

        std::vector<double> get_r(); //Вернуть значение радиус-вектора
        std::vector<double> get_V(); //Вернуть значение вектора скорости
        std::vector<double> get_forces(); //Вернуть значение сил

    protected:

        std::vector<double> stateVector; //Вектор состояния (x, y, z, Vx, Vy, Vz)
        std::vector<double> forces; //Вектор сил (Fx, Fy, Fz)
        double m; //Масса
        bool forcesUpToDate; //Флаг на проверку того, что после вызова шага были обновлены силы

        double dx_dt(); //Производная х
        double dy_dt(); //Производная y 
        double dz_dt(); //Производная z
        double dVx_dt(); //Производная Vx
        double dVy_dt(); //Производная Vy
        double dVz_dt(); //Производная Vz
        //double dm_dt();
};

#endif