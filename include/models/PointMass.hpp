#ifndef POINT_MASS_H
#define POINT_MASS_H

#include <math.h>
#include <vector>


class PointMass{
    public:
        PointMass();
        PointMass(PointMass &_pointMass);
        bool init(double _m, std::vector<double>& _stateVector);

        //Установка новго состояния точки
        bool set_state(std::vector<double>& _stateVector);

        virtual bool STEP(double dt); //Функция шага по времени 
        bool STEP(double dt, std::vector<double> dr_dt, std::vector<double> dV_dt); //Шаг по времени с заданием производных

        virtual ~PointMass();   

        bool set_forces(std::vector<double>& _forces); //Установка текущих сил, действующих на центр масс

        
        double get_m() const; //Вернуть значение массы
        double get_x() const; //Вернуть значение координаты х
        double get_y() const; //Вернуть значение координаты y
        double get_z() const; //Вернуть значение координаты z
        double get_Vx() const; //Вернуть значение скорости Vx
        double get_Vy() const; //Вернуть значение скорости Vy
        double get_Vz() const; //Вернуть значение скорости Vz
        double get_Vabs() const; //Вернуть значение модуля скорости
        double get_rabs() const; //Вернуть значение модуля радиус-вектора

        std::vector<double> get_r() const; //Вернуть значение радиус-вектора
        std::vector<double> get_V() const; //Вернуть значение вектора скорости
        std::vector<double> get_stateVector() const; //Вернуть значения вектора-состояния
        std::vector<double> get_forces() const; //Вернуть значение сил
        std::vector<double> get_dr_dt() const; //Вернуть значение производной радиус-вектора
        std::vector<double> get_dV_dt() const; //Вернуть значение производной вектора скорости

    protected:

        std::vector<double> stateVector; //Вектор состояния (x, y, z, Vx, Vy, Vz)
        std::vector<double> forces; //Вектор сил (Fx, Fy, Fz)
        double m; //Масса
        bool forcesUpToDate; //Флаг на проверку того, что после вызова шага были обновлены силы

        double dx_dt() const; //Производная х
        double dy_dt() const; //Производная y 
        double dz_dt() const; //Производная z
        double dVx_dt() const; //Производная Vx
        double dVy_dt() const; //Производная Vy
        double dVz_dt() const; //Производная Vz
        //double dm_dt();
};

#endif