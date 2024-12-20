#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/PointMass.hpp"

using namespace std;

PointMass::PointMass(): stateVector(vector<double>(6)), forces(vector<double>(3)), forcesUpToDate(false){
}

PointMass::~PointMass(){}

bool PointMass::init(double _m, vector<double>& _stateVector){
    if(_stateVector.size() != stateVector.size()){
        cout << "Ошибка при инициализации вектора состояния. Размер задаваемого вектора состояния неправильный\n";
        return false;
    }
    m = _m;
    for(int i = 0; i < stateVector.size(); i++){
        stateVector[i] = _stateVector[i];
    }
    return true;
}

bool PointMass::STEP(double dt){
    if(dt <= 0){
        cout << "Ошибка при совершении шага мат точки. Отрицательное dt\n";
        return false; 
    }

    if(!forcesUpToDate){
        cout << "Ошибка при совершении шага мат точки. Силы не были обновлены\n";
        return false;
    }

    stateVector[0] += dx_dt() * dt;
    stateVector[1] += dy_dt() * dt;
    stateVector[2] += dz_dt() * dt;
    stateVector[3] += dVx_dt() * dt;
    stateVector[4] += dVy_dt() * dt;
    stateVector[5] += dVz_dt() * dt;

    forcesUpToDate = false;
    
    return true;
}

bool PointMass::set_forces(vector<double>& _forces){
    if(_forces.size() != forces.size()){
        cout << "Ошибка при установке новых сил. Размер нового вектора сил не равен старому \n";
        return false;
    }

    if(forcesUpToDate){
        cout << "Ошибка. Обновление сил дважды подряд без совершения шага\n";
        return false;
    }

    for(int i = 0; i < forces.size(); i++){
        forces[i] = _forces[i];
    }

    forcesUpToDate = true;
    return true;
}

double PointMass::get_m() const{
    return m;
}

double PointMass::get_x() const{
    return stateVector[0];
}

double PointMass::get_y() const{
    return stateVector[1];
}

double PointMass::get_z() const{
    return stateVector[2];
}

double PointMass::get_Vx() const{
    return stateVector[3];
}

double PointMass::get_Vy() const{
    return stateVector[4];
}

double PointMass::get_Vz() const{
    return stateVector[5];
}

double PointMass::get_Vabs() const{
    return sqrt(get_Vx() * get_Vx() + get_Vy() * get_Vy() + get_Vz() * get_Vz());
}

double PointMass::get_rabs() const{
    return sqrt( get_x() * get_x() + get_y() * get_y() + get_z() * get_z());
}

vector<double> PointMass::get_r() const{
    vector<double> _r(3);
    for(int i = 0; i < _r.size(); i++){
        _r[i] = stateVector[i];
    }
    return _r;
}

vector<double> PointMass::get_V() const{
    vector<double> _V(3);
    for(int i = 0; i < _V.size(); i++){
        _V[i] = stateVector[3+i];
    }
    return _V;
}

vector<double> PointMass::get_forces() const{
    return forces;
}

double PointMass::dx_dt() const{
    return stateVector[3];
}

double PointMass::dy_dt() const{
    return stateVector[4];
}

double PointMass::dz_dt() const{
    return stateVector[5];
}

double PointMass::dVx_dt() const{
    if(!forcesUpToDate){
        throw std::runtime_error("Try to get not valid derivates. Forces are not upToDate\n");
    }
    return forces[0] / m;
}

double PointMass::dVy_dt() const{
    if(!forcesUpToDate){
        throw std::runtime_error("Try to get not valid derivates. Forces are not upToDate\n");
    }
    return forces[1] / m;
}

double PointMass::dVz_dt() const{
    if(!forcesUpToDate){
        throw std::runtime_error("Try to get not valid derivates. Forces are not upToDate\n");
    }
    return forces[2] / m;
}

bool PointMass::set_state(vector<double>& _stateVector){
    if(_stateVector.size() != stateVector.size()){
        throw std::runtime_error("Exception in PointMass::set_state(vector<double>& _stateVector). Wrong _stateVector size!\n");
        return false;
    }
    for(int i = 0; i < stateVector.size(); i++){
        stateVector[i] = _stateVector[i];
    }
    forcesUpToDate = false;
    return true;
}

vector<double> PointMass::get_stateVector() const{
    return stateVector;
}

vector<double> PointMass::get_dr_dt() const{
    vector<double> _dr_dt(3);
    _dr_dt[0] = dx_dt();
    _dr_dt[1] = dy_dt();
    _dr_dt[2] = dz_dt();

    return _dr_dt;
}

vector<double> PointMass::get_dV_dt() const{
    if(!forcesUpToDate){
        throw std::runtime_error("Try to get not valid derivates. Forces are not upToDate\n");
    }
    vector<double> _dV_dt(3);
    _dV_dt[0] = dVx_dt();
    _dV_dt[1] = dVy_dt();
    _dV_dt[2] = dVz_dt();
    return _dV_dt;
}

bool PointMass::STEP(double dt, std::vector<double> dr_dt, std::vector<double> dV_dt){
    for(int i = 0; i < 3; i++){
        stateVector[i] += dr_dt[i] * dt;
        stateVector[i + 3] += dV_dt[i] * dt;
    }
    forcesUpToDate = false;
    return true;
}

PointMass::PointMass(PointMass &_pointMass){
    std::vector<double> stateVector; //Вектор состояния (x, y, z, Vx, Vy, Vz)
    std::vector<double> forces; //Вектор сил (Fx, Fy, Fz)
    double m; //Масса
    bool forcesUpToDate; //Флаг на проверку того, что после вызова шага были обновлены силы
}