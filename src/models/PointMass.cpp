#define _USE_MATH_DEFINES
#include <iostream>
#include <math.h>
#include <vector>
#include "../../include/models/PointMass.hpp"

using namespace std;

PointMass::PointMass(): stateVector(vector<double>(6)), forces(vector<double>(3)), forcesUpToDate(false){
}

PointMass::~PointMass(){}

bool PointMass::init(double _m, vector<double> _stateVector){
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

bool PointMass::set_forces(vector<double> _forces){
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
    return forces[0] / m;
}

double PointMass::dVy_dt() const{
    return forces[1] / m;
}

double PointMass::dVz_dt() const{
    return forces[2] / m;
}